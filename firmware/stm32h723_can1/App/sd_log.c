#include "sd_log.h"

#include <stdio.h>
#include <string.h>

#include "acquisition.h"
#include "board.h"

#define RETRY_US 2000000U
#define SYNC_US 2000000U
#define WRITE_WAIT_US 1000000U
#define BATCH_BYTES 4096U
#define BATCH_RECORDS (BATCH_BYTES / LOG_RECORD_SIZE)
#define LAST_FILE_NUMBER 9999U

static FATFS filesystem;
static FIL file;
static uint8_t batch[BATCH_BYTES];
static SdLogStats stats;
static unsigned next_file = 1;
static unsigned names_checked;
static unsigned unsynced_records;
static uint64_t retry_at;
static uint64_t last_sync;
static SdLogOperation open_step = SD_OP_MOUNT;

static void after_io(uint64_t start, uint64_t *max_us)
{
    uint64_t elapsed = board_time_us() - start;
    if (max_us != NULL && elapsed > *max_us) {
        *max_us = elapsed;
    }
    acquisition_service();
}

static void storage_fault(SdLogOperation op, FRESULT result, bool short_write)
{
    stats.state = SD_LOG_FAULT;
    stats.failed_op = op;
    stats.last_error = result;
    stats.short_write = short_write;
    ++stats.failures;
    stats.uncertain_records += unsynced_records;
    unsynced_records = 0;
    open_step = SD_OP_MOUNT;

    // Unregister the failed volume without syncing an uncertain file a second time.
    (void)f_mount(NULL, "0:", 0);
    acquisition_service();
    retry_at = board_time_us() + RETRY_US;
}

static bool sync_file(void)
{
    uint64_t start = board_time_us();
    FRESULT result = f_sync(&file);
    last_sync = board_time_us();
    after_io(start, &stats.max_sync_us);
    if (result != FR_OK) {
        storage_fault(SD_OP_SYNC, result, false);
        return false;
    }
    unsynced_records = 0;
    return true;
}

static void open_file_step(void)
{
    uint64_t start = board_time_us();
    FRESULT result;
    switch (open_step) {
    case SD_OP_MOUNT:
        result = f_mount(&filesystem, "0:", 1);
        after_io(start, &stats.max_mount_us);
        if (result != FR_OK || filesystem.fs_type != FS_FAT32) {
            storage_fault(SD_OP_MOUNT, result != FR_OK ? result : FR_NO_FILESYSTEM, false);
        } else {
            names_checked = 0;
            open_step = SD_OP_OPEN;
        }
        break;
    case SD_OP_OPEN: {
        if (names_checked == LAST_FILE_NUMBER) {
            storage_fault(SD_OP_OPEN, FR_DENIED, false);
            break;
        }
        char name[16];
        unsigned file_number = next_file;
        (void)snprintf(name, sizeof name, "0:LOG%04u.BIN", file_number);
        result = f_open(&file, name, FA_WRITE | FA_CREATE_NEW);
        acquisition_service();
        if (result == FR_OK || result == FR_EXIST) {
            ++names_checked;
            next_file = file_number == LAST_FILE_NUMBER ? 1 : file_number + 1;
        }
        if (result == FR_OK) {
            stats.file_number = file_number;
            open_step = SD_OP_HEADER;
        } else if (result != FR_EXIST) {
            storage_fault(SD_OP_OPEN, result, false);
        }
        break;
    }
    case SD_OP_HEADER: {
        UINT written = 0;
        result = f_write(&file, log_header, LOG_HEADER_SIZE, &written);
        after_io(start, &stats.max_write_us);
        if (result != FR_OK || written != LOG_HEADER_SIZE) {
            storage_fault(SD_OP_HEADER, result, result == FR_OK);
        } else {
            open_step = SD_OP_SYNC;
        }
        break;
    }
    case SD_OP_SYNC:
        if (sync_file()) {
            stats.state = SD_LOG_OPEN;
        }
        break;
    default:
        break;
    }
}

static bool write_batch(LogRecords *log)
{
    unsigned count = log->count < BATCH_RECORDS ? log->count : BATCH_RECORDS;
    UINT size = count * LOG_RECORD_SIZE;
    for (unsigned i = 0; i < count; ++i) {
        memcpy(batch + i * LOG_RECORD_SIZE, log_peek(log, i), LOG_RECORD_SIZE);
    }
    uint64_t start = board_time_us();
    UINT written = 0;
    FRESULT result = f_write(&file, batch, size, &written);
    // Retire the attempted batch before CAN service can enqueue more windows.
    log_consume(log, count);
    unsynced_records += count;
    after_io(start, &stats.max_write_us);
    if (result != FR_OK || written != size) {
        storage_fault(SD_OP_WRITE, result, result == FR_OK);
        return false;
    }
    return true;
}

static uint64_t oldest_sealed_at(const LogRecords *log)
{
    const uint8_t *record = log_peek(log, 0);
    uint64_t start = 0;
    for (unsigned i = 0; i < 8; ++i) {
        start |= (uint64_t)record[i] << (8 * i);
    }
    return start + LOG_PERIOD_US + LOG_GRACE_US;
}

void sd_log_service(LogRecords *log, bool stop_requested)
{
    if (stats.state == SD_LOG_CLOSED || acquisition_allocation_pending()) {
        return;
    }
    uint64_t now = board_time_us();
    if (stats.state != SD_LOG_OPEN) {
        if (open_step == SD_OP_MOUNT && log->count == 0) {
            if (stop_requested) {
                stats.state = SD_LOG_CLOSED;
            }
        } else if (now >= retry_at) {
            open_file_step();
        }
        return;
    }
    if (log->count != 0 &&
        (stop_requested || log->count >= BATCH_RECORDS || now >= oldest_sealed_at(log) + WRITE_WAIT_US)) {
        if (!write_batch(log)) {
            return;
        }
    }
    if (acquisition_allocation_pending()) {
        return;
    }
    if (stop_requested && log->count == 0) {
        uint64_t start = board_time_us();
        FRESULT result = f_close(&file);
        after_io(start, &stats.max_sync_us);
        if (result != FR_OK) {
            storage_fault(SD_OP_CLOSE, result, false);
        } else {
            stats.state = SD_LOG_CLOSED;
            unsynced_records = 0;
        }
    } else if (unsynced_records != 0 && board_time_us() - last_sync >= SYNC_US) {
        (void)sync_file();
    }
}

const SdLogStats *sd_log_stats(void)
{
    return &stats;
}
