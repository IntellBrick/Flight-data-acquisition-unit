/*
 * MicroSD 文件记录状态机。
 * 主循环每轮调用一次，分步挂载、建文件、写头、同步，再批量消费日志队列。
 * 各文件操作返回后及时调用 CAN 服务，故障后保留采集时间轴并延时重试。
 */
#include "sd_log.h"

#include <stdio.h>
#include <string.h>

#include "acquisition.h"
#include "board.h"

#define RETRY_US 2000000U       /* 存储故障后等待约 2 秒再尝试挂载。 */
#define SYNC_US 2000000U        /* 有未同步记录时，约每 2 秒执行文件同步。 */
#define WRITE_WAIT_US 1000000U  /* 最早记录封帧后等待约 1 秒触发写入。 */
#define BATCH_BYTES 4096U
#define BATCH_RECORDS (BATCH_BYTES / LOG_RECORD_SIZE) /* 4096 / 80，单批最多 51 帧。 */
#define LAST_FILE_NUMBER 9999U

static FATFS filesystem;
static FIL file;
static uint8_t batch[BATCH_BYTES];
static SdLogStats stats;
static unsigned next_file = 1;
static unsigned names_checked;
static unsigned unsynced_records; /* 已尝试写入、尚未成功同步的记录数。 */
static uint64_t retry_at;
static uint64_t last_sync;
static SdLogOperation open_step = SD_OP_MOUNT; /* 与公开存储状态分开的建文件子阶段。 */

/* 统计本次文件操作耗时，然后优先处理该操作期间中断保存的 CAN 帧。 */
static void after_io(uint64_t start, uint64_t *max_us)
{
    uint64_t elapsed = board_time_us() - start;
    if (max_us != NULL && elapsed > *max_us) {
        *max_us = elapsed;
    }
    acquisition_service();
}

/* 记录故障阶段与结果，将未确认持久化的记录纳入 uncertain 统计。 */
static void storage_fault(SdLogOperation op, FRESULT result, bool short_write)
{
    stats.state = SD_LOG_FAULT;
    stats.data_written = false;
    stats.failed_op = op;
    stats.last_error = result;
    stats.short_write = short_write;
    ++stats.failures;
    stats.uncertain_records += unsynced_records;
    unsynced_records = 0;
    open_step = SD_OP_MOUNT;

    /* 注销故障卷，将已尝试的批次留在原文件中由离线 CRC 校验判断有效性。 */
    (void)f_mount(NULL, "0:", 0);
    acquisition_service();
    retry_at = board_time_us() + RETRY_US;
}

/* f_sync() 下层会调用 CTRL_SYNC，确认 FAT、目录及数据缓存提交并等待卡就绪。 */
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

/* 每次只执行一个子阶段，下一轮主循环先服务 CAN 再继续建文件。 */
static void open_file_step(void)
{
    uint64_t start = board_time_us();
    FRESULT result;
    switch (open_step) {
    case SD_OP_MOUNT:
        /* 挂载成功后再次检查卷类型，当前工程使用 FAT32。 */
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
        /* 从下一个候选名开始扫描，最多检查 9999 个名字并跳过已有文件。 */
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
            stats.data_written = false;
            stats.file_number = file_number;
            open_step = SD_OP_HEADER;
        } else if (result != FR_EXIST) {
            storage_fault(SD_OP_OPEN, result, false);
        }
        break;
    }
    case SD_OP_HEADER: {
        /* 文件头写完后还需在下一阶段同步，成功后才进入数据写入状态。 */
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

/* 只打包完整的 80 字节记录，同时检查 FatFs 返回值与实际写入字节数。 */
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
    /* 先消费已尝试批次再处理 CAN，以便新封帧可使用释放的队列槽位。
     * 写入失败时该批次也已退队，故障恢复从后续记录继续，避免重复写入。 */
    log_consume(log, count);
    unsynced_records += count;
    after_io(start, &stats.max_write_us);
    if (result != FR_OK || written != size) {
        storage_fault(SD_OP_WRITE, result, result == FR_OK);
        return false;
    }
    stats.data_written = true;
    return true;
}

/* 从最早记录的小端时间戳求其计划封帧时刻，作为批量等待时间的基准。 */
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
        /* 已关文件时结束写卡服务，待发分配应答优先获得发送机会。 */
        return;
    }
    uint64_t now = board_time_us();
    if (stats.state != SD_LOG_OPEN) {
        if (open_step == SD_OP_MOUNT && log->count == 0) {
            if (stop_requested) {
                stats.state = SD_LOG_CLOSED;
            }
        } else if (now >= retry_at) {
            /* 首次收到已封帧记录才开始挂载，故障重试遵守 retry_at。 */
            open_file_step();
        }
        return;
    }
    if (log->count != 0 &&
        (stop_requested || log->count >= BATCH_RECORDS || now >= oldest_sealed_at(log) + WRITE_WAIT_US)) {
        if (!write_batch(log)) {
            return;
        }
        /* 返回主循环发布最新传感器/写卡状态，再开始下一次同步或关闭操作。 */
        return;
    }
    if (acquisition_allocation_pending()) {
        return;
    }
    if (stop_requested && log->count == 0) {
        /* 已封帧队列排空后关闭，f_close() 自带同步，成功后进入 CLOSED。 */
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

/* 主循环读取静态统计对象，最大耗时单位为微秒。 */
const SdLogStats *sd_log_stats(void)
{
    return &stats;
}
