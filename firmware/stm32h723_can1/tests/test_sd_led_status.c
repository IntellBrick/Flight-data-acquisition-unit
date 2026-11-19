/* 验证 LED 所用的“当前文件已有数据”状态随写入、故障和重挂载变化。 */
#include <assert.h>
#include "board.h"
#include "sd_log.h"

static uint64_t now_us = 1400000;
static bool short_write;
static bool sync_error;

uint64_t board_time_us(void) { return now_us; }
void acquisition_service(void) {}
bool acquisition_allocation_pending(void) { return false; }

FRESULT f_mount(FATFS *fs, const TCHAR *path, BYTE immediate)
{
    (void)path; (void)immediate;
    if (fs != NULL) {
        fs->fs_type = FS_FAT32;
    }
    return FR_OK;
}

FRESULT f_open(FIL *file, const TCHAR *path, BYTE mode)
{
    (void)file; (void)path;
    assert(mode == (FA_WRITE | FA_CREATE_NEW));
    return FR_OK;
}

FRESULT f_write(FIL *file, const void *data, UINT count, UINT *written)
{
    (void)file; (void)data;
    *written = short_write ? count - 1 : count;
    return FR_OK;
}

FRESULT f_sync(FIL *file)
{
    (void)file;
    return sync_error ? FR_DISK_ERR : FR_OK;
}

FRESULT f_close(FIL *file) { (void)file; return FR_OK; }

static void create_file(LogRecords *log)
{
    for (unsigned i = 0; i < 4; ++i) {
        sd_log_service(log, false);
    }
    assert(sd_log_stats()->state == SD_LOG_OPEN);
    assert(!sd_log_stats()->data_written); /* 文件头和文件头同步均不算传感器数据。 */
}

int main(void)
{
    static LogRecords log;
    log_start(&log, 0);
    log_advance(&log, now_us);
    create_file(&log);
    sd_log_service(&log, false);
    assert(sd_log_stats()->data_written && log.count == 0);

    now_us += 2200000;
    sync_error = true;
    sd_log_service(&log, false);
    assert(sd_log_stats()->state == SD_LOG_FAULT && !sd_log_stats()->data_written);
    sync_error = false;
    now_us += 2000000;
    log_advance(&log, now_us);
    create_file(&log);
    sd_log_service(&log, false);
    assert(sd_log_stats()->file_number == 2 && sd_log_stats()->data_written);

    now_us += 2000000;
    log_advance(&log, now_us);
    short_write = true;
    sd_log_service(&log, false);
    assert(sd_log_stats()->state == SD_LOG_FAULT && !sd_log_stats()->data_written);
    assert(sd_log_stats()->short_write);
    short_write = false;
    now_us += 2000000;
    log_advance(&log, now_us);
    create_file(&log);
    sd_log_service(&log, true);
    assert(sd_log_stats()->data_written);
    sd_log_service(&log, true);
    assert(sd_log_stats()->state == SD_LOG_CLOSED && log.count == 0);
    return 0;
}
