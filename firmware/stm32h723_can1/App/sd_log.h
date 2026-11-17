#ifndef SD_LOG_H
#define SD_LOG_H

#include "ff.h"
#include "log_record.h"

typedef enum {
    SD_LOG_WAITING,
    SD_LOG_OPEN,
    SD_LOG_FAULT,
    SD_LOG_CLOSED
} SdLogState;

typedef enum {
    SD_OP_NONE,
    SD_OP_MOUNT,
    SD_OP_OPEN,
    SD_OP_HEADER,
    SD_OP_WRITE,
    SD_OP_SYNC,
    SD_OP_CLOSE
} SdLogOperation;

typedef struct {
    SdLogState state;
    SdLogOperation failed_op;
    FRESULT last_error;
    bool short_write;
    unsigned file_number;
    uint32_t failures;
    uint32_t uncertain_records;
    uint64_t max_mount_us;
    uint64_t max_write_us;
    uint64_t max_sync_us;
} SdLogStats;

void sd_log_service(LogRecords *log, bool stop_requested);
const SdLogStats *sd_log_stats(void);

#endif
