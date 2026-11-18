/* 写卡服务状态、故障诊断和主循环入口。 */
#ifndef SD_LOG_H
#define SD_LOG_H

#include "ff.h"
#include "log_record.h"

typedef enum {
    SD_LOG_WAITING, /* 启动时等待首个记录，或正分步创建第一个文件。 */
    SD_LOG_OPEN,    /* 文件头已同步，可以写数据记录。 */
    SD_LOG_FAULT,   /* 故障后的等待重试及恢复建文件阶段。 */
    SD_LOG_CLOSED   /* 停录且成功关闭文件，或停录时无需创建文件。 */
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
    SdLogOperation failed_op; /* 最近一次故障的阶段，恢复后保留供诊断。 */
    FRESULT last_error;       /* 最近一次故障的 FatFs 返回值。 */
    bool short_write;        /* FR_OK 但实际写入少于请求字节数。 */
    unsigned file_number;
    uint32_t failures;
    uint32_t uncertain_records; /* 故障时尚未确认同步成功的累计记录数。 */
    uint64_t max_mount_us;      /* 以下最长耗时均为微秒，包含故障操作。 */
    uint64_t max_write_us;
    uint64_t max_sync_us;
} SdLogStats;

/* 每轮推进写卡状态，stop_requested 为真时优先排空已封帧队列并关闭文件。 */
void sd_log_service(LogRecords *log, bool stop_requested);
const SdLogStats *sd_log_stats(void);

#endif
