/* DroneCAN 采集服务接口，由主循环和文件操作后的服务点调用。 */
#ifndef ACQUISITION_H
#define ACQUISITION_H

#include "log_record.h"

typedef struct {
    uint32_t allocations;     /* 完整 UID 匹配后，最终分配应答成功入队次数。 */
    uint32_t gps_messages;    /* 解码成功的完整 Fix2 数，包含 NO_FIX。 */
    uint32_t air_messages;    /* 抽取之前的完整 RawAirData 数，包含 NaN 压差。 */
    uint32_t crc_errors;      /* libcanard 报告的传输 CRC 错误数。 */
    uint32_t transport_errors; /* 其他接收传输错误，未订阅消息不计入。 */
    uint32_t dsdl_errors;     /* 生成的 DSDL 解码器报告的失败数。 */
    uint32_t tx_errors;       /* 发送入队失败或软件队列帧超过截止时间的次数。 */
    uint32_t gps_restarts;
    uint32_t air_restarts;
    bool gps_online;
    bool air_online;
} AcquisitionStats;

/* 状态灯使用实时完整报文状态，独立于日志候选及抽取计数。 */
typedef struct {
    uint64_t heartbeat_us;
    uint64_t sample_us;
    uint8_t mode;
    bool online;
    bool has_sample;
} AcquisitionNodeStatus;

typedef struct {
    AcquisitionNodeStatus gps;
    AcquisitionNodeStatus air;
    uint64_t utc_us;
    uint64_t time_received_us;
    uint32_t time_generation; /* GPS 重启/掉线/重新分配时递增，取消旧报时脉冲。 */
    uint8_t fix_status;
    uint8_t satellites;
    bool has_time;
} AcquisitionStatus;

/* 初始化一次，log 指向整个启动期间共享的静态日志对象。 */
void acquisition_init(LogRecords *log, uint64_t start_us);
/* 排空接收队列、推进窗口、发布心跳并尝试排空发送队列。 */
void acquisition_service(void);
/* 写卡服务用它判断当前是否需要优先处理节点分配应答。 */
bool acquisition_allocation_pending(void);
AcquisitionStats acquisition_stats(void);
AcquisitionStatus acquisition_status(void);
void acquisition_stop(void); /* 停止新窗口组帧，保留已封帧队列供写卡服务排空。 */

#endif
