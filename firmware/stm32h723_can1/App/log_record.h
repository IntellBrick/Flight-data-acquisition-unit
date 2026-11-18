/* 样本、窗口和日志队列的数据定义，所有时间统一使用微秒。 */
#ifndef LOG_RECORD_H
#define LOG_RECORD_H

#include <stdbool.h>
#include <stdint.h>

#define LOG_PERIOD_US 200000U /* 每 200 ms 一个记录窗口。 */
#define LOG_GRACE_US 20000U   /* 窗口结束后保留 20 ms 完成跨界多帧报文。 */
#define LOG_HEADER_SIZE 16U
#define LOG_RECORD_SIZE 80U
#define LOG_QUEUE_CAPACITY 64U

typedef struct {
    uint64_t time_us;  /* 被抽中完整报文的首帧接收时间。 */
    float pressure_pa; /* 原始压差，单位 Pa，负值和 NaN 可传入编码器。 */
    uint8_t node_id;   /* 0 表示本窗口尚无候选。 */
} AirSample;

typedef struct {
    uint64_t time_us;      /* Fix2 首帧的本地接收时间。 */
    uint64_t gnss_time_us; /* 设备 GNSS 时间，时标由 time_standard 指定。 */
    int64_t latitude_e8;   /* 纬度整数，乘以 10^-8 得到度。 */
    int64_t longitude_e8;  /* 经度整数，乘以 10^-8 得到度。 */
    int32_t altitude_mm;   /* 平均海平面高度，单位 mm。 */
    float velocity[3];    /* 北、东、地三个分量，单位 m/s，地向下为正。 */
    uint8_t node_id;       /* 0 表示本窗口尚无候选。 */
    uint8_t status;        /* 0 无定位、1 仅时间、2 二维定位、3 三维定位。 */
    uint8_t satellites;    /* 参与定位解算的卫星数。 */
    uint8_t time_standard; /* 0 NONE、1 TAI、2 UTC、3 GPS。 */
} GpsSample;

typedef struct {
    uint64_t start_us; /* 半开时间窗口的起点，窗口为 [start_us, start_us + 周期)。 */
    AirSample air;
    GpsSample gps;
} LogWindow;

typedef struct {
    LogWindow current;   /* 当前窗口。 */
    LogWindow previous;  /* 正在等待重组余量结束的上一窗口。 */
    uint64_t first_start_us; /* 本次启动的记录起点 T0。 */
    bool has_previous;   /* previous 是否仍可接收候选。 */
    bool collecting;     /* 是否正在推进记录窗口。 */
    /* 数组中保存已编码的完整 80 字节帧，入队后内容与窗口对象独立。 */
    uint8_t records[LOG_QUEUE_CAPACITY][LOG_RECORD_SIZE];
    unsigned head;       /* 已封帧队列的队头下标。 */
    unsigned count;      /* 当前待写卡的完整帧数。 */
    unsigned high_water; /* 历史最大队列长度。 */
    uint32_t next_sequence; /* 下一窗口序号，即使队列满导致丢帧也递增。 */
    uint32_t dropped;    /* 日志队列满时丢弃的新记录数。 */
    uint32_t air_missing; /* 封帧时没有空速候选的累计窗口数。 */
    uint32_t gps_missing; /* 封帧时没有 GPS 候选的累计窗口数。 */
    uint32_t air_late;    /* 首帧属于已封窗口的空速候选数。 */
    uint32_t gps_late;    /* 首帧属于已封窗口的 GPS 候选数。 */
} LogRecords;

extern const uint8_t log_header[LOG_HEADER_SIZE];

void log_encode(uint8_t out[LOG_RECORD_SIZE], const LogWindow *window, uint32_t sequence);
void log_start(LogRecords *log, uint64_t now_us);
/* 解码前传入该 CAN 帧时间，接收缓冲全空后才可传入当前时间补推。 */
void log_advance(LogRecords *log, uint64_t now_us);
void log_add_air(LogRecords *log, const AirSample *sample);
void log_add_gps(LogRecords *log, const GpsSample *sample);
void log_reset_air(LogRecords *log);
void log_reset_gps(LogRecords *log);
const uint8_t *log_peek(const LogRecords *log, unsigned index); /* index 小于当前 count。 */
void log_consume(LogRecords *log, unsigned count); /* count 不超过当前队列长度。 */

#endif
