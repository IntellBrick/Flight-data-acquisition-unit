/*
 * 200 ms 时间窗口和 FDAULOG v1 二进制编码。
 * 本模块由主循环单线程访问，维护当前窗口、前一待封窗口及 64 帧日志队列。
 */
#include "log_record.h"

#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "Log v1 requires IEEE 754 binary32");

/* 文件头依次为 magic、版本 1、记录长度 80、周期 200000，均按小端保存。 */
const uint8_t log_header[LOG_HEADER_SIZE] = {
    'F', 'D', 'A', 'U', 'L', 'O', 'G', 0, 1, 0, LOG_RECORD_SIZE, 0, 0x40, 0x0D, 3, 0
};

/* 显式按字节编码整数，避免结构体填充或 CPU 内存布局影响文件格式。 */
static void put_le(uint8_t *out, uint64_t value, unsigned size)
{
    for (unsigned i = 0; i < size; ++i) {
        out[i] = (uint8_t)value;
        value >>= 8;
    }
}

/* 用 memcpy 取出浮点原始位，满足别名规则，再按小端保存 IEEE 754 binary32。 */
static void put_float(uint8_t *out, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof bits);
    put_le(out, bits, 4);
}

/* CRC-32/ISO-HDLC 覆盖帧内 0～75 字节，初值和最终异或值均为全 1。 */
static uint32_t record_crc(const uint8_t *data)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < 76; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1U) ? UINT32_C(0xEDB88320) : 0U);
        }
    }
    return ~crc;
}

void log_encode(uint8_t out[LOG_RECORD_SIZE], const LogWindow *window, uint32_t sequence)
{
    const AirSample *air = &window->air;
    const GpsSample *gps = &window->gps;
    /* 缺样块、无效测量和所有保留位默认是 0，再填写满足有效性条件的字段。 */
    memset(out, 0, LOG_RECORD_SIZE);
    put_le(out, window->start_us, 8);
    if (air->node_id != 0) {
        put_le(out + 8, air->time_us - window->start_us, 4);
        out[16] = air->node_id;
        out[17] = 1; /* air_flags 位 0 表示收到候选，位 1 表示压差为有限值。 */
        if (isfinite(air->pressure_pa)) {
            put_float(out + 12, air->pressure_pa);
            out[17] |= 2;
        }
    }
    if (gps->node_id != 0) {
        put_le(out + 20, gps->time_us - window->start_us, 4);
        out[64] = gps->node_id;
        out[65] = gps->status;
        out[66] = gps->satellites;
        out[67] = gps->time_standard;
        out[68] = 1; /* gps_flags 位 0 表示收到候选，其余各位独立表示测量有效。 */
        if ((gps->status == 2 || gps->status == 3) &&
            gps->latitude_e8 >= -INT64_C(9000000000) && gps->latitude_e8 <= INT64_C(9000000000) &&
            gps->longitude_e8 >= -INT64_C(18000000000) && gps->longitude_e8 <= INT64_C(18000000000)) {
            put_le(out + 24, (uint64_t)gps->latitude_e8, 8);
            put_le(out + 32, (uint64_t)gps->longitude_e8, 8);
            if (gps->status == 3) {
                /* MSL 高度仅在经纬度合法且为 3D_FIX 时写入。 */
                put_le(out + 40, (uint32_t)gps->altitude_mm, 4);
            }
            out[68] |= 2;
        }
        /* GNSS 授时有效性独立于定位状态，允许 NO_FIX 下仍保留有效授时。 */
        if (gps->gnss_time_us != 0 && gps->time_standard >= 1 && gps->time_standard <= 3) {
            put_le(out + 56, gps->gnss_time_us, 8);
            out[68] |= 4;
        }
        if (gps->status == 3 &&
            isfinite(gps->velocity[0]) && isfinite(gps->velocity[1]) && isfinite(gps->velocity[2])) {
            for (unsigned i = 0; i < 3; ++i) {
                put_float(out + 44 + 4 * i, gps->velocity[i]);
            }
            out[68] |= 8;
        }
    }
    /* 固定尾部为窗口序号与 CRC，偏移需要与 PC 导出脚本保持一致。 */
    put_le(out + 72, sequence, 4);
    put_le(out + 76, record_crc(out), 4);
}

/* 向上取整到第一个 200 ms 边界，边界前的样本忽略，首窗口序号从 0 起。 */
void log_start(LogRecords *log, uint64_t now_us)
{
    log->first_start_us = ((now_us + LOG_PERIOD_US - 1) / LOG_PERIOD_US) * LOG_PERIOD_US;
    log->current.start_us = log->first_start_us;
    log->collecting = true;
}

static void seal_window(LogRecords *log)
{
    /* 先分配序号，再尝试入队，队列满时也留下可在文件中识别的序号缺口。 */
    uint32_t sequence = log->next_sequence++;
    log->air_missing += log->previous.air.node_id == 0;
    log->gps_missing += log->previous.gps.node_id == 0;
    if (log->count == LOG_QUEUE_CAPACITY) {
        ++log->dropped;
        return;
    }
    unsigned tail = (log->head + log->count) % LOG_QUEUE_CAPACITY;
    log_encode(log->records[tail], &log->previous, sequence);
    if (++log->count > log->high_water) {
        log->high_water = log->count;
    }
}

void log_advance(LogRecords *log, uint64_t now_us)
{
    if (!log->collecting) {
        return;
    }
    /* 每帧解码前调用，末帧时间达到窗口结束后 20 ms 时，旧窗口先封帧。 */
    while (true) {
        if (log->has_previous && now_us >= log->previous.start_us + LOG_PERIOD_US + LOG_GRACE_US) {
            seal_window(log);
            log->has_previous = false;
        }
        uint64_t next_start = log->current.start_us + LOG_PERIOD_US;
        if (now_us < next_start) {
            break;
        }
        /* 跨过 200 ms 边界后保留上一窗口，给首帧在边界前的传输留出重组余量。 */
        log->previous = log->current;
        log->has_previous = true;
        log->current = (LogWindow){.start_us = next_start};
    }
}

/* 候选样本按传输首帧时间归属，落在已封帧窗口中的候选累计为迟到。 */
static LogWindow *find_window(LogRecords *log, uint64_t time_us, uint32_t *late)
{
    if (!log->collecting || time_us < log->first_start_us) {
        return NULL;
    }
    if (time_us >= log->current.start_us) {
        return &log->current;
    }
    if (log->has_previous && time_us >= log->previous.start_us) {
        return &log->previous;
    }
    ++*late;
    return NULL;
}

/* 同窗口多候选时保留本地时间最新的一条，时间相同时采用后处理的一条。 */
void log_add_air(LogRecords *log, const AirSample *sample)
{
    LogWindow *window = find_window(log, sample->time_us, &log->air_late);
    if (window != NULL && sample->time_us >= window->air.time_us) {
        window->air = *sample;
    }
}

/* GPS 与空速分别更新各自数据块，两个传感器可在同一窗口保留不同偏移。 */
void log_add_gps(LogRecords *log, const GpsSample *sample)
{
    LogWindow *window = find_window(log, sample->time_us, &log->gps_late);
    if (window != NULL && sample->time_us >= window->gps.time_us) {
        window->gps = *sample;
    }
}

/* 节点重新分配、重启或离线时，清空两个未封窗口的空速候选。 */
void log_reset_air(LogRecords *log)
{
    log->current.air = (AirSample){0};
    log->previous.air = (AirSample){0};
}

/* 清空两个未封窗口的 GPS 候选，已编码记录仍可继续写卡。 */
void log_reset_gps(LogRecords *log)
{
    log->current.gps = (GpsSample){0};
    log->previous.gps = (GpsSample){0};
}

/* index 为从队头起的相对位置，调用者保证 index 小于 count。 */
const uint8_t *log_peek(const LogRecords *log, unsigned index)
{
    return log->records[(log->head + index) % LOG_QUEUE_CAPACITY];
}

/* 写卡批次尝试完成后释放这些槽位，调用者保证消费数不超过当前队列长度。 */
void log_consume(LogRecords *log, unsigned count)
{
    log->head = (log->head + count) % LOG_QUEUE_CAPACITY;
    log->count -= count;
}
