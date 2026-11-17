#include "log_record.h"

#include <float.h>
#include <math.h>
#include <string.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
               FLT_MAX_EXP == 128, "Log v1 requires IEEE 754 binary32");

const uint8_t log_header[LOG_HEADER_SIZE] = {
    'F', 'D', 'A', 'U', 'L', 'O', 'G', 0, 1, 0, LOG_RECORD_SIZE, 0, 0x40, 0x0D, 3, 0
};

static void put_le(uint8_t *out, uint64_t value, unsigned size)
{
    for (unsigned i = 0; i < size; ++i) {
        out[i] = (uint8_t)value;
        value >>= 8;
    }
}

static void put_float(uint8_t *out, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof bits);
    put_le(out, bits, 4);
}

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
    memset(out, 0, LOG_RECORD_SIZE);
    put_le(out, window->start_us, 8);
    if (air->node_id != 0) {
        put_le(out + 8, air->time_us - window->start_us, 4);
        out[16] = air->node_id;
        out[17] = 1;
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
        out[68] = 1;
        if ((gps->status == 2 || gps->status == 3) &&
            gps->latitude_e8 >= -INT64_C(9000000000) && gps->latitude_e8 <= INT64_C(9000000000) &&
            gps->longitude_e8 >= -INT64_C(18000000000) && gps->longitude_e8 <= INT64_C(18000000000)) {
            put_le(out + 24, (uint64_t)gps->latitude_e8, 8);
            put_le(out + 32, (uint64_t)gps->longitude_e8, 8);
            if (gps->status == 3) {
                put_le(out + 40, (uint32_t)gps->altitude_mm, 4);
            }
            out[68] |= 2;
        }
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
    put_le(out + 72, sequence, 4);
    put_le(out + 76, record_crc(out), 4);
}

void log_start(LogRecords *log, uint64_t now_us)
{
    log->first_start_us = ((now_us + LOG_PERIOD_US - 1) / LOG_PERIOD_US) * LOG_PERIOD_US;
    log->current.start_us = log->first_start_us;
    log->collecting = true;
}

static void seal_window(LogRecords *log)
{
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
    // Called before decoding a frame, so a transfer ending at the deadline is already late.
    while (true) {
        if (log->has_previous && now_us >= log->previous.start_us + LOG_PERIOD_US + LOG_GRACE_US) {
            seal_window(log);
            log->has_previous = false;
        }
        uint64_t next_start = log->current.start_us + LOG_PERIOD_US;
        if (now_us < next_start) {
            break;
        }
        log->previous = log->current;
        log->has_previous = true;
        log->current = (LogWindow){.start_us = next_start};
    }
}

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

void log_add_air(LogRecords *log, const AirSample *sample)
{
    LogWindow *window = find_window(log, sample->time_us, &log->air_late);
    if (window != NULL && sample->time_us >= window->air.time_us) {
        window->air = *sample;
    }
}

void log_add_gps(LogRecords *log, const GpsSample *sample)
{
    LogWindow *window = find_window(log, sample->time_us, &log->gps_late);
    if (window != NULL && sample->time_us >= window->gps.time_us) {
        window->gps = *sample;
    }
}

void log_reset_air(LogRecords *log)
{
    log->current.air = (AirSample){0};
    log->previous.air = (AirSample){0};
}

void log_reset_gps(LogRecords *log)
{
    log->current.gps = (GpsSample){0};
    log->previous.gps = (GpsSample){0};
}

const uint8_t *log_peek(const LogRecords *log, unsigned index)
{
    return log->records[(log->head + index) % LOG_QUEUE_CAPACITY];
}

void log_consume(LogRecords *log, unsigned count)
{
    log->head = (log->head + count) % LOG_QUEUE_CAPACITY;
    log->count -= count;
}
