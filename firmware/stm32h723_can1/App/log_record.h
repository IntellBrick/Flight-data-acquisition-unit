#ifndef LOG_RECORD_H
#define LOG_RECORD_H

#include <stdbool.h>
#include <stdint.h>

#define LOG_PERIOD_US 200000U
#define LOG_GRACE_US 20000U
#define LOG_HEADER_SIZE 16U
#define LOG_RECORD_SIZE 80U
#define LOG_QUEUE_CAPACITY 64U

typedef struct {
    uint64_t time_us;
    float pressure_pa;
    uint8_t node_id;
} AirSample;

typedef struct {
    uint64_t time_us;
    uint64_t gnss_time_us;
    int64_t latitude_e8;
    int64_t longitude_e8;
    int32_t altitude_mm;
    float velocity[3];
    uint8_t node_id;
    uint8_t status;
    uint8_t satellites;
    uint8_t time_standard;
} GpsSample;

typedef struct {
    uint64_t start_us;
    AirSample air;
    GpsSample gps;
} LogWindow;

typedef struct {
    LogWindow current;
    LogWindow previous;
    uint64_t first_start_us;
    bool has_previous;
    bool collecting;
    uint8_t records[LOG_QUEUE_CAPACITY][LOG_RECORD_SIZE];
    unsigned head;
    unsigned count;
    unsigned high_water;
    uint32_t next_sequence;
    uint32_t dropped;
    uint32_t air_missing;
    uint32_t gps_missing;
    uint32_t air_late;
    uint32_t gps_late;
} LogRecords;

extern const uint8_t log_header[LOG_HEADER_SIZE];

void log_encode(uint8_t out[LOG_RECORD_SIZE], const LogWindow *window, uint32_t sequence);
void log_start(LogRecords *log, uint64_t now_us);
void log_advance(LogRecords *log, uint64_t now_us);
void log_add_air(LogRecords *log, const AirSample *sample);
void log_add_gps(LogRecords *log, const GpsSample *sample);
void log_reset_air(LogRecords *log);
void log_reset_gps(LogRecords *log);
const uint8_t *log_peek(const LogRecords *log, unsigned index);
void log_consume(LogRecords *log, unsigned count);

#endif
