#ifndef ACQUISITION_H
#define ACQUISITION_H

#include "log_record.h"

typedef struct {
    uint32_t allocations;
    uint32_t gps_messages;
    uint32_t air_messages;
    uint32_t crc_errors;
    uint32_t transport_errors;
    uint32_t dsdl_errors;
    uint32_t tx_errors;
    uint32_t gps_restarts;
    uint32_t air_restarts;
    bool gps_online;
    bool air_online;
} AcquisitionStats;

void acquisition_init(LogRecords *log, uint64_t start_us);
void acquisition_service(void);
bool acquisition_allocation_pending(void);
AcquisitionStats acquisition_stats(void);
void acquisition_stop(void);

#endif
