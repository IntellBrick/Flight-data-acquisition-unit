#include <assert.h>
#include <string.h>

#include "acquisition.h"
#include "board.h"
#include "nodes.h"
#include "uavcan.equipment.gnss.Fix2.h"

static const uint64_t service_time = 5850000;
static BoardCanFrame input[32];
static unsigned input_next;
static unsigned input_count;
static LogRecords records;

uint64_t board_time_us(void)
{
    return service_time;
}

bool board_can_idle_time(uint64_t *now_us)
{
    *now_us = service_time;
    return input_next == input_count;
}

bool board_can_receive(BoardCanFrame *frame)
{
    if (input_next == input_count) {
        return false;
    }
    *frame = input[input_next++];
    return true;
}

bool board_can_send(const CanardCANFrame *frame)
{
    (void)frame;
    return true;
}

void board_can_recover(void)
{
}

static void queue_fix(CanardInstance *sender, uint8_t *tid, uint64_t first_us, uint64_t last_us)
{
    struct uavcan_equipment_gnss_Fix2 fix = {.sats_used = 5};
    uint8_t payload[UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE];
    uint32_t size = uavcan_equipment_gnss_Fix2_encode(&fix, payload);
    int16_t count = canardBroadcast(sender, UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE,
                                    UAVCAN_EQUIPMENT_GNSS_FIX2_ID, tid,
                                    CANARD_TRANSFER_PRIORITY_LOW, payload, (uint16_t)size, service_time);
    assert(count > 1 && (unsigned)count <= sizeof input / sizeof input[0] - input_count);
    for (unsigned i = 0; i < (unsigned)count; ++i) {
        input[input_count++] = (BoardCanFrame){
            .frame = *canardPeekTxQueue(sender),
            .received_us = first_us + i * (last_us - first_us) / ((unsigned)count - 1),
        };
        canardPopTxQueue(sender);
    }
}

int main(void)
{
    CanardInstance sender;
    _Alignas(8) uint8_t pool[1024];
    canardInit(&sender, pool, sizeof pool, NULL, NULL, NULL);
    canardSetLocalNodeID(&sender, GPS_NODE_ID);

    uint8_t tid = 0;
    queue_fix(&sender, &tid, 5199700, 5203000);
    queue_fix(&sender, &tid, 5399700, 5420000);
    acquisition_init(&records, 0);
    acquisition_service();

    AcquisitionStats stats = acquisition_stats();
    assert(stats.gps_messages == 2 && stats.crc_errors == 0 && stats.dsdl_errors == 0);
    assert(records.count == 4 && records.gps_late == 1);
    const uint8_t *first = log_peek(&records, 0);
    // Offset 199700 us, received NO_FIX, five satellites, position invalid.
    const uint8_t offset[] = {0x14, 0x0C, 0x03, 0};
    assert(memcmp(first + 20, offset, sizeof offset) == 0);
    assert(first[64] == GPS_NODE_ID && first[65] == 0 && first[66] == 5 && first[68] == 1);
    assert(log_peek(&records, 1)[68] == 0);
    return 0;
}
