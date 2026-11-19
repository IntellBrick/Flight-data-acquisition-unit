/* 用真实 DroneCAN 传输验证 LED 的实时状态输入及 UTC 换算。 */
#include <assert.h>
#include "acquisition.h"
#include "board.h"
#include "nodes.h"
#include "uavcan.equipment.air_data.RawAirData.h"
#include "uavcan.equipment.gnss.Fix2.h"
#include "uavcan.protocol.NodeStatus.h"

static CanardInstance sender;
static uint64_t now_us = 1000000;
static uint8_t tid;
static bool pending;
static LogRecords records;

uint64_t board_time_us(void) { return now_us; }
bool board_can_idle_time(uint64_t *now) { *now = now_us; return !pending; }
bool board_can_send(const CanardCANFrame *frame) { (void)frame; return true; }
void board_can_recover(void) {}

bool board_can_receive(BoardCanFrame *frame)
{
    const CanardCANFrame *next = pending ? canardPeekTxQueue(&sender) : NULL;
    if (next == NULL) {
        pending = false;
        return false;
    }
    *frame = (BoardCanFrame){.frame = *next, .received_us = now_us};
    canardPopTxQueue(&sender);
    return true;
}

static void send(uint8_t source, uint16_t type, uint64_t signature,
                 uint8_t *payload, uint32_t length)
{
    canardForgetLocalNodeID(&sender);
    canardSetLocalNodeID(&sender, source);
    assert(canardBroadcast(&sender, signature, type, &tid, 24, payload, length, now_us + 1000000) > 0);
    pending = true;
    acquisition_service();
}

static void heartbeat(uint8_t source, uint32_t uptime, uint8_t mode)
{
    struct uavcan_protocol_NodeStatus message = {.uptime_sec = uptime, .mode = mode};
    uint8_t payload[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    uint32_t length = uavcan_protocol_NodeStatus_encode(&message, payload);
    send(source, UAVCAN_PROTOCOL_NODESTATUS_ID, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE, payload, length);
}

static void fix(uint8_t standard, uint8_t leap, uint64_t timestamp)
{
    struct uavcan_equipment_gnss_Fix2 message = {
        .gnss_time_standard = standard, .num_leap_seconds = leap,
        .gnss_timestamp.usec = timestamp, .status = 3, .sats_used = 6,
    };
    uint8_t payload[UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE];
    uint32_t length = uavcan_equipment_gnss_Fix2_encode(&message, payload);
    send(GPS_NODE_ID, UAVCAN_EQUIPMENT_GNSS_FIX2_ID, UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE, payload, length);
}

int main(void)
{
    _Alignas(8) static uint8_t pool[2048];
    canardInit(&sender, pool, sizeof pool, NULL, NULL, NULL);
    acquisition_init(&records, 0);
    heartbeat(GPS_NODE_ID, 10, 1);
    heartbeat(AIR_NODE_ID, 10, 0);
    AcquisitionStatus status = acquisition_status();
    assert(status.gps.online && status.air.online && status.gps.mode == 1);
    assert(!status.gps.has_sample && !status.has_time);
    heartbeat(GPS_NODE_ID, 11, 0);
    struct uavcan_equipment_air_data_RawAirData air = {.differential_pressure = 1};
    uint8_t payload[UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE];
    uint32_t length = uavcan_equipment_air_data_RawAirData_encode(&air, payload);
    send(AIR_NODE_ID, UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID,
         UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE, payload, length);
    status = acquisition_status();
    assert(status.air.has_sample); /* 第一个完整报文即更新就绪状态，早于 10:1 抽取。 */

    fix(2, 0, 60000000);
    status = acquisition_status();
    assert(status.gps.has_sample && status.fix_status == 3 && status.satellites == 6);
    assert(status.has_time && status.utc_us == 60000000);
    assert(status.time_received_us == now_us);
    now_us += 200000;
    fix(1, 27, 97000000); /* TAI - 27 - 10 = UTC 60 秒。 */
    assert(acquisition_status().utc_us == 60000000);
    now_us += 200000;
    fix(3, 27, 78000000); /* GPS - 27 + 9 = UTC 60 秒。 */
    status = acquisition_status();
    assert(status.utc_us == 60000000 && status.time_received_us == now_us);
    uint64_t good_received = status.time_received_us;
    now_us += 200000;
    fix(3, 0, 99000000);
    assert(acquisition_status().time_received_us == good_received);
    fix(0, 27, 99000000);
    assert(acquisition_status().time_received_us == good_received);
    fix(1, 27, 1000);
    assert(acquisition_status().time_received_us == good_received);
    fix(2, 0, 0);
    assert(acquisition_status().time_received_us == good_received);

    heartbeat(GPS_NODE_ID, 1, 1);
    status = acquisition_status();
    assert(status.gps.online && status.gps.mode == 1 && !status.has_time && !status.gps.has_sample);
    assert(status.time_generation != 0);
    heartbeat(GPS_NODE_ID, 2, 0);
    fix(2, 0, 120000000);
    status = acquisition_status();
    assert(status.has_time && status.gps.has_sample && status.gps.mode == 0);
    now_us += 3000000;
    acquisition_service();
    status = acquisition_status();
    assert(!status.gps.online && !status.air.online && !status.has_time);
    assert(!status.gps.has_sample && !status.air.has_sample);
    assert(acquisition_stats().crc_errors == 0 && acquisition_stats().dsdl_errors == 0);
    return 0;
}
