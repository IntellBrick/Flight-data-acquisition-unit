#include "acquisition.h"

#include <string.h>

#include "board.h"
#include "nodes.h"
#include "uavcan.equipment.air_data.RawAirData.h"
#include "uavcan.equipment.gnss.Fix2.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.dynamic_node_id.Allocation.h"

#define ALLOCATION_TIMEOUT_US 500000U
#define HEARTBEAT_PERIOD_US 1000000U
#define OFFLINE_TIMEOUT_US 3000000U
#define LOG_START_TIMEOUT_US 5000000U
#define AIR_DECIMATION 10U

_Static_assert(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE == UINT64_C(0x0B2A812620A11D40),
               "Allocation signature");
_Static_assert(UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE == UINT64_C(0xCA41E7000F37435F), "Fix2 signature");
_Static_assert(UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE == UINT64_C(0xC77DF38BA122F5DA),
               "RawAirData signature");

typedef struct {
    uint8_t id;
    uint8_t uid[16];
    uint64_t last_heartbeat;
    uint64_t reset_at;
    uint32_t uptime;
    uint32_t restarts;
    bool has_heartbeat;
    bool online;
} Node;

static Node gps = {
    .id = GPS_NODE_ID,
    .uid = {0x47, 0x00, 0x07, 0x00, 0x0A, 0x51, 0x34, 0x33, 0x34, 0x33, 0x30, 0x37, 0, 0, 0, 0},
};
static Node air = {
    .id = AIR_NODE_ID,
    .uid = {0x51, 0x00, 0x35, 0x00, 0x12, 0x50, 0x47, 0x51, 0x53, 0x31, 0x31, 0x20, 0, 0, 0, 0},
};

static CanardInstance canard;
static _Alignas(8) uint8_t canard_pool[4096];
static LogRecords *records;
static AcquisitionStats stats;
static uint64_t can_start_us;
static uint64_t last_cleanup;
static uint64_t next_heartbeat;
static uint64_t received_us;
static bool stopped;
static unsigned air_decimation;
static uint8_t heartbeat_tid;
static uint8_t allocation_tid;
static uint8_t allocation_uid[16];
static unsigned allocation_length;
static uint64_t allocation_last_us;

static void reset_samples(Node *node, uint64_t time_us)
{
    node->reset_at = time_us;
    if (node == &air) {
        air_decimation = 0;
        log_reset_air(records);
    } else {
        log_reset_gps(records);
    }
}

static uint64_t offline_deadline(const Node *node)
{
    return node->online ? node->last_heartbeat + OFFLINE_TIMEOUT_US : UINT64_MAX;
}

static void start_if_ready(uint64_t now_us)
{
    if (!records->collecting && !stopped) {
        uint64_t timeout = can_start_us + LOG_START_TIMEOUT_US;
        if (now_us >= timeout) {
            log_start(records, timeout);
        } else if (gps.online && air.online) {
            log_start(records, now_us);
        }
    }
}

static void advance(uint64_t now_us)
{
    // Apply heartbeat timeouts at their receive-time deadlines, even after an SD stall.
    while (true) {
        uint64_t air_deadline = offline_deadline(&air);
        uint64_t gps_deadline = offline_deadline(&gps);
        bool air_first = air_deadline <= gps_deadline;
        uint64_t deadline = air_first ? air_deadline : gps_deadline;
        if (deadline > now_us) {
            break;
        }
        Node *node = air_first ? &air : &gps;
        start_if_ready(deadline);
        log_advance(records, deadline);
        node->online = false;
        reset_samples(node, deadline);
    }
    start_if_ready(now_us);
    log_advance(records, now_us);
    if (now_us - last_cleanup >= CANARD_RECOMMENDED_STALE_TRANSFER_CLEANUP_INTERVAL_USEC) {
        canardCleanupStaleTransfers(&canard, now_us);
        last_cleanup = now_us;
    }
}

static bool should_accept(const CanardInstance *instance, uint64_t *signature,
                          uint16_t type, CanardTransferType kind, uint8_t source)
{
    (void)instance;
    if (kind != CanardTransferTypeBroadcast) {
        return false;
    }
    if (type == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID && source == 0) {
        *signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
    } else if (type == UAVCAN_PROTOCOL_NODESTATUS_ID && (source == GPS_NODE_ID || source == AIR_NODE_ID)) {
        *signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
    } else if (type == UAVCAN_EQUIPMENT_GNSS_FIX2_ID && source == GPS_NODE_ID) {
        *signature = UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE;
    } else if (type == UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID && source == AIR_NODE_ID) {
        *signature = UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE;
    } else {
        return false;
    }
    return true;
}

static bool broadcast(uint16_t id, uint64_t signature, uint8_t priority, uint8_t *tid,
                      const uint8_t *payload, uint16_t size, uint64_t deadline)
{
    CanardTxTransfer transfer;
    canardInitTxTransfer(&transfer);
    transfer.data_type_id = id;
    transfer.data_type_signature = signature;
    transfer.priority = priority;
    transfer.inout_transfer_id = tid;
    transfer.payload = payload;
    transfer.payload_len = size;
    transfer.deadline_usec = deadline;
    if (canardBroadcastObj(&canard, &transfer) <= 0) {
        ++stats.tx_errors;
        return false;
    }
    return true;
}

static void handle_allocation(const CanardRxTransfer *transfer)
{
    uint64_t now = board_time_us();
    if (allocation_length != 0 && now - allocation_last_us >= ALLOCATION_TIMEOUT_US) {
        allocation_length = 0;
    }
    if (now - transfer->timestamp_usec >= ALLOCATION_TIMEOUT_US) {
        allocation_length = 0;
        return;
    }
    struct uavcan_protocol_dynamic_node_id_Allocation request;
    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &request)) {
        ++stats.dsdl_errors;
        return;
    }
    unsigned expected = allocation_length == 12 ? 4U : 6U;
    if (request.first_part_of_unique_id != (allocation_length == 0) || request.unique_id.len != expected) {
        return;
    }
    memcpy(allocation_uid + allocation_length, request.unique_id.data, expected);
    unsigned length = allocation_length + expected;
    Node *node = NULL;
    if (memcmp(gps.uid, allocation_uid, length) == 0) {
        node = &gps;
    } else if (memcmp(air.uid, allocation_uid, length) == 0) {
        node = &air;
    }
    if (node == NULL) {
        return;
    }

    bool final = length == sizeof allocation_uid;
    struct uavcan_protocol_dynamic_node_id_Allocation response = {0};
    response.unique_id.len = (uint8_t)length;
    memcpy(response.unique_id.data, allocation_uid, length);
    response.node_id = final ? node->id : 0;
    uint8_t payload[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
    uint32_t size = uavcan_protocol_dynamic_node_id_Allocation_encode(&response, payload);
    bool sent = broadcast(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                          UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                          CANARD_TRANSFER_PRIORITY_HIGH, &allocation_tid, payload, (uint16_t)size,
                          transfer->timestamp_usec + ALLOCATION_TIMEOUT_US);
    allocation_length = sent && !final ? length : 0;
    allocation_last_us = transfer->timestamp_usec;
    if (sent && final) {
        ++stats.allocations;
        node->online = false;
        node->has_heartbeat = false;
        reset_samples(node, received_us);
    }
}

static void handle_heartbeat(const CanardRxTransfer *transfer)
{
    struct uavcan_protocol_NodeStatus status;
    if (uavcan_protocol_NodeStatus_decode(transfer, &status)) {
        ++stats.dsdl_errors;
        return;
    }
    Node *node = transfer->source_node_id == AIR_NODE_ID ? &air : &gps;
    if (node->has_heartbeat && status.uptime_sec < node->uptime) {
        ++node->restarts;
        reset_samples(node, received_us);
    }
    node->uptime = status.uptime_sec;
    node->last_heartbeat = received_us;
    node->has_heartbeat = true;
    node->online = true;
    start_if_ready(received_us);
}

static void handle_air(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_air_data_RawAirData message;
    if (uavcan_equipment_air_data_RawAirData_decode(transfer, &message)) {
        ++stats.dsdl_errors;
        return;
    }
    ++stats.air_messages;
    if (transfer->timestamp_usec < air.reset_at) {
        return;
    }
    if (++air_decimation == AIR_DECIMATION) {
        air_decimation = 0;
        AirSample sample = {
            .time_us = transfer->timestamp_usec,
            .pressure_pa = message.differential_pressure,
            .node_id = transfer->source_node_id,
        };
        log_add_air(records, &sample);
    }
}

static void handle_gps(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_gnss_Fix2 message;
    if (uavcan_equipment_gnss_Fix2_decode(transfer, &message)) {
        ++stats.dsdl_errors;
        return;
    }
    ++stats.gps_messages;
    if (transfer->timestamp_usec < gps.reset_at) {
        return;
    }
    GpsSample sample = {
        .time_us = transfer->timestamp_usec,
        .gnss_time_us = message.gnss_timestamp.usec,
        .latitude_e8 = message.latitude_deg_1e8,
        .longitude_e8 = message.longitude_deg_1e8,
        .altitude_mm = message.height_msl_mm,
        .node_id = transfer->source_node_id,
        .status = message.status,
        .satellites = message.sats_used,
        .time_standard = message.gnss_time_standard,
    };
    memcpy(sample.velocity, message.ned_velocity, sizeof sample.velocity);
    log_add_gps(records, &sample);
}

static void on_transfer(CanardInstance *instance, CanardRxTransfer *transfer)
{
    (void)instance;
    switch (transfer->data_type_id) {
    case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
        handle_allocation(transfer);
        break;
    case UAVCAN_PROTOCOL_NODESTATUS_ID:
        handle_heartbeat(transfer);
        break;
    case UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID:
        handle_air(transfer);
        break;
    case UAVCAN_EQUIPMENT_GNSS_FIX2_ID:
        handle_gps(transfer);
        break;
    }
}

static void heartbeat(uint64_t now)
{
    if (now < next_heartbeat) {
        return;
    }
    next_heartbeat = now + HEARTBEAT_PERIOD_US;
    struct uavcan_protocol_NodeStatus status = {.uptime_sec = (uint32_t)(now / HEARTBEAT_PERIOD_US)};
    uint8_t payload[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    uint32_t size = uavcan_protocol_NodeStatus_encode(&status, payload);
    (void)broadcast(UAVCAN_PROTOCOL_NODESTATUS_ID, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    CANARD_TRANSFER_PRIORITY_LOW, &heartbeat_tid, payload, (uint16_t)size, next_heartbeat);
}

void acquisition_init(LogRecords *log, uint64_t start_us)
{
    records = log;
    can_start_us = start_us;
    last_cleanup = start_us;
    next_heartbeat = start_us;
    canardInit(&canard, canard_pool, sizeof canard_pool, on_transfer, should_accept, NULL);
    canardSetLocalNodeID(&canard, LOGGER_NODE_ID);
}

void acquisition_service(void)
{
    board_can_recover();
    BoardCanFrame item;
    while (board_can_receive(&item)) {
        received_us = item.received_us;
        advance(received_us);
        if ((item.frame.id & 127U) == 0 &&
            (item.frame.data[item.frame.data_len - 1] & 0xE0U) != 0xC0U) {
            ++stats.transport_errors;
            continue;
        }
        int16_t result = canardHandleRxFrame(&canard, &item.frame, received_us);
        if (result == -CANARD_ERROR_RX_BAD_CRC) {
            ++stats.crc_errors;
        } else if (result < 0 && result != -CANARD_ERROR_RX_NOT_WANTED) {
            ++stats.transport_errors;
        }
    }
    uint64_t now;
    if (board_can_idle_time(&now)) {
        advance(now);
    }
    heartbeat(now);
    const CanardCANFrame *frame;
    while ((frame = canardPeekTxQueue(&canard)) != NULL) {
        if (board_time_us() >= frame->deadline_usec) {
            ++stats.tx_errors;
        } else if (!board_can_send(frame)) {
            break;
        }
        canardPopTxQueue(&canard);
    }
}

bool acquisition_allocation_pending(void)
{
    // Allocation transfers outrank NodeStatus in the libcanard TX queue.
    const CanardCANFrame *frame = canardPeekTxQueue(&canard);
    return frame != NULL && ((frame->id >> 8) & 0xFFFFU) == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID;
}

AcquisitionStats acquisition_stats(void)
{
    stats.gps_online = gps.online;
    stats.air_online = air.online;
    stats.gps_restarts = gps.restarts;
    stats.air_restarts = air.restarts;
    return stats;
}

void acquisition_stop(void)
{
    stopped = true;
    records->collecting = false;
}
