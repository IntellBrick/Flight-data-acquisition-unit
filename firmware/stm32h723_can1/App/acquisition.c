/*
 * DroneCAN 协议和采集调度。
 * 主循环与每次文件操作后的服务点调用 acquisition_service()，
 * 在这里重组多帧、分配节点、监测心跳，并提交传感器候选样本。
 */
#include "acquisition.h"

#include <string.h>

#include "board.h"
#include "nodes.h"
#include "uavcan.equipment.air_data.RawAirData.h"
#include "uavcan.equipment.gnss.Fix2.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.dynamic_node_id.Allocation.h"

#define ALLOCATION_TIMEOUT_US 500000U  /* UID 分配交换的后续请求时限。 */
#define HEARTBEAT_PERIOD_US 1000000U   /* 主控 NodeStatus 发布间隔。 */
#define OFFLINE_TIMEOUT_US 3000000U    /* 从节点最近一次心跳起算的离线时限。 */
#define LOG_START_TIMEOUT_US 5000000U  /* 设备未到齐时，最迟约 5 秒开始组帧。 */
#define AIR_DECIMATION 10U            /* 每 10 条解码成功的完整空速报文取第 10 条。 */

/* 编译时检查生成器与固件使用的类型签名一致，签名参与多帧传输 CRC。 */
_Static_assert(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE == UINT64_C(0x0B2A812620A11D40),
               "Allocation signature");
_Static_assert(UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE == UINT64_C(0xCA41E7000F37435F), "Fix2 signature");
_Static_assert(UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE == UINT64_C(0xC77DF38BA122F5DA),
               "RawAirData signature");

typedef struct {
    uint8_t id;
    uint8_t uid[16];          /* 实物唯一标识，更换设备时在下方配置表中修改。 */
    uint64_t last_heartbeat; /* 最近一次有效心跳的本地接收时间，单位微秒。 */
    uint64_t reset_at;       /* 屏蔽跨越重启、重新分配或离线边界的旧传输。 */
    uint32_t uptime;         /* 设备发布的启动秒数，回退代表检测到一次重启。 */
    uint32_t restarts;
    bool has_heartbeat;
    bool online;
} Node;

/* 两台设备按完整 UID 获得不同节点 ID，避开各自独立测试时都使用 125 的冲突。 */
static Node gps = {
    .id = GPS_NODE_ID,
    .uid = {0x47, 0x00, 0x07, 0x00, 0x0A, 0x51, 0x34, 0x33, 0x34, 0x33, 0x30, 0x37, 0, 0, 0, 0},
};
static Node air = {
    .id = AIR_NODE_ID,
    .uid = {0x51, 0x00, 0x35, 0x00, 0x12, 0x50, 0x47, 0x51, 0x53, 0x31, 0x31, 0x20, 0, 0, 0, 0},
};

static CanardInstance canard;
static _Alignas(8) uint8_t canard_pool[4096]; /* libcanard 的重组和发送固定内存池。 */
static LogRecords *records;
static AcquisitionStats stats;
static uint64_t can_start_us;
static uint64_t last_cleanup;
static uint64_t next_heartbeat;
static uint64_t received_us; /* 当前正在处理的 CAN 帧时间，回调期间为末帧时间。 */
static bool stopped;
static unsigned air_decimation;
/* 不同数据类型分别维护发送 transfer-ID，libcanard 在成功入队时更新。 */
static uint8_t heartbeat_tid;
static uint8_t allocation_tid;
static uint8_t allocation_uid[16];
static unsigned allocation_length; /* 已确认的 UID 前缀长度，依次为 0、6、12。 */
static uint64_t allocation_last_us;

/* 清除两个尚未封帧窗口中的该节点样本，已编码日志保留原记录。 */
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

/* 未在线的节点用最大值表示当前没有待处理的离线截止事件。 */
static uint64_t offline_deadline(const Node *node)
{
    return node->online ? node->last_heartbeat + OFFLINE_TIMEOUT_US : UINT64_MAX;
}

/* 两节点在线或等待时限已到后启动窗口，T0 由 log_start() 向上对齐。 */
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
    /* 按离线事件的实际时间依次推进，写卡后积压回放仍遵守同一时间轴。 */
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
    /* libcanard 自行按传输超时回收旧状态，这里约每秒触发一次清理。 */
    if (now_us - last_cleanup >= CANARD_RECOMMENDED_STALE_TRANSFER_CLEANUP_INTERVAL_USEC) {
        canardCleanupStaleTransfers(&canard, now_us);
        last_cleanup = now_us;
    }
}

/* 在硬件过滤之后再次核对传输类型、消息类型和来源，并向协议栈提供签名。 */
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

/* 将广播编码为 CAN 帧并放入 libcanard 发送队列，返回成功表示入队成功。 */
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
    /* 分配协议按当前本地时间判断请求是否过期，避免回放积压请求时发出旧应答。 */
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
    /* 匿名请求分三次携带 6、6、4 字节，阶段不符时等待设备重新请求。 */
    unsigned expected = allocation_length == 12 ? 4U : 6U;
    if (request.first_part_of_unique_id != (allocation_length == 0) || request.unique_id.len != expected) {
        return;
    }
    memcpy(allocation_uid + allocation_length, request.unique_id.data, expected);
    unsigned length = allocation_length + expected;
    /* 每个阶段检查前缀，只为本工程配置的两台设备推进交换。 */
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
    response.node_id = final ? node->id : 0; /* 前两次只回显前缀，完整匹配后授予 ID。 */
    uint8_t payload[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];
    uint32_t size = uavcan_protocol_dynamic_node_id_Allocation_encode(&response, payload);
    bool sent = broadcast(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                          UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                          CANARD_TRANSFER_PRIORITY_HIGH, &allocation_tid, payload, (uint16_t)size,
                          transfer->timestamp_usec + ALLOCATION_TIMEOUT_US);
    allocation_length = sent && !final ? length : 0;
    allocation_last_us = transfer->timestamp_usec;
    if (sent && final) {
        /* 节点需在分配后重新发布心跳，在线状态由心跳确认。 */
        ++stats.allocations;
        node->online = false;
        node->has_heartbeat = false;
        reset_samples(node, received_us);
    }
}

/* 每节点独立维护心跳和启动时间，uptime 回退时清除该节点旧样本。 */
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
    /* NaN 压差也参与完整报文计数，测量有效性在日志编码时单独判定。 */
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

/* 每条有效解码的 Fix2 都是候选，NO_FIX 也保留状态、卫星数和授时时标。 */
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
    /* transfer 时间为首帧时间，received_us 仍是完成该传输的末帧时间。 */
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

/* 只在 libcanard 完成重组、通过传输校验后进入，按消息类型分派到业务处理。 */
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

/* 发布主控心跳，uptime_sec 使用本地单调时间换算。 */
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
    /* 在 main() 中调用一次，使用静态清零状态和共享日志对象。 */
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
        /* 先推进再解码，末帧恰好等于封帧截止时刻的传输会被判为迟到。 */
        advance(received_us);
        if ((item.frame.id & 127U) == 0 &&
            (item.frame.data[item.frame.data_len - 1] & 0xE0U) != 0xC0U) {
            ++stats.transport_errors;
            continue;
        }
        /* 单帧只送一次，协议栈负责 transfer-ID、Toggle、多帧 CRC 和重组。 */
        int16_t result = canardHandleRxFrame(&canard, &item.frame, received_us);
        if (result == -CANARD_ERROR_RX_BAD_CRC) {
            ++stats.crc_errors;
        } else if (result < 0 && result != -CANARD_ERROR_RX_NOT_WANTED) {
            ++stats.transport_errors;
        }
    }
    uint64_t now;
    /* 软件队列和硬件 FIFO 同时为空时，才能用当前时间补推缺样窗口。 */
    if (board_can_idle_time(&now)) {
        advance(now);
    }
    heartbeat(now);
    const CanardCANFrame *frame;
    while ((frame = canardPeekTxQueue(&canard)) != NULL) {
        if (board_time_us() >= frame->deadline_usec) {
            ++stats.tx_errors;
        } else if (!board_can_send(frame)) {
            /* 硬件暂时忙时保留队头，下一次服务继续发送，超过截止时间才丢弃。 */
            break;
        }
        canardPopTxQueue(&canard);
    }
}

bool acquisition_allocation_pending(void)
{
    /* 分配应答优先级高于主控心跳，写卡服务据此让出一次文件操作机会。 */
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
    /* 停止推进记录窗口，已封帧队列交给写卡服务排空，CAN 服务仍由主循环调用。 */
    stopped = true;
    records->collecting = false;
}
