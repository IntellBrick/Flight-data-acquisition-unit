#include "status_led.h"

#define HEARTBEAT_TIMEOUT_US UINT64_C(3000000)
#define HALF_MINUTE_US UINT64_C(30000000)
#define MINUTE_US UINT64_C(60000000)
#define PULSE_US UINT64_C(500000)
#define CLOCK_JUMP_US UINT64_C(1000000)

static uint64_t earlier(uint64_t a, uint64_t b)
{
    return a < b ? a : b;
}

static bool sensor_ready(const AcquisitionNodeStatus *node)
{
    /* DroneCAN Operational 的模式值为 0；完整报文可包含无效测量值。 */
    return node->online && node->mode == 0 && node->has_sample;
}

StatusLedSnapshot status_led_prepare(const AcquisitionStatus *acquisition,
                                     bool recording, bool storage_ready)
{
    const AcquisitionNodeStatus *gps = &acquisition->gps;
    const AcquisitionNodeStatus *air = &acquisition->air;
    uint64_t gps_until = earlier(gps->heartbeat_us + HEARTBEAT_TIMEOUT_US,
                                 gps->sample_us + STATUS_LED_SAMPLE_TIMEOUT_US);
    uint64_t air_until = earlier(air->heartbeat_us + HEARTBEAT_TIMEOUT_US,
                                 air->sample_us + STATUS_LED_SAMPLE_TIMEOUT_US);
    StatusLedSnapshot snapshot = {
        .sensors_until_us = earlier(gps_until, air_until),
        .sensors_ready = recording && storage_ready && sensor_ready(gps) && sensor_ready(air),
        .gps_fixed = acquisition->fix_status == 3 && acquisition->satellites >= 6,
        .time_valid = gps->online && acquisition->has_time,
        .time_until_us = earlier(gps->heartbeat_us + HEARTBEAT_TIMEOUT_US,
                                 acquisition->time_received_us + STATUS_LED_SAMPLE_TIMEOUT_US),
        .utc_us = acquisition->utc_us,
        .time_received_us = acquisition->time_received_us,
        .time_generation = acquisition->time_generation,
    };
    return snapshot;
}

static bool time_output(StatusLedController *controller,
                        const StatusLedSnapshot *snapshot, uint64_t now_us)
{
    if (controller->time_generation != snapshot->time_generation) {
        controller->time_generation = snapshot->time_generation;
        controller->clock_locked = false;
        controller->pulse_active = false;
    }
    if (!snapshot->time_valid || now_us >= snapshot->time_until_us ||
        now_us < snapshot->time_received_us) {
        controller->clock_locked = false;
        controller->pulse_active = false;
        return true;
    }
    uint64_t utc = snapshot->utc_us + now_us - snapshot->time_received_us;
    uint64_t expected = controller->previous_utc_us + now_us - controller->previous_local_us;
    uint64_t correction = utc > expected ? utc - expected : expected - utc;
    if (!controller->clock_locked || correction >= CLOCK_JUMP_US) {
        /* 首次授时、恢复或大幅跳时从下一未来边界开始，跳过历史事件。 */
        controller->next_boundary_us = (utc / HALF_MINUTE_US + 1) * HALF_MINUTE_US;
        controller->pulse_active = false;
        controller->clock_locked = true;
    } else if (utc >= controller->next_boundary_us) {
        uint64_t boundary = controller->next_boundary_us;
        if (boundary > controller->last_event_us) {
            controller->last_event_us = boundary;
            controller->double_pulse = boundary % MINUTE_US != 0;
            /* 亚秒级校时跨过边界时，仍将脉冲对齐到该边界的估计本地时刻。 */
            uint64_t delay = utc - boundary;
            controller->pulse_start_us = now_us >= delay ? now_us - delay : 0;
            controller->pulse_active = true;
        }
        controller->next_boundary_us = (utc / HALF_MINUTE_US + 1) * HALF_MINUTE_US;
    }
    controller->previous_local_us = now_us;
    controller->previous_utc_us = utc;
    if (!controller->pulse_active) {
        return false;
    }
    uint64_t elapsed = now_us - controller->pulse_start_us;
    uint64_t duration = controller->double_pulse ? 3 * PULSE_US : PULSE_US;
    if (elapsed >= duration) {
        controller->pulse_active = false;
        return false;
    }
    return elapsed < PULSE_US || elapsed >= 2 * PULSE_US;
}

StatusLedOutput status_led_tick(StatusLedController *controller,
                               const StatusLedSnapshot *snapshot, uint64_t now_us)
{
    unsigned mode = 0; /* 0 常亮、1 快闪、2 慢闪。 */
    if (snapshot->sensors_ready && now_us < snapshot->sensors_until_us) {
        mode = snapshot->gps_fixed ? 2U : 1U;
    }
    if (mode != controller->mode) {
        controller->mode = mode;
        controller->mode_start_us = now_us;
    }
    uint64_t period = mode == 2 ? UINT64_C(2000000) : UINT64_C(500000);
    StatusLedOutput output = {
        .sensor_on = mode == 0 || (now_us - controller->mode_start_us) % period < UINT64_C(250000),
        .time_on = time_output(controller, snapshot, now_us),
    };
    return output;
}
