/* LED 逻辑采用本地单调微秒时间，可在测试中独立模拟时间推进。 */
#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "acquisition.h"

#define STATUS_LED_SAMPLE_TIMEOUT_US UINT64_C(1000000)

typedef struct {
    uint64_t sensors_until_us;
    uint64_t time_until_us;
    uint64_t utc_us;
    uint64_t time_received_us;
    uint32_t time_generation;
    bool sensors_ready;
    bool gps_fixed;
    bool time_valid;
} StatusLedSnapshot;

typedef struct {
    uint64_t mode_start_us;
    uint64_t previous_local_us;
    uint64_t previous_utc_us;
    uint64_t next_boundary_us;
    uint64_t last_event_us;
    uint64_t pulse_start_us;
    uint32_t time_generation;
    unsigned mode;
    bool clock_locked;
    bool pulse_active;
    bool double_pulse;
} StatusLedController;

typedef struct {
    bool sensor_on;
    bool time_on;
} StatusLedOutput;

/* 主循环准备快照，板级代码在短临界区中将其发布给 SysTick。 */
StatusLedSnapshot status_led_prepare(const AcquisitionStatus *acquisition,
                                     bool recording, bool storage_ready);
/* 每毫秒调用，按绝对时间计算输出；同一模式持续更新时保持原有相位。 */
StatusLedOutput status_led_tick(StatusLedController *controller,
                               const StatusLedSnapshot *snapshot, uint64_t now_us);

#endif
