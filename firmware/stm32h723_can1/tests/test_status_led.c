/* LED 确定性测试，通过手动推进微秒时间验证灯光，无需实际 GPIO。 */
#include <assert.h>
#include <string.h>
#include "status_led.h"

static AcquisitionStatus ready(uint64_t now)
{
    AcquisitionStatus acquisition = {
        .gps = {.online = true, .has_sample = true, .heartbeat_us = now, .sample_us = now},
        .air = {.online = true, .has_sample = true, .heartbeat_us = now, .sample_us = now},
        .fix_status = 3,
        .satellites = 6,
    };
    return acquisition;
}

static void test_sensor_patterns(void)
{
    StatusLedController controller = {0};
    AcquisitionStatus acquisition = ready(10000000);
    StatusLedSnapshot snapshot = status_led_prepare(&acquisition, true, false);
    assert(status_led_tick(&controller, &snapshot, 10000000).sensor_on);
    assert(controller.mode == 0); /* 尚未成功写入数据，保持常亮。 */
    snapshot = status_led_prepare(&acquisition, true, true);
    assert(status_led_tick(&controller, &snapshot, 10000000).sensor_on);
    assert(controller.mode == 2);
    assert(status_led_tick(&controller, &snapshot, 10249999).sensor_on);
    assert(!status_led_tick(&controller, &snapshot, 10250000).sensor_on);
    acquisition = ready(10750000);
    snapshot = status_led_prepare(&acquisition, true, true);
    assert(!status_led_tick(&controller, &snapshot, 10750000).sensor_on);
    assert(controller.mode_start_us == 10000000); /* 连续发布快照保持原相位。 */
    acquisition = ready(11999999);
    snapshot = status_led_prepare(&acquisition, true, true);
    assert(!status_led_tick(&controller, &snapshot, 11999999).sensor_on);
    assert(status_led_tick(&controller, &snapshot, 12000000).sensor_on);

    acquisition.satellites = 5;
    snapshot = status_led_prepare(&acquisition, true, true);
    assert(status_led_tick(&controller, &snapshot, 12000000).sensor_on);
    assert(controller.mode == 1);
    assert(!status_led_tick(&controller, &snapshot, 12250000).sensor_on);
    assert(status_led_tick(&controller, &snapshot, 12500000).sensor_on);
    assert(!status_led_tick(&controller, &snapshot, 12750000).sensor_on);
    acquisition = ready(13000000);
    acquisition.fix_status = 2;
    snapshot = status_led_prepare(&acquisition, true, true);
    (void)status_led_tick(&controller, &snapshot, 13000000);
    assert(controller.mode == 1);

    /* 初始化中、无报文、离线、停止记录和存储故障均回到常亮。 */
    acquisition.gps.mode = 1;
    snapshot = status_led_prepare(&acquisition, true, true);
    assert(status_led_tick(&controller, &snapshot, 13000000).sensor_on && controller.mode == 0);
    acquisition = ready(13000000);
    acquisition.air.has_sample = false;
    snapshot = status_led_prepare(&acquisition, true, true);
    assert(!snapshot.sensors_ready);
    acquisition = ready(13000000);
    acquisition.gps.online = false;
    assert(!status_led_prepare(&acquisition, true, true).sensors_ready);
    acquisition = ready(13000000);
    assert(!status_led_prepare(&acquisition, false, true).sensors_ready);
    assert(!status_led_prepare(&acquisition, true, false).sensors_ready);
    snapshot = status_led_prepare(&acquisition, true, true);
    (void)status_led_tick(&controller, &snapshot, 13000000);
    assert(status_led_tick(&controller, &snapshot, 14000000).sensor_on && controller.mode == 0);
    acquisition = ready(15000000);
    acquisition.air.heartbeat_us = 12000000;
    snapshot = status_led_prepare(&acquisition, true, true);
    assert(status_led_tick(&controller, &snapshot, 15000000).sensor_on && controller.mode == 0);
}

static StatusLedOutput clock_tick(StatusLedController *controller, uint64_t now, uint64_t utc)
{
    AcquisitionStatus acquisition = ready(now);
    acquisition.has_time = true;
    acquisition.utc_us = utc;
    acquisition.time_received_us = now;
    StatusLedSnapshot snapshot = status_led_prepare(&acquisition, true, true);
    return status_led_tick(controller, &snapshot, now);
}

static void test_time_patterns(void)
{
    StatusLedController controller = {0};
    AcquisitionStatus acquisition = ready(10000000);
    StatusLedSnapshot snapshot = status_led_prepare(&acquisition, true, true);
    assert(status_led_tick(&controller, &snapshot, 10000000).time_on);
    assert(!clock_tick(&controller, 10000000, 59900000).time_on);
    assert(clock_tick(&controller, 10100000, 60000000).time_on);
    assert(clock_tick(&controller, 10599999, 60499999).time_on);
    assert(!clock_tick(&controller, 10600000, 60500000).time_on);
    /* 同一 UTC 秒反复出现时也不重复触发已报出的边界。 */
    assert(!clock_tick(&controller, 10700000, 60000000).time_on);
    assert(!clock_tick(&controller, 10800000, 60500000).time_on);

    memset(&controller, 0, sizeof controller);
    assert(!clock_tick(&controller, 20000000, 89900000).time_on);
    assert(clock_tick(&controller, 20100000, 90000000).time_on);
    assert(!clock_tick(&controller, 20600000, 90500000).time_on);
    assert(!clock_tick(&controller, 21099999, 90999999).time_on);
    assert(clock_tick(&controller, 21100000, 91000000).time_on);
    assert(clock_tick(&controller, 21599999, 91499999).time_on);
    assert(!clock_tick(&controller, 21600000, 91500000).time_on);
    assert(!clock_tick(&controller, 21700000, 120000000).time_on); /* 大幅跳时跳过历史脉冲。 */
    assert(controller.next_boundary_us == 150000000);

    acquisition = ready(50100000);
    acquisition.has_time = true;
    acquisition.time_received_us = 50100000;
    acquisition.utc_us = 120000000;
    snapshot = status_led_prepare(&acquisition, true, true);
    assert(!status_led_tick(&controller, &snapshot, 51099999).time_on);
    assert(status_led_tick(&controller, &snapshot, 51100000).time_on);
    assert(!clock_tick(&controller, 51200000, 121100000).time_on);

    acquisition = ready(51200000);
    acquisition.has_time = true;
    acquisition.utc_us = 121100000;
    acquisition.time_received_us = 51200000;
    acquisition.time_generation = 1;
    snapshot = status_led_prepare(&acquisition, true, true);
    (void)status_led_tick(&controller, &snapshot, 51200000);
    assert(controller.next_boundary_us == 150000000);
    acquisition.gps.online = false;
    snapshot = status_led_prepare(&acquisition, true, true);
    assert(status_led_tick(&controller, &snapshot, 51300000).time_on);
}

static void test_interrupt_only_progress(void)
{
    StatusLedController controller = {0};
    AcquisitionStatus acquisition = ready(10000000);
    acquisition.has_time = true;
    acquisition.utc_us = 59900000;
    acquisition.time_received_us = 10000000;
    StatusLedSnapshot snapshot = status_led_prepare(&acquisition, true, true);
    (void)status_led_tick(&controller, &snapshot, 10000000);
    /* 不发布新快照，模拟主循环被写卡阻塞，SysTick 仍精确结束脉冲。 */
    assert(status_led_tick(&controller, &snapshot, 10100000).time_on);
    assert(!status_led_tick(&controller, &snapshot, 10600000).time_on);
    assert(status_led_tick(&controller, &snapshot, 11000000).time_on);
    assert(controller.mode == 0);
}

int main(void)
{
    test_sensor_patterns();
    test_time_patterns();
    test_interrupt_only_progress();
    return 0;
}
