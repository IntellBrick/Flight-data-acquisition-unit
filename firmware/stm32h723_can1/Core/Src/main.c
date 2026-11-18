/*
 * 固件的 C 语言入口。
 * 启动文件完成供电、内核和 C 运行库初始化后进入 main()。
 * 裸机主循环依次服务 CAN、停止请求、写卡和串口诊断，中断负责及时保存原始帧。
 */
#include "acquisition.h"
#include "board.h"
#include "sd_log.h"

#include <inttypes.h>
#include <stdio.h>

/* 静态存储在进入 main() 前清零，集中保存窗口、日志队列和累计统计。 */
static LogRecords records;

/* 每秒输出采集、总线、存储三组诊断，便于串口观察缺样和故障恢复。 */
static void report_status(void)
{
    const AcquisitionStats acq = acquisition_stats();
    const BoardCanStats can = board_can_stats();
    const SdLogStats *sd = sd_log_stats();
    static const char *const storage_names[] = {"waiting", "open", "fault", "closed"};
    static const char *const operation_names[] = {"-", "mount", "open", "header", "write", "sync", "close"};
    char line[384];
    (void)snprintf(line, sizeof line,
                   "GPS %s rx=%" PRIu32 " restarts=%" PRIu32 " missing=%" PRIu32 " late=%" PRIu32
                   " | AIR %s rx=%" PRIu32 " restarts=%" PRIu32 " missing=%" PRIu32 " late=%" PRIu32 "\r\n",
                   acq.gps_online ? "online" : "offline", acq.gps_messages, acq.gps_restarts,
                   records.gps_missing, records.gps_late,
                   acq.air_online ? "online" : "offline", acq.air_messages, acq.air_restarts,
                   records.air_missing, records.air_late);
    board_uart_write(line);
    (void)snprintf(line, sizeof line,
                   "CAN alloc=%" PRIu32 " fifo_lost=%" PRIu32 " ring_drop=%" PRIu32
                   " ring_max=%" PRIu32 " crc=%" PRIu32 " transport=%" PRIu32
                   " dsdl=%" PRIu32 " tx_error=%" PRIu32 " bus_off=%" PRIu32 "\r\n",
                   acq.allocations, can.fifo_lost_events, can.queue_dropped, can.queue_high_water,
                   acq.crc_errors, acq.transport_errors, acq.dsdl_errors, acq.tx_errors, can.bus_off_events);
    board_uart_write(line);
    (void)snprintf(line, sizeof line,
                   "SD %s file=%04u op=%s err=%d short=%u failures=%" PRIu32 " uncertain=%" PRIu32
                   " seq=%" PRIu32 " queue=%u max=%u drop=%" PRIu32
                   " mount/write/sync_max_us=%" PRIu64 "/%" PRIu64 "/%" PRIu64 "\r\n",
                   storage_names[sd->state], sd->file_number, operation_names[sd->failed_op],
                   (int)sd->last_error, (unsigned)sd->short_write, sd->failures, sd->uncertain_records,
                   records.next_sequence, records.count, records.high_water, records.dropped,
                   sd->max_mount_us, sd->max_write_us, sd->max_sync_us);
    board_uart_write(line);
}

int main(void)
{
    /* 先启动 HAL、系统时钟、微秒时钟和串口，使协议栈可以取得本地时间。 */
    board_init();
    /* 注册完整报文回调、接收筛选函数和主控节点 ID，再开放 CAN 接收。 */
    acquisition_init(&records, board_time_us());
    board_can_start();
    board_uart_write("FDAU CAN1 1 Mbit/s | send s to drain and close the log\r\n");

    uint64_t report_at = 0;
    bool stopping = false;
    for (;;) {
        /* 优先回放接收队列，按各帧原始接收时间解码并推进记录窗口。 */
        acquisition_service();
        if (!stopping && board_stop_requested()) {
            /* s/S 只触发一次停录，后续循环继续将已封帧记录写完并关闭文件。 */
            stopping = true;
            acquisition_stop();
        }
        sd_log_service(&records, stopping);
        /* 诊断使用当前时间调度，串口输出为带超时的轮询发送。 */
        uint64_t now = board_time_us();
        if (now >= report_at) {
            report_at = now + 1000000U;
            report_status();
        }
    }
}
