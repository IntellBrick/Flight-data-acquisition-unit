/* 板级公共接口，使采集模块只依赖 CAN 帧、时间和发送能力。 */
#ifndef BOARD_H
#define BOARD_H

#include "canard.h"

typedef struct {
    CanardCANFrame frame;
    uint64_t received_us; /* CAN 中断取帧时的本地单调时间，单位微秒。 */
} BoardCanFrame;

typedef struct {
    uint32_t fifo_lost_events; /* 硬件 FIFO 丢帧事件次数。 */
    uint32_t queue_dropped;   /* 软件队列满时丢弃的新帧数。 */
    uint32_t queue_high_water; /* 软件队列历史最高积压帧数。 */
    uint32_t bus_off_events;  /* 控制器进入 bus-off 的次数。 */
} BoardCanStats;

/* 由 main() 顺序调用板级初始化和 CAN 启动，中间先完成采集模块初始化。 */
void board_init(void);
void board_can_start(void);
uint64_t board_time_us(void); /* 从 TIM2 读取启动后的 64 位微秒时间。 */
bool board_can_receive(BoardCanFrame *frame);
bool board_can_idle_time(uint64_t *now_us); /* 始终填写时间，返回两个接收缓冲是否均为空。 */
bool board_can_send(const CanardCANFrame *frame);
void board_can_recover(void);
BoardCanStats board_can_stats(void);
bool board_stop_requested(void); /* 消费串口接收字符并检查 s/S。 */
void board_uart_write(const char *text);

#endif
