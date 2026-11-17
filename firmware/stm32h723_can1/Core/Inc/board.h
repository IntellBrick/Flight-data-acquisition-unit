#ifndef BOARD_H
#define BOARD_H

#include "canard.h"

typedef struct {
    CanardCANFrame frame;
    uint64_t received_us;
} BoardCanFrame;

typedef struct {
    uint32_t fifo_lost_events;
    uint32_t queue_dropped;
    uint32_t queue_high_water;
    uint32_t bus_off_events;
} BoardCanStats;

void board_init(void);
void board_can_start(void);
uint64_t board_time_us(void);
bool board_can_receive(BoardCanFrame *frame);
bool board_can_idle_time(uint64_t *now_us);
bool board_can_send(const CanardCANFrame *frame);
void board_can_recover(void);
BoardCanStats board_can_stats(void);
bool board_stop_requested(void);
void board_uart_write(const char *text);

#endif
