#include "board.h"

#include <string.h>

#include "stm32h7xx_hal.h"

#include "nodes.h"
#include "uavcan.equipment.air_data.RawAirData.h"
#include "uavcan.equipment.gnss.Fix2.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.dynamic_node_id.Allocation.h"

#define CAN_RX_CAPACITY 1024U
#define CAN_RX_MASK (CAN_RX_CAPACITY - 1U)

static FDCAN_HandleTypeDef can;
static TIM_HandleTypeDef timer;
static UART_HandleTypeDef uart;

static BoardCanFrame rx_queue[CAN_RX_CAPACITY];
static volatile uint32_t rx_head;
static volatile uint32_t rx_tail;
static volatile BoardCanStats can_stats;
static volatile uint32_t timer_wraps;
static volatile bool bus_off;

static void require(HAL_StatusTypeDef result)
{
    if (result != HAL_OK) {
        __disable_irq();
        for (;;) {
            __NOP();
        }
    }
}

static void clock_init(void)
{
    require(HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY));
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    RCC_OscInitTypeDef osc = {
        .OscillatorType = RCC_OSCILLATORTYPE_HSE,
        .HSEState = RCC_HSE_ON,
        .PLL = {
            .PLLState = RCC_PLL_ON,
            .PLLSource = RCC_PLLSOURCE_HSE,
            .PLLM = 5,
            .PLLN = 80,
            .PLLP = 2,
            .PLLQ = 4,
            .PLLR = 2,
            .PLLRGE = RCC_PLL1VCIRANGE_2,
            .PLLVCOSEL = RCC_PLL1VCOWIDE,
        },
    };
    require(HAL_RCC_OscConfig(&osc));
    RCC_ClkInitTypeDef clocks = {
        .ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1,
        .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
        .SYSCLKDivider = RCC_SYSCLK_DIV1,
        .AHBCLKDivider = RCC_HCLK_DIV1,
        .APB1CLKDivider = RCC_APB1_DIV2,
        .APB2CLKDivider = RCC_APB2_DIV2,
        .APB3CLKDivider = RCC_APB3_DIV2,
        .APB4CLKDivider = RCC_APB4_DIV2,
    };
    require(HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_4));
    RCC_PeriphCLKInitTypeDef peripheral = {
        .PeriphClockSelection = RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_SDMMC | RCC_PERIPHCLK_USART1,
        .FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE,
        .SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL,
        .Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2,
    };
    require(HAL_RCCEx_PeriphCLKConfig(&peripheral));
}

static void timer_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    timer.Instance = TIM2;
    timer.Init.Prescaler = 199; // APB1 = 100 MHz, TIM2 = 2 x APB1.
    timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer.Init.Period = UINT32_MAX;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    require(HAL_TIM_Base_Init(&timer));
    __HAL_TIM_CLEAR_FLAG(&timer, TIM_FLAG_UPDATE);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    require(HAL_TIM_Base_Start_IT(&timer));
}

static void uart_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    GPIO_InitTypeDef pins = {
        .Pin = GPIO_PIN_9 | GPIO_PIN_10,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = GPIO_AF7_USART1,
    };
    HAL_GPIO_Init(GPIOA, &pins);
    uart.Instance = USART1;
    uart.Init.BaudRate = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    require(HAL_UART_Init(&uart));
}

void board_init(void)
{
    require(HAL_Init());
    clock_init();
    timer_init();
    uart_init();
}

void HAL_SD_MspInit(SD_HandleTypeDef *sd)
{
    if (sd->Instance != SDMMC1) {
        return;
    }
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    GPIO_InitTypeDef pins = {
        .Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF12_SDIO1,
    };
    HAL_GPIO_Init(GPIOC, &pins);
    pins.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &pins);
    pins.Pin = GPIO_PIN_12;
    pins.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &pins);
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *sd)
{
    if (sd->Instance == SDMMC1) {
        __HAL_RCC_SDMMC1_FORCE_RESET();
        __HAL_RCC_SDMMC1_RELEASE_RESET();
        __HAL_RCC_SDMMC1_CLK_DISABLE();
    }
}

static void can_filter(unsigned index, uint32_t id, uint32_t mask)
{
    FDCAN_FilterTypeDef filter = {
        .IdType = FDCAN_EXTENDED_ID,
        .FilterIndex = index,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = id,
        .FilterID2 = mask,
    };
    require(HAL_FDCAN_ConfigFilter(&can, &filter));
}

void board_can_start(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_FDCAN_CLK_ENABLE();
    GPIO_InitTypeDef pins = {
        .Pin = GPIO_PIN_0 | GPIO_PIN_1,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF9_FDCAN1,
    };
    HAL_GPIO_Init(GPIOD, &pins);
    can.Instance = FDCAN1;
    can.Init = (FDCAN_InitTypeDef){
        .FrameFormat = FDCAN_FRAME_CLASSIC,
        .Mode = FDCAN_MODE_NORMAL,
        // A missing peer must not hold the hardware FIFO on an unacknowledged frame.
        .AutoRetransmission = DISABLE,
        .NominalPrescaler = 1,
        .NominalSyncJumpWidth = 2,
        .NominalTimeSeg1 = 21,
        .NominalTimeSeg2 = 3,
        .DataPrescaler = 1,
        .DataSyncJumpWidth = 1,
        .DataTimeSeg1 = 1,
        .DataTimeSeg2 = 1,
        .ExtFiltersNbr = 5,
        .RxFifo0ElmtsNbr = 64,
        .RxFifo0ElmtSize = FDCAN_DATA_BYTES_8,
        .TxFifoQueueElmtsNbr = 8,
        .TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION,
        .TxElmtSize = FDCAN_DATA_BYTES_8,
    };
    require(HAL_FDCAN_Init(&can));
    // The anonymous frame's upper ID bits contain the discriminator.
    can_filter(0, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID << 8, 0x3FFU);
    can_filter(1, (UAVCAN_PROTOCOL_NODESTATUS_ID << 8) | GPS_NODE_ID, 0xFFFFFFU);
    can_filter(2, (UAVCAN_PROTOCOL_NODESTATUS_ID << 8) | AIR_NODE_ID, 0xFFFFFFU);
    can_filter(3, (UAVCAN_EQUIPMENT_GNSS_FIX2_ID << 8) | GPS_NODE_ID, 0xFFFFFFU);
    can_filter(4, (UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID << 8) | AIR_NODE_ID, 0xFFFFFFU);
    require(HAL_FDCAN_ConfigGlobalFilter(&can, FDCAN_REJECT, FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE));
    require(HAL_FDCAN_ActivateNotification(&can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                          FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_BUS_OFF, 0));
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    require(HAL_FDCAN_Start(&can));
}

// Caller masks interrupts so that the counter and the software high word agree.
static uint64_t read_timer(void)
{
    uint32_t wraps = timer_wraps;
    uint32_t count = TIM2->CNT;
    if ((TIM2->SR & TIM_SR_UIF) != 0) {
        ++wraps;
        count = TIM2->CNT;
    }
    return ((uint64_t)wraps << 32) | count;
}

uint64_t board_time_us(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint64_t time_us = read_timer();
    __set_PRIMASK(mask);
    return time_us;
}

bool board_can_idle_time(uint64_t *now_us)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *now_us = read_timer();
    bool idle = rx_head == rx_tail && HAL_FDCAN_GetRxFifoFillLevel(&can, FDCAN_RX_FIFO0) == 0;
    __set_PRIMASK(mask);
    return idle;
}

bool board_can_receive(BoardCanFrame *frame)
{
    uint32_t tail = rx_tail;
    if (tail == rx_head) {
        return false;
    }
    __DMB();
    *frame = rx_queue[tail & CAN_RX_MASK];
    __DMB();
    rx_tail = tail + 1;
    return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *fdcan, uint32_t interrupts)
{
    if ((interrupts & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0) {
        ++can_stats.fifo_lost_events;
    }
    while (HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO0) != 0) {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[64];
        uint64_t received_us = board_time_us();
        if (HAL_FDCAN_GetRxMessage(fdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
            break;
        }
        if (header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength == 0 ||
            header.DataLength > CANARD_CAN_FRAME_MAX_DATA_LEN) {
            continue;
        }
        uint32_t head = rx_head;
        uint32_t used = head - rx_tail;
        if (used == CAN_RX_CAPACITY) {
            ++can_stats.queue_dropped;
            continue;
        }
        BoardCanFrame *item = &rx_queue[head & CAN_RX_MASK];
        item->received_us = received_us;
        item->frame.id = header.Identifier | CANARD_CAN_FRAME_EFF;
        item->frame.data_len = (uint8_t)header.DataLength;
        item->frame.iface_id = 0;
        memcpy(item->frame.data, data, header.DataLength);
        __DMB();
        rx_head = head + 1;
        if (used + 1 > can_stats.queue_high_water) {
            can_stats.queue_high_water = used + 1;
        }
    }
}

bool board_can_send(const CanardCANFrame *frame)
{
    if (bus_off || (FDCAN1->CCCR & FDCAN_CCCR_INIT) != 0 ||
        HAL_FDCAN_GetTxFifoFreeLevel(&can) == 0) {
        return false;
    }
    FDCAN_TxHeaderTypeDef header = {
        .Identifier = frame->id & CANARD_CAN_EXT_ID_MASK,
        .IdType = FDCAN_EXTENDED_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = frame->data_len,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    };
    return HAL_FDCAN_AddMessageToTxFifoQ(&can, &header, frame->data) == HAL_OK;
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *fdcan, uint32_t interrupts)
{
    if ((interrupts & FDCAN_IT_BUS_OFF) != 0 && (fdcan->Instance->PSR & FDCAN_PSR_BO) != 0) {
        ++can_stats.bus_off_events;
        bus_off = true;
    }
}

void board_can_recover(void)
{
    if (bus_off) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        // M_CAN enters INIT on bus-off. Clear it to start the ISO CAN recovery sequence.
        FDCAN1->TXBCR = FDCAN1->TXBRP;
        CLEAR_BIT(FDCAN1->CCCR, FDCAN_CCCR_INIT);
        bus_off = false;
        __set_PRIMASK(mask);
    }
}

BoardCanStats board_can_stats(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    BoardCanStats snapshot = can_stats;
    __set_PRIMASK(mask);
    return snapshot;
}

bool board_stop_requested(void)
{
    bool stop = false;
    while (__HAL_UART_GET_FLAG(&uart, UART_FLAG_RXNE)) {
        uint8_t ch = (uint8_t)USART1->RDR;
        stop |= ch == 's' || ch == 'S';
    }
    __HAL_UART_CLEAR_OREFLAG(&uart);
    return stop;
}

void board_uart_write(const char *text)
{
    (void)HAL_UART_Transmit(&uart, (const uint8_t *)text, (uint16_t)strlen(text), 100);
}

void TIM2_IRQHandler(void)
{
    __HAL_TIM_CLEAR_FLAG(&timer, TIM_FLAG_UPDATE);
    ++timer_wraps;
}

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&can);
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}
