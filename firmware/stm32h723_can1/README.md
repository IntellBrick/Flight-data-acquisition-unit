# STM32H723 CAN1 采集与 MicroSD 记录

本工程记录 CAN1 上一台 NEO 4SE 和一台 SKYE2 的数据。节点配置分别为 GPS 125、空速计 124、主控 100。更换传感器时，在 [acquisition.c](App/acquisition.c) 中更新对应的 16 字节 UID。

启动顺序、主循环、数据流和各源码文件职责见[中文代码说明](../../docs/can1_sd_logging_code_guide.md)。

## 构建与上板

1. 在 Keil µVision 5 中打开 [stm32h723_can1.uvprojx](MDK-ARM/stm32h723_can1.uvprojx)，选择 Arm Compiler 6。工程使用 STM32H723VGTx 和相对路径，源码依赖已随工程提供。
2. 安装 Keil STM32H7xx_DFP 4.1.3，用于 SWD 调试和片内 Flash 下载。调试器选择 **CMSIS-DAP Debugger**，端口选 **SW**，Flash 算法选 **STM32H72x-73x_1024**。输出文件位于 `MDK-ARM/Objects/`。[Keil STM32H7xx DFP](https://www.keil.arm.com/packs/stm32h7xx_dfp-keil/overview/)
3. 插入预先格式化为 FAT32 的 MicroSD 卡。按方案接入两台传感器和独立供电，联调前将两台传感器完整断电一次。CAN 分析仪设为经典 CAN 1 Mbit/s 的监听模式。
4. 打开 DAPLink 虚拟串口，设为 **115200 8N1**。每秒输出节点在线状态、累计消息数、缺样和迟到数、CAN FIFO 丢帧事件、软件队列丢帧与高水位、CRC 错误和写卡诊断。计划拔卡或重启前发送 `s`，等待串口报告 `SD closed`。

板级时钟与引脚在 [board.c](Core/Src/board.c) 中初始化。[CubeMX 配置文件](stm32h723_can1.ioc) 用于查看板级资源。用 CubeMX 调整配置时，可在单独目录生成代码，再将改动合入现有板级代码及 Keil 工程。

| 资源 | 配置 |
| --- | --- |
| 时钟 | HSE 25 MHz，SYSCLK/HCLK 200 MHz，APB1/APB2 100 MHz |
| CAN1 | PD0 RX、PD1 TX，HSE 内核时钟，1 Mbit/s，88% 采样点 |
| TIM2 | APB1 定时器时钟 200 MHz，预分频 199，单调时间单位 1 µs |
| USART1 | PA9 TX、PA10 RX，115200 8N1 |
| SDMMC1 | PC8–PC11 数据、PC12 时钟、PD2 命令，四位轮询模式，正常卡时钟 12.5 MHz |

## LED 状态指示

逻辑 LED1 为 PA15（原理图 LED2），逻辑 LED2 为 PD15（原理图 LED3），均为低电平点亮。两灯在板级初始化早期常亮。

| 灯 | 状态 | 灯光 |
| --- | --- | --- |
| LED1 | 任一传感器未就绪、数据过期、尚未成功写入数据、存储故障或已停止记录 | 常亮 |
| LED1 | 两传感器就绪且当前文件已有成功数据写入，GPS 未达到 3D_FIX 或卫星数少于 6 | 2 Hz，亮 0.25 秒、灭 0.25 秒 |
| LED1 | 两传感器就绪且当前文件已有成功数据写入，GPS 为 3D_FIX 且卫星数至少 6 | 0.5 Hz，亮 0.25 秒、灭 1.75 秒 |
| LED2 | 未授时、连续 1 秒无有效授时，或 GPS 重启/掉线 | 常亮 |
| LED2 | 有效授时后的 UTC 整分 00 秒 | 亮 0.5 秒一次 |
| LED2 | 有效授时后的 UTC 半分 30 秒 | 亮 0.5 秒、灭 0.5 秒、再亮 0.5 秒 |
| LED2 | 有效授时且当前没有报时脉冲 | 熄灭 |

传感器就绪要求心跳在线、模式为 Operational，并且最近 1 秒内有解码成功的完整传感器报文。记录窗口已启动且当前文件的完整数据写入成功后，LED1 才开始闪烁。重新挂载时重新等待新文件首批数据写入。

LED2 独立于搜星及写卡状态。UTC 时标直接使用，TAI/GPS 时标根据 Fix2 的闰秒字段换算，未知时标或换算所需闰秒未知时不刷新授时。初次授时、超时恢复、节点重启或大幅跳时后，从下一未来边界开始报时。时序由 1 ms SysTick 和本地单调时钟驱动，实际对时精度受 Fix2 报文发送与接收延迟影响。

## 日志导出

从仓库根目录运行以下命令。可省略 `-o` 以输出到标准输出。序号缺口、CRC 错误与短尾报告写到标准错误。重挂载后创建新文件，同一次启动中的各文件共享时间轴和帧序号。

```powershell
python tools/decode_sd_log.py E:/LOG0001.BIN -o LOG0001.csv
```

`uncertain` 统计发生读写、短写或同步故障时，固件无法确认已持久化的记录数。导出时以各文件里的有效 CRC 和实际序号为准。每次启动从空闲文件名开始，卡上已有日志继续保留。

## 窗口回归测试

从本工程目录用 GCC 或 Clang 运行。在积压帧回放中，测试覆盖首帧归属、跨界接收和恰好达到 20 ms 截止时刻的迟到报文。

```sh
cc -std=c11 -DCANARD_ENABLE_DEADLINE=1 -IApp -ICore/Inc \
  -IThirdParty/libcanard -IThirdParty/DSDL/include \
  tests/test_can1_windows.c App/acquisition.c App/log_record.c \
  ThirdParty/libcanard/canard.c ThirdParty/DSDL/src/*.c -o test_can1_windows
./test_can1_windows
```

LED 时序和状态集成测试可在本工程目录执行。

```sh
cc -std=c11 -IApp -ICore/Inc -IThirdParty/libcanard \
  tests/test_status_led.c App/status_led.c -o test_status_led
./test_status_led
cc -std=c11 -DCANARD_ENABLE_DEADLINE=1 -IApp -ICore/Inc \
  -IThirdParty/libcanard -IThirdParty/DSDL/include \
  tests/test_acquisition_status.c App/acquisition.c App/log_record.c \
  ThirdParty/libcanard/canard.c ThirdParty/DSDL/src/*.c -o test_acquisition_status
./test_acquisition_status
cc -std=c11 -IApp -ICore/Inc -IThirdParty/libcanard -IThirdParty/FatFs/source \
  tests/test_sd_led_status.c App/sd_log.c App/log_record.c -o test_sd_led_status
./test_sd_led_status
```

## 固定依赖

- STM32H7 HAL v1.11.5，软件命令超时 300 ms，内部数据等待与 ACMD41 初始化协商超时 1200 ms。[HAL v1.11.5](https://github.com/STMicroelectronics/stm32h7xx-hal-driver/tree/v1.11.5)
- STM32H7 CMSIS Device v1.10.5 与 Arm CMSIS Core 5.9.0。启动文件将栈空间设为 8 KiB。[STM32H7 CMSIS Device](https://github.com/STMicroelectronics/cmsis-device-h7/tree/v1.10.5) · [Arm CMSIS Core](https://github.com/ARM-software/CMSIS_5/tree/5.9.0)
- libcanard 提交 `601ed35467e0ac38819df17cd7c918de19f62d58`，启用发送截止时间。多帧回调使用该传输的首帧时间，与头文件的接口约定一致。DSDL 代码使用 DroneCAN 1.0.27 的定义和生成器提交 `431170fa4bfe2212b516b8f33bdc796267907f1c` 生成。[libcanard](https://github.com/DroneCAN/libcanard/tree/601ed35467e0ac38819df17cd7c918de19f62d58) · [DSDL 生成器](https://github.com/DroneCAN/dronecan_dsdlc/tree/431170fa4bfe2212b516b8f33bdc796267907f1c)
- FatFs R0.16，STM32 中间件提交 `dd9126e957ae841a13ccbe2115d39a1fb7414126`，使用单卷、512 字节扇区和 8.3 文件名。[STM32 FatFs](https://github.com/STMicroelectronics/stm32-mw-fatfs/tree/dd9126e957ae841a13ccbe2115d39a1fb7414126)
