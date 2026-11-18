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

## 固定依赖

- STM32H7 HAL v1.11.5，软件命令超时 300 ms，内部数据等待与 ACMD41 初始化协商超时 1200 ms。[HAL v1.11.5](https://github.com/STMicroelectronics/stm32h7xx-hal-driver/tree/v1.11.5)
- STM32H7 CMSIS Device v1.10.5 与 Arm CMSIS Core 5.9.0。启动文件将栈空间设为 8 KiB。[STM32H7 CMSIS Device](https://github.com/STMicroelectronics/cmsis-device-h7/tree/v1.10.5) · [Arm CMSIS Core](https://github.com/ARM-software/CMSIS_5/tree/5.9.0)
- libcanard 提交 `601ed35467e0ac38819df17cd7c918de19f62d58`，启用发送截止时间。多帧回调使用该传输的首帧时间，与头文件的接口约定一致。DSDL 代码使用 DroneCAN 1.0.27 的定义和生成器提交 `431170fa4bfe2212b516b8f33bdc796267907f1c` 生成。[libcanard](https://github.com/DroneCAN/libcanard/tree/601ed35467e0ac38819df17cd7c918de19f62d58) · [DSDL 生成器](https://github.com/DroneCAN/dronecan_dsdlc/tree/431170fa4bfe2212b516b8f33bdc796267907f1c)
- FatFs R0.16，STM32 中间件提交 `dd9126e957ae841a13ccbe2115d39a1fb7414126`，使用单卷、512 字节扇区和 8.3 文件名。[STM32 FatFs](https://github.com/STMicroelectronics/stm32-mw-fatfs/tree/dd9126e957ae841a13ccbe2115d39a1fb7414126)
