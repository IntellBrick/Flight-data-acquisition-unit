# CAN1 双传感器采集与 MicroSD 记录实施方案

| 项目 | 内容 |
|---|---|
| 项目 | Flight-data acquisition unit |
| 日期 | 2026-11-17（北京时间） |
| 文档版本 | v1.0 |
| 二进制日志格式版本 | 1 |
| 状态 | 台架验证阶段的实施设计，待编码和上板验证 |

## 1. 目标与工程路径

用 STM32H723VGT6 开发板的 CAN1 同时接入一台 CUAV NEO 4SE GPS 和一台 CUAV SKYE2 空速计。采用 STM32CubeH7 HAL、裸机主循环和 FDCAN 接收中断。每 200 ms 生成一帧日志，将两路样本写入板载 MicroSD。串口用于观察节点状态、采集计数和写卡故障，PC 端负责导出 CSV。本版范围为台架采集、存储、回读和故障验证。

| 路径 | 用途 |
|---|---|
| `firmware/stm32h723_can1/` | 固件根目录，放置 CubeMX `.ioc`、构建说明及板级配置 |
| `firmware/stm32h723_can1/MDK-ARM/stm32h723_can1.uvprojx` | Keil µVision 工程入口 |
| `firmware/stm32h723_can1/Core/` | CubeMX 生成的启动、外设初始化和中断入口 |
| `firmware/stm32h723_can1/App/` | CAN 帧队列、节点分配、传感器解码、时间窗口、日志编码与写卡 |
| `firmware/stm32h723_can1/ThirdParty/` | 固定版本的 DroneCAN libcanard 和所需 DSDL 生成代码 |
| `tools/decode_sd_log.py` | 校验日志并导出 CSV 的计划入口 |
| `reports/` | 已有抓包及后续台架测试原始结果 |

Keil 工程使用相对路径和 Arm Compiler 6。目标设备选择 STM32H723VG，使用 STM32H7xx_DFP 对应的片内 Flash 算法。通过板载 DAPLink 的 CMSIS-DAP 接口，以 SWD 调试和下载。USART1 的 `PA9/PA10` 已接到板载 DAPLink 串口通路，台架上确认虚拟串口枚举和收发。

## 2. 板级配置与接线

| 接口 | 原理图核对结果 |
|---|---|
| FDCAN1 | `PD0` 为 RX，`PD1` 为 TX。U5 为 TJA1042T/1118，STB 接地，R3 为板载 120 Ω 终端 |
| CAN1 连接器 U25 | 1 脚 `CAN_L`，2 脚 `CAN_H`，9 脚 `GND` |
| MicroSD 卡槽 J2 | SDMMC1 四位模式，`D0–D3` 对应 `PC8–PC11`，`CK` 对应 `PC12`，`CMD` 对应 `PD2` |
| 卡槽检测 | 原理图未见独立插卡检测信号接入 MCU。通过挂载及块设备返回值判定介质状态 |

开发板放在 CAN 主干的一端，R3 作为该端终端。核对传感器和适配器已有的终端配置，在主干两端各保留一处 120 Ω 终端。断电后测量 `CAN_H` 到 `CAN_L`，两端终端均有效时约为 60 Ω。三台设备按信号名接线并共地。NEO 4SE 按其 4.7–5.2 V 规格供电，SKYE2 通过适配其 35 W 峰值加热的独立供电链路接入，按厂家要求核对 SKYE2 HV HUB 和线束。

FDCAN1 使用正常模式、经典 CAN、1 Mbit/s 和 29 位扩展数据帧。建议从板载 25 MHz HSE 取得 FDCAN 内核时钟，起始参数为预分频 1、`TSEG1=21`、`TSEG2=3`、`SJW=2`，采样点为 88%。用 CubeMX 核对实际时钟树和 FDCAN 的 APB 时钟约束，再用 CANable 抓包确认位速率及错误计数。SDMMC1 先按 HAL 块读写和 FAT32 上板，写卡路径保持 FDCAN 中断可服务。

## 3. DroneCAN 采集与节点管理

### 3.1 动态节点分配

| 设备 | 节点 ID | 已测设备 UID（16 字节） |
|---|---:|---|
| STM32 主控 | 100 | 固定主控 ID |
| NEO 4SE | 125 | `47 00 07 00 0A 51 34 33 34 33 30 37 00 00 00 00` |
| SKYE2 | 124 | `51 00 35 00 12 50 47 51 53 31 31 20 00 00 00 00` |

两台设备在独立测试中均使用过动态 ID 125。固件按完整 UID 查表，在同一总线上授予上表中的不同 ID。更换实物时更新 UID 配置。双设备测试前先对两台设备完整断电，清除独立测试留下的动态 ID。

- 单分配器按 `6 + 6 + 4` 字节处理匿名 `Allocation` 请求。前两步回显累计的 6 或 12 字节，第三步匹配完整 UID 后回包，内容为所授节点 ID 与完整 16 字节 UID。
- 同一时刻推进一组交换。阶段不匹配的请求跳过，由设备重新发起。500 ms 未收到有效后续请求即清理状态。若主循环从写卡返回时待处理的分配请求已经超过交换时限，清空分配状态并等待新请求。
- 分配器一直运行。识别同一 UID 再次申请或心跳运行时间回退时，清理对应节点的应用层采样状态，并按 libcanard 的超时机制清理过期传输。两台设备同时冷启动时验证最终均能上线。
- 主控约每秒发布一次 `NodeStatus`，并独立监测两个节点。以心跳接收时间计算，连续约 3 秒无心跳时标记离线。台架适配器用于监听，STM32 是总线上唯一的动态分配器。

### 3.2 报文与抽取

| 报文 | 数据类型 ID | 已测频率 | 用途 |
|---|---:|---:|---|
| `dynamic_node_id.Allocation` | 1 | 启动和重连时出现 | 动态节点 ID 分配 |
| `NodeStatus` | 341 | 每节点约 1 Hz | 在线和重启诊断 |
| `gnss.Fix2` | 1063 | 约 5 Hz，样本通常用 10 个 CAN 帧传输 | 每条通过校验且解码成功的报文都是记录候选，保留 `NO_FIX` |
| `air_data.RawAirData` | 1027 | 约 50 Hz，样本通常用 3 个 CAN 帧传输 | 每收到 10 条通过校验且解码成功的完整报文，保留第 10 条 |

使用 libcanard 处理多帧重组、传输 ID、Toggle 和传输 CRC，使用对应 DSDL 生成的解码器。分别核对 `Allocation`、`Fix2`、`RawAirData` 的类型签名 `0B2A812620A11D40`、`CA41E7000F37435F`、`C77DF38BA122F5DA`。接收过滤保留匿名分配请求和上表需要的报文，统计重组或 CRC 错误。空速计抽取计数在节点重启或确认离线时归零。

抽取的单位始终是完整 `RawAirData` 报文。含 `NaN` 压差但通过传输校验和 DSDL 解码的报文仍参与计数，被抽中时记录为已收到、压差无效。

## 4. 裸机调度、时间戳与窗口

### 4.1 调度与队列

1. FDCAN RX 中断及时取走 FIFO 中的帧，保存扩展 ID、有效载荷长度、数据和本地接收时间，放入单生产者、单消费者环形队列。中断中只做收帧、入队和错误计数。
2. 主循环优先清空 CAN 队列，按接收顺序把帧和原始时间戳交给 libcanard，再处理发送队列、心跳和记录窗口。分配应答优先于启动下一次写卡操作。
3. 每次 FatFs 调用返回后先服务积压的 CAN。`f_write()` 与 `f_sync()` 之间也服务一次 CAN。用可超时的 HAL 块操作，实测挂载、写入、同步和重挂载各自造成的最长解码停顿。

原始 CAN 队列以可容纳 **1024 帧**为起点。两台设备独立抓包合计约 320 帧/秒，按该平均值可保留约 3.2 秒，留量由双机峰值及 FDCAN FIFO、软件队列的实测高水位决定。分别统计硬件 FIFO 丢帧和软件环溢出，软件环满时丢新帧并计数。5 Hz 抽取只降低写卡量，CAN 队列容量按输入流量估算。

裸机写卡期间，中断仍可收帧，主循环的解码和节点分配会延后。卡长时间忙或故障时，以有限的驱动超时结束该操作，然后恢复处理 CAN。短期积压由队列吸收，分配协议超时由设备重试恢复。

### 4.2 时间和窗口归属

- 建议用内部时钟驱动 TIM2，配置为 1 MHz，并把硬件计数扩展为 64 位启动后单调时间。检查定时器时钟来源和硬件计数回绕，测试回绕边界上的读数单调性。
- 多帧报文的样本时间取首个 CAN 帧的接收时间。软件时间戳在 FDCAN 中断取帧时采集，代表主控观察到的接收时间。确认所固定版本 libcanard 在多帧回调中返回首帧时间。
- 台架记录在两台设备上线后开始。若 CAN1 启动约 5 秒后仍有设备缺席，开始写出含缺样标志的记录。记录起点 `T0` 取进入记录阶段后的第一个 200 ms 边界，之后各窗口为 `[T0 + n × 200000, T0 + (n + 1) × 200000)` µs。
- 候选样本按首帧时间归入窗口。同一传感器在一个窗口有多个候选时，保留接收时间最新的一条。空速计 10:1 抽取和设备各自的时钟独立运行，因此个别窗口可能没有候选。对应数据块填 0，缺样标志如实记录。
- 窗口结束后给跨边界多帧报文预留 **20 ms 初始重组余量**。同时维持当前窗口和前一待封窗口。先处理截止时刻前接收到的 CAN 帧，再封帧。积压回放期间按处理到的帧接收时间推进窗口与传输超时，队列与硬件 FIFO 排空时才用当前单调时间补推。末帧接收时间早于截止时刻的完整报文归入首帧所在窗口，达到或超过截止时刻则计为迟到并丢弃该日志候选。
- 每个窗口对应一帧，包括两路均缺样的窗口。应用在入日志队列前分配帧序号，因此日志队列丢帧可从序号缺口定位。窗口阶段和序号跨同一次 MCU 启动中的重新挂载延续。

## 5. 日志格式 v1

多字节字段固定小端，整数使用表中声明的宽度，有符号数采用二补码，`float32` 采用 IEEE 754 binary32。逐字段序列化以固定偏移写入。文件头之后连续写入固定 80 字节数据帧。所有保留字节和未定义标志位写 0。

### 5.1 文件头（16 字节）

| 字节偏移 | 字段 | 类型 | v1 值 |
|---|---|---|---|
| 0–7 | `magic` | `uint8[8]` | ASCII `FDAULOG` 后接一个 `00` 字节 |
| 8–9 | `format_version` | `uint16` | `1` |
| 10–11 | `record_size` | `uint16` | `80` |
| 12–15 | `period_us` | `uint32` | `200000` |

### 5.2 数据帧（80 字节）

排列顺序为 **时间戳 → 空速计数据 → GPS 数据 → 帧序号和 CRC**。偏移从单帧起点计算。

| 字节偏移 | 字段 | 类型 | 含义 |
|---|---|---|---|
| 0–7 | `frame_time_us` | `uint64` | 窗口起点的本地时间，µs |
| 8–11 | `air_offset_us` | `uint32` | 空速计首帧接收时间减窗口起点，µs |
| 12–15 | `diff_pressure_pa` | `float32` | SKYE2 原始压差，Pa，保留有效的负值 |
| 16 | `air_node_id` | `uint8` | 空速计源节点 ID |
| 17 | `air_flags` | `uint8` | 空速计有效性标志 |
| 18–19 | 保留 | `uint8[2]` | 填 0 |
| 20–23 | `gps_offset_us` | `uint32` | GPS 首帧接收时间减窗口起点，µs |
| 24–31 | `latitude_e8` | `int64` | 纬度，整数值乘以 `10^-8` 得到度 |
| 32–39 | `longitude_e8` | `int64` | 经度，整数值乘以 `10^-8` 得到度 |
| 40–43 | `alt_msl_mm` | `int32` | 平均海平面高度，mm |
| 44–47 | `vel_n` | `float32` | 北向速度，m/s |
| 48–51 | `vel_e` | `float32` | 东向速度，m/s |
| 52–55 | `vel_d` | `float32` | 地向速度，向下为正，m/s |
| 56–63 | `gnss_time_us` | `uint64` | `Fix2.gnss_timestamp`，µs，时标见 `gnss_time_standard` |
| 64 | `gps_node_id` | `uint8` | GPS 源节点 ID |
| 65 | `fix_status` | `uint8` | 0 `NO_FIX`，1 `TIME_ONLY`，2 `2D_FIX`，3 `3D_FIX` |
| 66 | `sats_used` | `uint8` | 用于求解的卫星数 |
| 67 | `gnss_time_standard` | `uint8` | 0 `NONE`，1 `TAI`，2 `UTC`，3 `GPS` |
| 68 | `gps_flags` | `uint8` | GPS 有效性标志 |
| 69–71 | 保留 | `uint8[3]` | 填 0 |
| 72–75 | `sequence` | `uint32` | 每次 MCU 启动后首个记录窗口为 0，每个窗口加 1 |
| 76–79 | `crc32` | `uint32` | `CRC-32/ISO-HDLC`，覆盖本帧第 0–75 字节，校验值按小端保存 |

### 5.3 有效性规则

| 标志 | 位 | 置位条件 |
|---|---:|---|
| `air_flags` | 0 | 本窗口有选中的空速计样本 |
| `air_flags` | 1 | 已收到样本且压差为有限值 |
| `gps_flags` | 0 | 本窗口有选中的 GPS 样本 |
| `gps_flags` | 1 | 已收到 `2D_FIX` 或 `3D_FIX`，且经纬度在合法范围内 |
| `gps_flags` | 2 | 已收到 GPS 样本，GNSS 时间非零且时标属于 `TAI`、`UTC` 或 `GPS` |
| `gps_flags` | 3 | 已收到 `3D_FIX`，且三个 NED 速度分量均为有限值 |

缺样的数据块全部填 0。收到样本但测量无效时，仍保存节点 ID、时间偏移和设备状态，相关数值字段填 0 并清除对应有效位。GPS 高度仅在位置有效且 `fix_status=3` 时有效，二维定位时高度填 0。`NO_FIX` 与 `TIME_ONLY` 仍保留卫星数和时标，GNSS 时间按位 2 独立判定。

有样本的偏移范围为 `0–199999 µs`，0 是合法偏移值。样本本地时间等于 `frame_time_us + offset_us`。本地时间用于两路数据对齐，GNSS 时间及原时标供后处理使用。本版记录原始压差，CSV 可另加估算 IAS 列，并注明换算假设。

### 5.4 编解码对照样例

文件头十六进制如下。

```text
46 44 41 55 4C 4F 47 00 01 00 50 00 40 0D 03 00
```

样例帧序号为 0，窗口起点为 `1,000,000 µs`。空速计偏移 `42,000 µs`，压差 `-9.25 Pa`。GPS 偏移 `128,000 µs`，纬度 `31.23456789°`，经度 `-121.23456789°`，高度 `123.250 m`，NED 速度依次为 `1.25`、`-2.5`、`0.5 m/s`，状态为 `3D_FIX`，卫星数为 12，GNSS 时间无效。两路样本相差 `86,000 µs`。CRC 应为 `0xEC7A719A`。

```text
40 42 0F 00 00 00 00 00 10 A4 00 00 00 00 14 C1
7C 03 00 00 00 F4 01 00 15 2B 2C BA 00 00 00 00
EB BA 62 2D FD FF FF FF 72 E1 01 00 00 00 A0 3F
00 00 20 C0 00 00 00 3F 00 00 00 00 00 00 00 00
7D 03 0C 00 0B 00 00 00 00 00 00 00 9A 71 7A EC
```

## 6. 写卡、故障恢复与离线导出

- 挂载预先格式化为 FAT32 的卡。用 `FA_CREATE_NEW` 依次创建 `LOG0001.BIN` 等新文件，保留已有文件。新文件先写入并同步文件头，再追加数据帧。计划重启或停止记录时，先写出剩余完整帧，再 `f_close()`。
- 日志队列以 64 帧为起点，另设一个 4 KiB 打包缓冲。每次打包完整帧，4 KiB 最多容纳 51 帧。若达到此批量阈值，或最早待写帧已等待约 1 秒，发起一次写入。持续记录时约每 2 秒同步一次。以 5 帧/秒计算，数据量约 400 B/s，即 1.44 MB/小时。
- `f_write()` 同时检查返回值和实际写入字节数。`f_sync()` 对应的块设备 `CTRL_SYNC` 必须等待卡就绪，写入和等待均有超时。分别记录最长写入和同步耗时，观察卡接近写满时的行为。同步间隔是台架起始值，实际掉电损失由实测确定。
- 拔卡、卡满、短写或同步失败时，将存储标记为故障。继续接收和组帧，串口报告故障，日志队列满后丢新帧并累计计数。对结果不确定的写入批次计数并跳过重放，避免重试造成重复记录。通过有间隔的挂载重试恢复，恢复后创建新文件，保留当前采集时间和序号。
- SDMMC1 首版用 HAL 块读写验证。有必要降低 CPU 占用时再评估 IDMA。启用 IDMA 时，块设备使用 SDMMC1 可访问的 AXI SRAM 作为专用缓冲，核对链接映射、缓存行对齐及 D-Cache 一致性处理。
- 导出脚本先校验文件头，再以 80 字节为单位核对 CRC。跳过 CRC 错误的整帧并报告文件偏移和序号缺口，忽略不足 80 字节的末尾并报告剩余字节数。CSV 每帧一行，时间戳和时间偏移保留整数精度，缺样或无效的数值导出为空，同时保留有效标志、节点 ID 和定位状态。

## 7. 实施顺序与验收

| 阶段 | 验收内容 |
|---|---|
| Keil 工程和板级资源 | 从仓库中的 `.uvprojx` 打开并完整编译，用 DAPLink/SWD 下载并进入调试。确认串口、TIM2、FDCAN1 和 SDMMC1 正常初始化 |
| CAN1 单节点 | 分别冷启动 GPS 和 SKYE2，检查三阶段分配、节点 ID、心跳、原始频率和 CRC。用已有抓包回放核对解码字段 |
| CAN1 双节点 | 验证两台同时冷启动、独立断电重连和主控复位，确认节点 ID 唯一。GPS 室内保留 `NO_FIX`，SKYE2 静置与吹气压差应有清楚变化 |
| 组帧和导出 | 核对样例文件头与 CRC。回放验证每 10 条空速报文抽 1 条、跨窗口重组、写卡后积压回放、重复候选、缺样、`NO_FIX`、时间戳回绕和短文件尾。帧时间差应等于序号差乘以 200 ms，序号缺口应可解释 |
| 连续记录 | 双节点同时记录约 30 分钟。核对帧率、两路有效样本率与缺样计数，记录 CAN FIFO 丢帧、软件队列溢出、CRC 错误、队列高水位和最长写卡停顿。稳定台架条件下硬件和软件接收丢帧、日志队列溢出及 CRC 错误目标为 0 |
| 故障与恢复 | 分别测试拔卡、卡满、写卡期间拔掉再接回传感器，以及突然断电。确认 CAN 恢复处理、重挂载新建文件、旧文件仍可检查，并测量掉电前最后可恢复记录的时间 |

## 8. 参考资料

- [项目背景](../project_background.md)与[系统架构](../system_architecture.md)
- [开发板 V1.2 原理图](<../Reference/Core board information/SCH_STM32H7x_DAP_CORE_4C_V1.2_2026-06-12.pdf>)与[开发板 CAN485 原理图](<../Reference/Core board information/SCH_STM32H7x_DAP_CORE_CAN485_原理图_2025-12-11.pdf>)
- [GPS 测试结果](../GPS测试结果.md)与[空速计测试结果](../空速计测试结果.md)
- [DroneCAN 应用层规范](https://dronecan.github.io/Specification/6._Application_level_functions/)、[libcanard](https://github.com/dronecan/libcanard)与[DSDL 编译器](https://dronecan.github.io/Implementations/dronecan_dsdlc/)
- [ST STM32H723 参考手册](https://www.st.com/resource/en/reference_manual/dm00603761-stm32h723733-stm32h725735-and-stm32h730-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)与[STM32H7 SDMMC 应用说明 AN5200](https://www.st.com/resource/en/application_note/an5200-getting-started-with-the-stm32h7-mcu-sdmmc-host-controller-stmicroelectronics.pdf)
- [ST FDCAN 内核时钟与 APB 时钟约束](https://community.st.com/stm32-mcus-60/faq-fixing-stm32-fdcan-communication-disruptions-apb-bus-kernel-and-time-quanta-clocks-141123)
- [FatFs 批量写入说明](https://elm-chan.org/fsw/ff/doc/write.html)、[文件同步说明](https://elm-chan.org/fsw/ff/doc/sync.html)与[块设备写入接口](https://elm-chan.org/fsw/ff/doc/dwrite.html)
- [CUAV NEO 4SE](https://doc.cuav.net/en/gnss/neo-series/neo-4se.html)与[CUAV SKYE2](https://doc.cuav.net/others/skye/en/skye2.html)
- [Keil STM32H7xx_DFP](https://www.keil.arm.com/packs/stm32h7xx_dfp-keil/overview/)与[CMSIS-DAP 调试说明](https://www.keil.com/support/man/docs/dapdebug/default.htm)
