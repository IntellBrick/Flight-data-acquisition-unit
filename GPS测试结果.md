# GPS 测试结果

日期：2026-10-13
被测设备：CUAV NEO 4SE GPS 模块 ×1（DroneCAN 节点）
测试环境：Makerbase CANable 2.0 USB-CAN 分析仪（slcan 固件，COM10）+ 独立 5 V 供电 + 室内台架
参考实现：`tools/gps_alloc_test.py`（含与 UAVCAN v0 规范示例逐字节比对通过的分配器与解码器，可作固件移植参照）

## 1. 测试结论

| 项目 | 结果 |
|---|---|
| CAN 通信链路 | ✅ 1 Mbit/s 经典 CAN 稳定通信，180 s 连续运行 27,832 帧，0 错误帧、0 CRC 失败 |
| 动态节点 ID 分配 | ✅ 按规范三阶段交换完成，耗时 0.44 s，模块获得节点 ID 125 |
| 设备识别 | ✅ `net.cuav.neo-4-se`，软件 1.4，硬件 0.5，vcs `24bff26b` |
| 节点状态 | ✅ NodeStatus 1 Hz：health=OK，mode=OPERATIONAL，模块自评估 healthy=True、ARMABLE |
| GNSS 数据解析 | ✅ gnss.Fix2 5.00 Hz 稳定接收并正确解码（位置/速度/卫星数/DOP/时间戳） |
| 定位状态 | 室内测试：NO_FIX、0 颗卫星（预期现象，非故障）；板载磁力计读数正常 |
| CAN FD 需求 | **不需要**。模块 `CAN1_FD_EN_MODE=2`（Auto）在经典 CAN 总线上按经典帧正常工作 |

模块唯一 ID（16 字节，来自分配交换，与 GetNodeInfo→HardwareVersion 返回值逐字节一致）：

```
47 00 07 00 0A 51 34 33 34 33 30 37 00 00 00 00
```

## 2. 总线与协议参数（固件配置依据）

| 参数 | 实测/规范值 | 说明 |
|---|---|---|
| 位速率 | **1 000 000 bit/s** | 模块默认值，实测有效 |
| 帧格式 | CAN 2.0B 扩展帧（29 位 ID） | 所有 DroneCAN 传输均用扩展帧 |
| 帧模式 | 经典 CAN（非 FD） | 主控侧保持经典帧即可 |
| 采样点 | 87.5%（适配器侧） | 1 Mbit/s 常规配置 |
| 总线负载（单模块） | ~155 帧/s ≈ 2% | 含分配后的全部消息 |
| 主控（分配器）节点 ID | 建议 100（静态） | 不得用 0；126/127 为网络维护保留 |
| 模块被分配节点 ID | 125 | 模块请求 preferred=0（无偏好）时按规范授予最大可用值 125 |
| 分配器响应优先级 | 30（0x1E） | 任意偶数值均可；模块自身发布用优先级 24（0x18） |

### 2.1 CAN ID 位布局（UAVCAN v0）

| 帧类型 | bit28..24 | bit23..8 | bit7 | bit6..0 |
|---|---|---|---|---|
| 普通消息 | 优先级（5 位，0 最高） | 数据类型 ID（16 位） | 0 | 源节点 ID |
| 匿名消息（分配请求） | 优先级 | 判别码（14 位，bit23..10）+ 数据类型 ID 低 2 位（bit9..8） | 0 | 0 |
| 服务帧 | 优先级 | 服务类型（8 位，bit23..16）+ 请求标志（bit15）+ 目标节点（bit14..8） | 1 | 源节点 ID |

识别匿名分配请求的条件：`(id & 0x7F) == 0 && (id & 0x80) == 0 && ((id >> 8) & 3) == 1`（Allocation 数据类型 ID=1 的低 2 位）。

### 2.2 实测 CAN ID（模块节点 ID=125，优先级 24）

| 消息 | CAN ID | 数据类型 ID |
|---|---|---|
| NodeStatus | `0x1801557D` | 341 |
| gnss.Fix2 | `0x1804277D` | 1063 |
| gnss.Auxiliary | `0x1804257D` | 1061 |
| ahrs.MagneticFieldStrength2 | `0x1803EA7D` | 1002 |
| ardupilot.gnss.Status | `0x184E237D` | 20003 |
| 分配请求（匿名） | 如 `0x183F9900`（判别码随机） | 1（仅低 2 位） |
| 分配器响应（节点 100 发） | `0x1E000164` | 1 |
| GetNodeInfo 请求（100→125） | `0x1E01FDE4` | 服务 1 |

## 3. 初始化流程：动态节点 ID 分配（正确方式）

模块上电后 `CAN_NODE_ID=0`，以约 1 Hz 周期发送匿名单帧 Allocation 请求，**直到被正确授权才发布任何数据**。分配交换共 3 个阶段，unique_id 按 **6+6+4 = 16 字节** 分段交付：

```
模块(匿名)                          分配器(节点100)
  stage1 请求 (first_part=1, 6B)  →
                                  ← follow-up：0x00 + 已收 6 字节
  stage2 请求 (first_part=0, 6B)  →
                                  ← follow-up：0x00 + 已收 12 字节（多帧）
  stage3 请求 (first_part=0, 4B)  →
                                  ← grant：(节点ID<<1) + 完整 16 字节（多帧）
  模块立即以新节点 ID 上线（不重启，uptime 连续），停止请求
```

### 3.1 Allocation 消息格式

载荷第 0 字节 = `(node_id << 1) | first_part_of_unique_id`，其后直接跟 unique_id 字节（无长度前缀）：

| 报文 | 载荷 |
|---|---|
| 模块 stage1 请求 | `01` + 6 字节（例：`01 47 00 07 00 0A 51`） |
| 模块 stage2/3 请求 | `00` + 6/4 字节 |
| 分配器 follow-up | `00` + 已累计的全部字节（6 B 单帧；12 B 多帧） |
| 分配器 grant | `(ID<<1)`（ID=125 时为 `FA`）+ 完整 16 字节 |

### 3.2 时序参数（规范值）

| 参数 | 值 | 说明 |
|---|---|---|
| 模块请求周期 Trequest | 600–1000 ms（随机） | 收到任何 Allocation 消息后重新计时 |
| 模块响应延迟 Tfollowup | 0–400 ms（随机） | 收到 follow-up 后到发出下一阶段的延迟 |
| 分配器跟随超时 | **500 ms** | 超时未收到下一阶段请求即清空状态、重新等 stage1 |
| 分配器响应要求 | 每个请求只回一次，且尽快（实测 <50 ms 即可） | 在 Tfollowup 期间发送多余的 Allocation 报文会中止模块的推进 |

### 3.3 多帧传输规则（收发通用）

| 规则 | 内容 |
|---|---|
| 分帧 | 载荷 >7 字节即多帧：`CRC16（2 字节小端）+ 载荷` 按 **7 字节/帧** 切分 |
| 传输 CRC | CRC-16-CCITT-FALSE（多项式 0x1021、初值 0xFFFF、不反转），输入 = **数据类型签名 8 字节小端 + 载荷**（缺签名前缀会导致对端静默丢弃） |
| 尾字节 | bit7=S（首帧 1）、bit6=E（末帧 1）、bit5=Toggle（首帧 0，此后逐帧取反）、bit4..0=传输 ID（同一传输内恒定，不同传输递增，模 32） |
| 接收校验 | 按（源节点、数据类型、传输 ID）重组；Toggle 必须逐帧交替；建议 1 s 传输超时清理 |

### 3.4 数据类型签名（CRC 前缀用，pydronecan DSDL 编译器核算）

| 数据类型 | 签名（十六进制，使用时按小端发送） |
|---|---|
| dynamic_node_id.Allocation（ID 1） | `0B2A812620A11D40` |
| gnss.Fix（1060） | `54C1572B9E07F297` |
| gnss.Fix2（1063） | `CA41E7000F37435F` |
| gnss.Auxiliary（1061） | `9BE8BDC4C3DBBFD2` |
| NodeStatus（341） | `0F0868D0C1A7C6F1` |
| GetNodeInfo（服务 1） | `EE468A8121C46A9E` |

### 3.5 实测交换报文（可直接用作固件比对样例）

```
RX 183F9900  01 47 00 07 00 0A 51 C0     ← stage1 请求（尾字节含 tid）
TX 1E000164  00 47 00 07 00 0A 51 C0     ← follow-up（单帧）
RX 18062500  00 34 33 34 33 30 37 CE     ← stage2 请求
TX 1E000164  CRC(2B小端)+00+12字节      ← follow-up（3 帧）
RX 18221900  00 00 00 00 00 CF           ← stage3 请求（末 4 字节为 00）
TX 1E000164  CRC(2B小端)+FA+16字节      ← grant 节点 ID=125（3 帧）
RX 1801557D  9B 0F 00 00 00 00 00 C2     ← NodeStatus 上线（uptime=3995s，OK/OPERATIONAL）
```

## 4. 模块上电台账（订阅与缓冲设计依据）

| 消息 | 数据类型 ID | 频率 | 帧数/传输 | 载荷长度 | 用途 |
|---|---|---|---|---|---|
| uavcan.protocol.NodeStatus | 341 | 1.00 Hz | 1 | 7 B | 节点在线/健康监测（超时 3 s 判离线） |
| uavcan.equipment.gnss.**Fix2** | 1063 | 5.00 Hz | 10 | 68 B | **定位解（主数据源）** |
| uavcan.equipment.gnss.Auxiliary | 1061 | 4.99 Hz | 3 | 19 B | vdop/hdop |
| uavcan.equipment.ahrs.MagneticFieldStrength2 | 1002 | 83.6 Hz | 1 | 7 B | 板载磁力计（本项目可不用） |
| ardupilot.gnss.Status | 20003 | 4.99 Hz | 1 | 7 B | GNSS 自检状态（healthy/error_codes） |

设计要点：单模块合计约 **155 帧/s**（其中磁力计占 84 帧/s）；重组缓冲最大按 Fix2 的 **70 字节（2 CRC + 68 载荷）** 取整即可；若不需要磁力计数据，固件验收滤波器可直接过滤 1002 以降低中断负载。

## 5. 关键消息解码格式

DSDL 位序规则（两条）：字段按声明顺序从最高位起填充；多字节整数为小端。字节对齐字段直接小端读取，跨字节位域按大端位序提取（参考 `tools/gps_alloc_test.py` 的 `BitStream`）。

### 5.1 gnss.Fix2（1063，68 字节实测载荷）

| 字节偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0–6 | timestamp | uint56 LE | µs |
| 7–13 | gnss_timestamp | uint56 LE | µs；无定位时为 0 |
| 14 | gnss_time_standard | bit7..5 | 0=NONE 1=TAI 2=UTC 3=GPS |
| 15 | num_leap_seconds | uint8 | 0=未知 |
| 16–31 | **位置块（128 位）** | 位域 | 依次：经度 int37（×1e-8 度）、纬度 int37（×1e-8 度）、椭球高 int27（mm）、海拔 MSL int27（mm） |
| 32–43 | ned_velocity | float32[3] LE | 北/东/天负方向，m/s |
| 44 | sats_used / status | bit7..2 / bit1..0 | status：0=NO_FIX 1=TIME_ONLY 2=2D 3=3D |
| 45–46 | mode / sub_mode | uint4 / uint6 | 0=SINGLE 1=DGPS 2=RTK 3=PPP |
| 47 起 | covariance | float16[≤36]（6 位长度前缀）、pdop float16 | 实测含 9 元协方差 + pdop |

室内无定位时实测：`status=NO_FIX, sats=0, pdop=100.0, lat=lon=0`。

### 5.2 NodeStatus（341，7 字节单帧）

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0–3 | uptime | uint32 LE，秒 |
| 4 | health(bit7..6) / mode(bit5..3) / sub_mode(bit2..0) | health：0=OK；mode：0=OPERATIONAL 1=INITIALIZATION |
| 5–6 | vssc | uint16 LE |

### 5.3 ardupilot.gnss.Status（20003，7 字节单帧）

error_codes uint32（实测 0）、healthy bool（实测 True）、status uint23（bit1=ARMABLE，实测置位）。可作为 GNSS 子系统故障判据之一。

### 5.4 GetNodeInfo（服务 1，可选）

请求：空载荷单帧，CAN ID = `(优先级<<24)|(1<<16)|(1<<15)|(目标节点<<8)|(1<<7)|自身节点`。响应（多帧）含 NodeStatus + 软/硬件版本 + **16 字节 unique_id** + 名称（实测 `net.cuav.neo-4-se`）。建议上电分配完成后查询一次并记录，用于节点识别与双 GPS 区分。

## 6. 单片机侧实现建议（STM32H723 FDCAN）

1. **FDCAN1 配置**：经典 CAN 模式、1 Mbit/s、仅扩展帧接收；RX FIFO 中断入口立即打统一单调时间戳再入队（对应架构文档的采集层要求）。
2. **验收滤波**：若不订阅磁力计，过滤掉 dtid 1002；匿名分配请求用软件判据（见 2.1）识别即可。
3. **分配器**：按第 3 节状态机实现（累计 6/12/16 字节三档），对每个请求只回一帧/一次传输，500 ms 超时清状态；**全程保持运行**以应对模块断电重启（动态分配的节点 ID 不保持，模块每次上电都会重新请求）。
4. **重组器**：按（源节点、数据类型、传输 ID）建表，Toggle 交替校验，CRC 校验必须带第 3.4 节的签名前缀，1 s 超时清理。
5. **记录内容**：每条 Fix2 记录节点 ID、位置块原始值、status/sats、gnss_timestamp 与本地接收时间戳（供 PPS/UTC 映射与双 GPS 对比）；NodeStatus 超时 3 s 判节点离线并记录事件。
6. **双 GPS 接入**：两台 NEO 4SE 同时请求时分配器需按判别码/状态隔离多路交换，或逐台上电分配后再上第二台；每次分配记录 unique_id ↔ 节点 ID 映射，作为稳定设备标识。

## 7. 硬件连接与配置注意事项

| 项目 | 要求 | 备注 |
|---|---|---|
| 模块供电 | 4.7–5.2 V 独立电源 | 不得从 USB-CAN 适配器 CAN 口取电（无 5 V 输出）；正式设计中也不由主控 5 V 轨供电 |
| **共地** | 模块供电电源 GND 必须与 CAN 主控/适配器 GND 连通 | 未共地表现为总线完全静默（此前一次排查的直接原因） |
| CAN_H/CAN_L | **按信号名对应，不按连接器引脚序号** | 模块数据口 GHR-04V-S（4P）：5V / CAN_H / CAN_L / GND，以丝印为准 |
| 终端电阻 | 台架短电缆（≤30 cm）无终端可通信（已实测）；装机总线须**仅两端各 120 Ω** | 断电时 H–L 间等效电阻应约 60 Ω |
| 天线 | 外置有源天线，需天空视野 | 室内 NO_FIX/0 卫星为正常现象；有视野后冷启动约 30 s–数分钟出 3D_FIX |
| CAN FD | 模块为自动模式，跟随总线 | 主控侧保持经典帧；不要在同一总线上发 FD 帧 |
| 节点 ID | 模块每次上电重新请求分配 | 分配器节点 ID 固定（如 100），126/127 保留不用 |
| 上电顺序 | 先起总线主控（分配器）再上电模块亦可；模块会持续请求直至成功 | 分配器晚于模块上电没有影响 |

## 8. 原始数据存档

| 内容 | 位置 |
|---|---|
| 测试脚本（分配器 + 解码 + 自检） | `tools/gps_alloc_test.py` |
| 首次分配运行（JSON/MD/帧日志） | `reports/gps_alloc_test_20261013_163858.*` |
| 已分配状态复测 | `reports/gps_alloc_test_20261013_164415.*` |
| 180 s 稳定性运行 | `reports/gps_alloc_test_20261013_164441.*` |
| 本次排查过程与根因分析 | `reports/gps_allocation_success_20261013.md` |
