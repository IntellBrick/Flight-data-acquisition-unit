# CUAV NEO 4SE GPS 模块 DroneCAN 测试成功报告（2026-10-13）

日期：2026-10-13
设备：Makerbase CANable 2.0（slcan 固件 `16e7497-dirty github.com/normaldotcom/canable2.git`，`16D0:117E`，COM10，1 Mbit/s 经典 CAN）+ CUAV NEO 4SE GPS 模块（`net.cuav.neo-4-se`，软件 1.4，硬件 0.5，vcs `24bff26b`）
测试工具：`tools/gps_alloc_test.py`（本次新写，含 UAVCAN v0 动态节点 ID 分配器 + GNSS 解码 + 自检）

## 1. 结论摘要

**测试通过（PASS）：GPS 模块确认可正常工作，位置信息消息（gnss.Fix2）已能稳定接收并正确解码。**

| 项目 | 结果 |
|---|---|
| 动态节点 ID 分配 | ✅ 成功，3 阶段交换 0.44 s 完成，模块获得节点 ID 125 |
| 模块身份确认 | ✅ GetNodeInfo：`net.cuav.neo-4-se` sw=1.4 hw=0.5，unique_id 与授权值完全一致 |
| 节点健康状态 | ✅ NodeStatus 1 Hz：health=OK，mode=OPERATIONAL |
| GNSS 数据流 | ✅ gnss.Fix2 (dtid 1063) 5 Hz、gnss.Auxiliary (1061) 15 Hz、ardupilot.gnss.Status (20003) 5 Hz，全部正确解码；180 s 稳定性运行 27,832 帧、CRC 失败 0 |
| 板载磁力计 | ✅ MagneticFieldStrength2 (1002) ~83 Hz，读数 [-0.135, -0.225, 0.618] G，为合理地磁场值 |
| 模块自评估 | ✅ ardupilot.gnss.Status：error_codes=0，healthy=True，STATUS_ARMABLE |
| 位置锁定 | ⚠️ **NO_FIX，0 颗卫星**（模块侧无故障；需将天线接好并放到有天空视野处，见第 5 节） |

**此前"上电后设备进入等待 ID 分配状态"的阻塞问题已彻底解决，且证明无需 CAN FD、无需飞控/LGC 地面站——纯经典 CAN + 正确的规范实现即可。**

## 2. 之前 6 版分配器失败的根因（3 个叠加的 bug）

前次会话（`reports/can_diagnosis_20260914.md` 第 7.3 节）的 6 版自建分配器（`tools/dronecan_allocator1..6.py`）都能触发 stage 2，但模块始终不采纳授权，当时推测需要 CAN FD 单帧交付 16 字节 unique_id。**该推测不成立**，真实根因是以下三个叠加缺陷：

### 2.1 缺少第 3 阶段交换（协议状态机不完整）

规范规定 allocatee 的 16 字节 unique_id 分 **3 个单帧阶段**交付（6+6+4 字节）：

```
stage1(first_part=1, 6B) → 分配器回发 6B follow-up → stage2(6B) → 分配器回发 12B follow-up → stage3(4B) → 分配器发 grant
```

旧分配器在收到 stage 2（累计 12 字节）后**直接授权**，从不发送 12 字节 follow-up，模块因此永远发不出携带最后 4 字节的 stage 3 请求。而规则 E 要求授权报文中的 unique_id 必须**完整匹配 16 字节**（本模块的 unique_id 实际以 `00 00 00 00` 结尾，最后 4 字节只能通过 stage 3 获知）。

### 2.2 多帧传输 CRC 计算缺少数据类型签名前缀

UAVCAN v0 的传输 CRC = CRC-16-CCITT-FALSE(**数据类型签名 8 字节小端 + payload**)。旧代码只对 payload 计算 CRC，模块端重组器按 CRC 校验失败丢弃了所有多帧 grant——即使发对了内容也到不了模块。

### 2.3 重组器 toggle 位判断写反（影响接收侧）

`can_comm_test.py` 的 `Reassembler` 把"toggle 位必须在帧间交替"写成了"必须相等"，会丢弃一切多帧传输的第 2 帧。此前从未暴露是因为模块（匿名状态下）只发单帧。

### 2.4 其他修正（解码正确性）

- v0 优先级是 **5 位**（bits 28–24），服务标志在 bit 7（旧代码按 4 位优先级、bit 24 服务标志理解）。
- NodeStatus 的 health/mode/sub_mode 按 MSB 优先位序解码（旧代码 health 与 sub_mode 颠倒）。
- gnss.Fix (1060) 实际是 `int37/int27` 压缩坐标布局（56 位时间戳 ×2 + 128 位位置块 + float16 速度），旧解码器按 float64 经纬度解读是错的。本模块实际发布 Fix2 (1063)。

新脚本 `tools/gps_alloc_test.py` 用 UAVCAN v0 规范自带的示例报文日志做了逐字节自检（`--self-test`）：follow-up 与 grant 的帧序列、CRC（0xB005/0xBA29）与规范示例完全一致。数据类型签名用 pydronecan 官方 DSDL 编译器计算并交叉验证。

## 3. 实测分配过程（原始帧日志摘录）

```
t+0.053  183F9900  01 47 00 07 00 0A 51 C0   ← stage1 请求（first_part=1, 6B）
         （分配器单帧回发 00 + 6B echo）
t+0.240  18062500  00 34 33 34 33 30 37 CE   ← stage2 请求（6B，累计 12B）
         （分配器 3 帧回发 00 + 12B echo，CRC 正确）
t+0.442  18221900  00 00 00 00 00 CF         ← stage3 请求（4B，unique_id 以 4 个 00 结尾）
         （分配器 3 帧发送 grant：node_id=125 + 完整 16B unique_id）
t+1.14   节点 125 上线：NodeStatus health=OK mode=OPERATIONAL（模块未重启，uptime 连续）
```

- 授权后模块**不重启**即切换为节点 125（uptime 从 3995 s 连续累加）。
- 分配器节点 ID=100；模块请求 preferred=0（无偏好），按规范授予 125。
- 模块 unique ID：`47 00 07 00 0A 51 34 33 34 33 30 37 00 00 00 00`（与 GetNodeInfo→HardwareVersion 返回值逐字节一致，双重确认）。

## 4. 模块上电台账（M2 里程碑素材）

| 数据类型 | ID | 频率 | 说明 |
|---|---|---|---|
| uavcan.protocol.NodeStatus | 341 | 1 Hz | health=OK, mode=OPERATIONAL |
| uavcan.equipment.gnss.Fix2 | 1063 | 5 Hz | 定位解（当前 NO_FIX） |
| uavcan.equipment.gnss.Auxiliary | 1061 | 15 Hz | vdop/hdop |
| uavcan.equipment.ahrs.MagneticFieldStrength2 | 1002 | ~83 Hz | 板载磁力计（高斯） |
| ardupilot.gnss.Status | 20003 | 5 Hz | error_codes=0, healthy=True, ARMABLE |

总线：1 Mbit/s 经典 CAN，无错误帧、无 CRC 失败；消息全部为多帧广播，重组正确。

## 5. 当前唯一未完成项：卫星锁定（NO_FIX，0 颗卫星）

模块自检 healthy、数据流正常，但 Fix2 中 `sats_used=0, status=NO_FIX, pdop=100, gnss_ts=0`。这是接收问题而非模块/总线问题。请检查：

1. **外置有源天线是否已接到模块的 SMA 座**（未接天线时表现为永久 0 卫星）。
2. 天线放置位置是否有天空视野（室内/靠窗/室外）。冷启动首次定位通常需要 30 s–数分钟天空视野。

放置好天线后直接复测（模块已分配 ID 时脚本会自动跳过分配阶段）：

```powershell
$env:PYTHONPATH="D:\Lab\Flight-data-acquisition-unit\pylibs;D:\Lab\Flight-data-acquisition-unit\tools"
python D:\Lab\Flight-data-acquisition-unit\tools\gps_alloc_test.py --port COM10 --monitor 120
```

出现 `status=3D_FIX` 及非零经纬度即为完整闭环。注意：动态分配的节点 ID **断电不保持**，重新上电后脚本会自动重新分配。

## 6. 对项目后续工作的结论

1. **M2（CANdleLight/PC 单节点验证）对 GPS #1 实质完成**：位速率 1 Mbit/s、动态节点 ID 机制、消息类型与频率台账均已实测记录；且证明 CANable 2.0（slcan）可胜任该验证工作，旧 CANdleLight 停用。
2. 模块 `CAN1_FD_EN_MODE=2`（Auto）在经典 CAN 总线上自动工作于经典模式，**不需要 CAN FD 适配器**；此前"需 FD"的假设由本次实测否定。
3. 自建分配器可直接用于后续 STM32 主控的移植参考（纯状态机 + CRC，无外部依赖），也为接入第二台 NEO 4SE 与 SKYE2（阶段 B/C/D）提供了现成工具。
4. 三次运行报告与原始帧日志：
   - `reports/gps_alloc_test_20261013_163858.*`——首次运行：完整 3 阶段分配 + 授权 + 30 s 监测；
   - `reports/gps_alloc_test_20261013_164415.*`——复测：验证"模块已分配"快速路径（0 请求直接进入监测）；
   - `reports/gps_alloc_test_20261013_164441.*`——180 s 稳定性监测：27,832 帧，CRC 失败 0，Fix2 精确 5.00 Hz、NodeStatus 1.00 Hz、磁力计 ~83.6 Hz，长期解码无误。
