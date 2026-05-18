# 阶段 D 任务日志：UART 链路与仿真循环

> 对照计划：`docs/vision_line_tracking_plan_v2.md` §12 阶段 D
> 起止：2026-05-17 ~
> 负责人：—
> 当前状态：**代码完成（CONFIG_VERSION=phaseD-0.1，含 comms/ 四模块 + vision_line_tracking.py
> 集成 + tools/uart_dump.py）；板端联调与 10 min 连续验收待回填**

---

## 1. 任务清单与状态

| # | 任务 | 计划文档锚点 | 状态 | 交付物 |
|---|------|--------------|------|--------|
| D-1 | 实现 `comms/ms901m.py`（MS901MParser 字节级状态机） | §10.2 / Stage4-K230-Side §2 | ✅ 已完成 | `comms/ms901m.py`：MS901MParser + self_test |
| D-2 | 实现 `comms/frame.py`（CRC16 / 帧编码 / MCUFrameParser） | §10.1 / Stage4-K230-Side §3 | ✅ 已完成 | `comms/frame.py`：crc16_ccitt / encode_frame / MCUFrameParser |
| D-3 | 实现 `comms/protocol.py`（CMD 枚举 / 业务帧打包解包） | §10.5 / Stage4-K230-Side §4 | ✅ 已完成 | `comms/protocol.py`：CMD 常量 / parse_* / make_* |
| D-4 | 实现 `comms/uart_link.py`（ImuLink + McuLink） | §10.6 / Stage4-K230-Side §5 | ✅ 已完成 | `comms/uart_link.py`：ImuLink / McuLink + bench fallback |
| D-5 | `comms/__init__.py` 公开接口 | §11.1 | ✅ 已完成 | `comms/__init__.py` |
| D-6 | `config.py` Stage D 通讯常量节 | §11.2 | ✅ 已完成 | `config.py`：COMMS_ENABLE / UART ID / 超时 / 节拍 |
| D-7 | `vision_line_tracking.py` 集成 comms | §12 阶段 D | ✅ 已完成 | `vision_line_tracking.py`：_setup_comms / 主循环 drain / 定时发送 / OSD 行 / 日志 |
| D-8 | `tools/uart_dump.py` PC 端抓包工具 | §10.8 / §13.1 | ✅ 已完成 | `tools/uart_dump.py`：pyserial + 人可读 / CSV + loopback 自测 |
| D-9 | 双向心跳 + 超时降级：10 min 丢帧率 ≤ 0.1% | §12 阶段 D 验收 | ⏳ 待板端 | — |
| D-10 | 断开 K230 → 主控 200 ms 内进入平衡告警 | §12 阶段 D 验收 | ⏳ 待板端 | — |
| D-11 | 模拟主控冻结 → K230 200 ms 内发 `flags.noimu` | §12 阶段 D 验收 | ⏳ 待板端 | — |

---

## 2. 代码侧实现纪要

### 2.1 文件结构（阶段 D 追加部分）

```
K230/
├── comms/
│   ├── __init__.py        # 公开 exports（MS901MParser / MCUFrameParser / ImuLink / McuLink / 协议常量）
│   ├── ms901m.py          # MS901MParser：字节级状态机，解析 ID 0x01/0x02/0x03
│   ├── frame.py           # crc16_ccitt / encode_frame / MCUFrameParser
│   ├── protocol.py        # CMD 常量 / 安全状态 / parse_* / make_*
│   └── uart_link.py       # ImuLink（UART1 115200）/ McuLink（UART2 921600）
├── config.py              # CONFIG_VERSION=phaseD-0.1，新增 Stage D 节
├── tools/
│   └── uart_dump.py       # PC 端 pyserial 抓包 / 解码 / loopback 工具
├── vision_line_tracking.py  # 集成：_setup_comms / 主循环 UART drain / 定时发送
└── docs/
    └── TaskLog/
        └── phase_D.md     # 本文件
```

### 2.2 协议规范对照（Stage4-K230-Side.md）

#### 帧格式（MCU 命令链路，UART2 921600）

```
[0xAA][0x55][LEN][CMD][PAYLOAD×LEN][CRC16_LO][CRC16_HI][0x55][0xAA]
```

- CRC16-CCITT：多项式 `0x1021`，初始值 `0xFFFF`，校验范围 `LEN+CMD+PAYLOAD`
- 最大 PAYLOAD：32 字节

#### IMU 直通（UART1 115200）

MS901M 以 115200 8N1 主动推送，K230 仅使用 RX，解析 `0x55 0x55 ID LEN DATA CHECKSUM`。本工程只消费 ID 0x01（姿态）和 0x03（陀螺+加速）；ID 0x02（四元数）也解析但暂不进 OSD/日志（Stage E 云台前馈时启用）。

#### 业务帧（阶段 D 实现，阶段 E 起 target_v/omega 由控制律填充）

| 方向 | CMD | 频率 | Payload 格式 | 说明 |
|------|-----|------|--------------|------|
| MCU→K230 | 0x01 | 20 Hz | `<iBH`（7 B）avg_cps + safety_state + bat_mv | VEHICLE_STATUS |
| MCU→K230 | 0x02 | 1 Hz | `<I`（4 B）uptime_ms | HEARTBEAT_MCU |
| K230→MCU | 0x11 | 40 Hz | `<hhB`（5 B）target_v + target_omega + mode | MOTION_CMD |
| K230→MCU | 0x12 | ~2.5 Hz | `<I`（4 B）uptime_ms | HEARTBEAT_K230 |
| K230→MCU | 0x13 | 按需 | `<Bfff`（13 B）pid_id + kp + ki + kd | PID_INJECT（调试期） |

### 2.3 双向心跳与超时降级

```
MCU 侧：500 ms 无任何 K230 帧 → 运动指令归零 + 平衡告警
K230 侧：
  MCU_TIMEOUT_MS = 500 ms 无 HEARTBEAT_MCU → is_online()=False
  is_online()=False → send_motion 自动改 mode=0（停止）
  HB_SEND_INTERVAL_MS = 400 ms 发一次 HEARTBEAT_K230（< MCU 超时阈值）
  CMD_SEND_INTERVAL_MS = 25 ms 发一次 MOTION_CMD（40 Hz）
```

降级安全层（Stage4-K230-Side.md §6）：
1. MCU 离线时 K230 侧 `send_motion(0, 0, mode=0)` 主动归零
2. `vehicle_safety >= SAFETY_FALLEN(3)` 时 mode 强制为 0
3. `bat_mv < BAT_DEGRADE_MV(9500)` 时 target_v 限幅到 ±BAT_DEGRADE_V_MAX(200)
4. 程序退出 finally 块发一帧 stop（尽力而为）

### 2.4 bench 兼容性

`config.COMMS_ENABLE = False`（默认）时：
- `_setup_comms()` 直接返回 `(None, None)`，不尝试导入 `machine.UART`
- `ImuLink` / `McuLink` 内部若 `machine` 不可用，uart 实例为 None，所有 drain/send 为空操作
- 视觉算法链路（Phase A/B/C）完全不受影响

### 2.4.1 IMU 数据速率与 drain 策略

MS901M 以 **200Hz** 速率主动推送三种帧（每周期 5ms，依次发送 0x01→0x02→0x03）：

| 帧 ID | 内容 | LEN | 完整帧长 |
|:---:|---|:---:|:---:|
| 0x01 | 姿态 roll/pitch/yaw | 6 | **11 B** |
| 0x02 | 四元数 q0~q3 | 8 | **13 B** |
| 0x03 | 陀螺 + 加速度 | 12 | **17 B** |
| — | 合计每周期 | — | **41 B** |

- **总字节率**：200 Hz × 41 B = **8200 B/s**（信道利用率 71.2%，115200 baud = 11520 B/s）
- **每 K230 主循环积累量**：8200 × 48ms ≈ **394 B**

> 若每次 `drain()` 仅读 64 B，连续积压后 UART 硬件缓冲区（通常 256~512B）溢出，导致帧头丢失，坏帧率实测高达 **~23%**。
>
> 修复方案：`drain()` 改为 `while True: read(256)` 循环读至缓冲为空，配合 **`timeout=0` 非阻塞** 打开 UART（否则循环末次 `read()` 会阻塞主循环）。

### 2.5 主循环集成点

```python
# 1. 每帧开头（snapshot 前）drain UART
if imu_link: imu_link.drain()
if mcu_link: mcu_link.drain(now); mcu_online = mcu_link.is_online(now)

# 2. 算法完成后，40 Hz 节拍发 MOTION_CMD
if ticks_diff(now, last_cmd_ms) >= CMD_SEND_INTERVAL_MS:
    mcu_link.send_motion(target_v, target_omega)   # mode 由 is_safe_to_drive() 决定

# 3. HB 节拍（~2.5 Hz）发 HEARTBEAT_K230
if ticks_diff(now, last_hb_ms) >= HB_SEND_INTERVAL_MS:
    mcu_link.send_heartbeat(now)

# 4. 退出前 send_stop()
```

### 2.6 OSD / 日志新增字段（阶段 D 增量）

| 来源 | 字段 | 含义 |
|------|------|------|
| OSD 行（1 Hz） | `MCU:ON/OFF  bat:XXXXmV  cps:±NNN` | MCU 在线状态 / 电压 / 速度；离线时标红 |
| 5 s 日志 | `mcu=ON/OFF bat=XXXXmV cps=±NNN imu_g=N/b=N mcu_g=N/b=N` | 链路质量统计 |

---

## 3. 硬件接线指引

> 本节给出 Stage 4 两路 UART 的完整连线步骤，可直接作为装车检查清单使用。
> 操作前务必**断开所有电源**。

### 3.1 总体拓扑

```
  ┌──────────────────────────────────────────────────────────────────────┐
  │  MS901M（ATK-MS901M，115200 8N1）                                     │
  │  TX ─────────────────────────┬──→ MCU UART3_RX（已完成，MCU 侧接线）  │
  │  GND ──────────────────┐     │                                        │
  └────────────────────────│─────│────────────────────────────────────────┘
                           │     │  Y 分线（直接短接或 100Ω 隔离电阻）
                           │     └──→ K230 IO_4  (Header NO.13, UART1_RXD)
                           │
  ┌──────────────────────────────────────────────────────────────────────┐
  │  MSPM0G3507 MCU                                                       │
  │  UART1_TX / PB6 ─────────────→ K230 IO_6  (Header NO.20, UART2_RXD) │
  │  UART1_RX / PB7 ←───────────── K230 IO_5  (Header NO.17, UART2_TXD) │
  │  GND ──────────────────────────→ K230 GND  (Header NO.2/4/37)        │
  └──────────────────────────────────────────────────────────────────────┘
```

**电平**：MS901M / MCU / K230 全部 3.3 V，直连兼容。  
**禁止**：不得将 K230 的 5 V 电源（Header NO.1）接到 MCU 或 MS901M 的任何信号引脚。

---

### 3.2 K230 LP4 V3.0 使用引脚汇总

> 引脚位置参考 `docs/k230_canmv_docs/pin/LP_PIN.md`，双排 40-pin Header。

| Header NO. | 引脚标签 | 本项目用途 | 连接目标 |
|:---:|---|---|---|
| **2 / 4 / 37** | GND | 公共地 | MCU GND（任选一脚，建议靠近信号线的 NO.37） |
| **13** | JTAG_TDO / IO_4 | UART1_RXD（IMU 直通） | MS901M TX Y 分支路 |
| **18** | JTAG_TDI / IO_3 | UART1_TXD（仅 FPIOA，不接线） | — |
| **17** | UART2_TXD / IO_5 | UART2 TX → MCU RX | MCU PB7（UART1_RX） |
| **20** | UART2_RXD / IO_6 | UART2 RX ← MCU TX | MCU PB6（UART1_TX） |

> **UART 通道可用性**（K230 CanMV UART API 手册）：
> - UART0：小核 SH 占用，**禁用**
> - UART3：大核 SH 占用，**禁用**
> - **UART1 / UART2 / UART4：用户可用**
>
> **UART1 引脚限制**（来自 `fpioa.help()` 实测）：
> - UART1_TXD 只能分配到 **IO_3** 或 IO_9
> - UART1_RXD 只能分配到 **IO_4** 或 IO_10
> - K230 CanMV UART 驱动**要求 TX/RX 同时配置 FPIOA**，否则报 `tx not configured`，即使只接收也不例外
> - IO_3/IO_4 默认功能为 JTAG_TDI/TDO，正常运行期间不占用，可安全复用

---

### 3.3 链路一：MCU 命令链路（UART2，921600 baud，双向）

| 线号 | K230 端 | K230 Header | 方向 | MCU 端 |
|:---:|---|:---:|:---:|---|
| ① | IO_5 (UART2_TXD) | **NO.17** | K230 → MCU | PB7 (UART1_RX) |
| ② | IO_6 (UART2_RXD) | **NO.20** | MCU → K230 | PB6 (UART1_TX) |
| ③ | GND | **NO.37** | 共地 | GND |

**接线步骤**：
1. 用杜邦线连接 K230 Header NO.17（IO_5）→ MCU PB7
2. 用杜邦线连接 K230 Header NO.20（IO_6）← MCU PB6
3. 用杜邦线连接 K230 Header NO.37（GND）→ MCU GND
4. 用万用表核验：K230 GND 与 MCU GND 之间阻值 < 1 Ω（共地良好）

---

### 3.4 链路二：IMU 直通（UART1，115200 baud，仅 RX）

MS901M TX 信号需 **Y 分**同时馈入 MCU UART3_RX 和 K230 UART1_RXD。

| 线号 | K230 端 | K230 Header | 方向 | 来源 |
|:---:|---|:---:|:---:|---|
| ④ | IO_4 (UART1_RXD) | **NO.13** | MS901M → K230 | MS901M TX Y 分支路 |
| — | IO_3 (UART1_TXD) | NO.18 | **不接线** | 仅 FPIOA 软件配置，驱动必须 |
| — | — | — | — | MS901M TX → MCU UART3_RX（已完成） |

**Y 分接法**：

```
MS901M TX ──┬──→ MCU UART3_RX（已有线）
            │
            └──→ K230 Header NO.13（IO_4）
```

短接距离 < 20 cm 时可直接短接；若导线较长（> 30 cm）建议在 K230 分支串联 **100 Ω** 隔离电阻，防止反射干扰 MCU 侧接收。

> K230 侧 IO_3（Header NO.18，UART1_TXD）**无需接任何线**，只需在 FPIOA 软件层配置，否则驱动报 `tx not configured` 拒绝打开 UART1。

---

### 3.5 FPIOA 软件配置（由 `comms/uart_link.py` 自动完成）

`comms/uart_link.py` 中 `ImuLink.__init__` / `McuLink.__init__` 会在 `UART()` 打开前自动调用以下等效操作：

```python
from machine import FPIOA
fpioa = FPIOA()

# 链路二：UART1 TX+RX（IMU，IO_3/IO_4 = Header NO.18/NO.13）
# K230 UART 驱动要求 TX 和 RX 同时配置 FPIOA，即使只使用 RX
fpioa.set_function(3, FPIOA.UART1_TXD)   # IO_3，不接线，仅满足驱动要求
fpioa.set_function(4, FPIOA.UART1_RXD)   # IO_4，接 MS901M TX Y 分线

# 链路一：UART2 TX/RX（MCU 命令，IO_5/IO_6 = Header NO.17/NO.20）
fpioa.set_function(5, FPIOA.UART2_TXD)
fpioa.set_function(6, FPIOA.UART2_RXD)
```

引脚号由 `config.py` 统一管理：

```python
# config.py
IMU_UART1_TX_IO  = 3    # IO_3，Header NO.18（JTAG_TDI 复用，不接线）
IMU_UART1_RX_IO  = 4    # IO_4，Header NO.13（JTAG_TDO 复用，接 MS901M TX）
MCU_UART2_TX_IO  = 5    # IO_5，Header NO.17
MCU_UART2_RX_IO  = 6    # IO_6，Header NO.20
```

> **为何不能用 IO_20**：`fpioa.help()` 实测显示 IO_20 可用功能为 `GPIO20/RESV/RESV/RESV/RESV`，不含 UART 功能，`set_function(20, UART1_RXD)` 会报 `set pin func failed`。UART1 的 RXD 只能分配到 IO_4 或 IO_10。

---

### 3.6 上电前检查清单

**硬件**

- [ ] K230 Header NO.17（IO_5）→ MCU PB7，线序正确（TX→RX）
- [ ] K230 Header NO.20（IO_6）← MCU PB6，线序正确（RX←TX）
- [ ] K230 Header NO.37（GND）→ MCU GND，共地连接
- [ ] K230 Header NO.13（IO_4）← MS901M TX Y 分支路
- [ ] K230 Header NO.18（IO_3）**不接线**（仅 FPIOA 软件配置，驱动要求）
- [ ] MS901M GND → MCU GND → K230 GND，三方共地
- [ ] 无 5 V 电源接入 MCU / MS901M 信号引脚
- [ ] 所有杜邦线插到位（轻拉不脱），无短路

**软件**

- [ ] `config.py` 中 `COMMS_ENABLE = True`
- [ ] `config.py` 中 `IMU_UART_ID = 1`、`MCU_UART_ID = 2`（默认正确）
- [ ] `config.py` 中 `IMU_UART1_TX_IO = 3`、`IMU_UART1_RX_IO = 4`（默认正确，勿改）
- [ ] 运行 `vision_line_tracking.py`，启动日志应出现：
  ```
  [imu_link] UART1 @115200 TX=IO_3 RX=IO_4 opened
  [mcu_link] UART2 @921600 TX=IO_5 RX=IO_6 opened
  ```

---

## 4. PC 端单测覆盖

| 模块 | 测试场景 | 通过判据 |
|------|----------|----------|
| `MS901MParser.self_test` | 构造 ID=0x01 LEN=6 pitch≈90° 帧 | good_frames=1, pitch_deg≈90.0 |
| `MCUFrameParser.self_test` | 往返 VEHICLE_STATUS | 解析字段与原始一致，good=1 bad=0 |
| `protocol.self_test` | make_motion_cmd / parse_vehicle_status / make_heartbeat_k230 | 3 组往返均通过 |
| `McuLink.self_test` | make_motion_cmd v=200 omega=-100 mode=1 | 字段正确 |
| `tools/uart_dump.py --loopback` | TX 短接 RX，发 10 帧 | good≥10, bad=0 |

---

## 5. 验收记录占位（板端联调后回填）

> 阶段 D 验收（plan §12）：
> - 10 min 连续运行，UART 帧丢帧率 ≤ 0.1%（good / (good+bad) ≥ 99.9%）；
> - 主动断开 K230 → 主控 200 ms 内进入平衡告警（MCU `safety_state` 变 DISARMED）；
> - 模拟主控冻结（停发心跳）→ K230 `MCU_TIMEOUT_MS=500ms` 内 `is_online()=False`，MOTION_CMD 改 mode=0。

| 测试场景 | 配置 | 实测 good | 实测 bad | 丢帧率 | 通过 |
|---------|------|-----------|----------|--------|------|
| 10 min 连续运行（MOTION_CMD 40 Hz + HB 2.5 Hz） | COMMS_ENABLE=True, track | _____ | _____ | _____ | ☐ |
| 拔 K230 TX 线 → MCU 平衡告警 | 同上 | — | — | ≤ 200 ms | ☐ |
| 拔 MCU TX 线 → K230 降级 mode=0 | 同上 | — | — | ≤ 500 ms | ☐ |
| loopback 自测 10 帧 | tools/uart_dump.py --loopback | ≥10 | 0 | 0% | ☐ |
| MS901M 直通 10 s | COMMS_ENABLE=True, IMU UART1 | 测 good 帧率 | _____ | — | ☐ |

测试方法：

1. 板端 `config.COMMS_ENABLE = True`，确认接线（UART1 ← MS901M TX Y，UART2 ↔ MCU PB6/PB7，共 GND）；
2. 运行 `vision_line_tracking.py`，等 30 s 进入稳态；
3. 观察 OSD `MCU:ON` 行和 5 s 日志 `mcu=ON ... mcu_g=.../b=...`，验证丢帧率；
4. 拔 K230→MCU TX 线，等待 MCU 侧平衡告警；
5. 拔 MCU→K230 TX 线，观察 OSD `MCU:OFF`（≤ 500 ms），verify MOTION_CMD mode=0；
6. PC 端 `python tools/uart_dump.py --port COM3 --baud 921600` 实时观测。

---

## 6. 已知问题与遗留 TODO

- [ ] **`flags` 位域未打包进 MOTION_CMD**：plan §1.3 / §10.6 里 `flags(uint8)` 含 `degrade`/`lost`/`calib_change` 位。当前 `make_motion_cmd` 仅含 `(v, omega, mode)`（5 字节）；MCU 侧 VEHICLE_STATUS 解析时也只看这三字段。Stage E 控制律接入后需要在 MOTION_CMD 末尾增加 `flags` 字节，同步更新 `protocol.py` 和 MCU 侧帧解析代码。
- [ ] **`seq` 帧序号未实现**：plan §1.3 的 `seq(uint16)` 供主控检测丢帧。当前未打包；Stage E 时补入。
- [ ] **IMU 前馈尚未消费**：`ImuLink.pitch_deg()` 已可读，但 Stage E 前 target_v/omega=0，云台前馈补偿（plan §3.3 / §5.1）留待 Stage E 实现。
- [ ] **target_v / omega 占位为 0**：当前 `MOTION_DEFAULT_V=0` / `MOTION_DEFAULT_OMEGA=0`，实际行驶需要 Stage E 控制律填充（plan §12 阶段 E）。
- [ ] **calib.json schema 版本与 CONFIG_VERSION 联调**：plan §5 TODO 延续到阶段 D——`phaseD-0.1` 改变了 CONFIG_VERSION，若旧版 calib.json 仍在 SD 卡，需确认 `config.load_calibration` 的版本警告不影响 IPM 正常运行（目前仅 warn，不拒绝）。
- [ ] **UART1 RX IO 引脚待装车确认**：当前 `IMU_UART1_RX_IO=20`（LP Header NO.5，空闲引脚），装车后若 IO_20 与其他外设冲突，改 `config.py` 中 `IMU_UART1_RX_IO` 为另一空闲引脚（IO_27/28/30/52/53），FPIOA 配置会自动跟随。
- [ ] **`MCU_TIMEOUT_MS` 与 `HB_SEND_INTERVAL_MS` 装车微调**：当前 500 ms / 400 ms 是理论值，实测 UART 抖动可能需要放宽到 600 ms / 450 ms，或在 Stage E 根据 K230 主循环实际帧周期（~30 ms）重新评估。

---

## 7. 进入下一阶段的前置条件

**代码侧已可进入阶段 E**（controller 前馈 + 反馈 + 限幅）。下列实测项不阻塞代码推进，但**装车联调前**必须完成：

1. §4 验收表全部回填（尤其 10 min 丢帧率 ≤ 0.1%）；
2. `config.COMMS_ENABLE = True` 装车实测通过（MCU 侧能打印 `k230_ON`，K230 OSD 显示 `MCU:ON`）；
3. `tools/uart_dump.py` 在 PC 端捕获到正常的双向帧流，确认 CRC 无误。

---

## 8. 给阶段 E 的接力条目

- **控制律输出**（plan §12 阶段 E）：Stage E `controller.py` 输出的 `(v_ref_mm_s, omega_ref_mrads)` 需要转换为 `MOTION_CMD` 的 `(target_v, target_omega)` 单位：
  - `target_v = v_ref_mm_s / WHEEL_CIRC_MM * ENCODER_CPR / MOTION_SCALE`（由运动学参数决定，当前用 SCALE=10）
  - `target_omega = omega_ref_mrads / OMEGA_SCALE_PER_PERMILLE`
  - 具体换算系数在 Stage E 结合 MCU 侧编码器标定值确定后写回 `config.py`
- **flags 字段**：Stage E 启用控制律时补入 `flags.degrade`（Q_full < Q_DEGRADE）/ `flags.lost`（EMA age ≥ AGE_MAX）；MCU 侧收到 `flags.degrade` 时可选择降速或报警。
- **IMU 前馈**：`imu_link.parser.pitch_deg` / `.gy_dps` 已就绪，Stage E 开始消费：
  - 云台俯仰前馈 = `-pitch_deg × K_FF_PITCH`
  - IPM 动态补偿（可选）：用 `pitch_deg` 实时修正 `GroundMapper` 的 H 矩阵行首元素
- **`seq` 帧序号**：在 `McuLink.send_motion` 里追加递增 `seq` 并打包，MCU 侧 `frame.good_frames - received_seq` 可直接给出丢帧统计。

---

## 9. 日志与 OSD 字段含义速查（阶段 D 增量）

### 9.1 主循环 5 s 日志新增字段

```
[VLT] ... mcu=ON bat=10909mV cps=+0 imu_g=12345/b=0 mcu_g=600/b=0 ...
```

| 字段 | 单位 | 含义 | 健康范围 |
|---|---|---|---|
| `mcu` | str | MCU 在线（ON/OFF） | 始终 ON |
| `bat` | mV | 最新电池电压 | > 9500 mV |
| `cps` | counts/s | 左右轮平均速度（正=前进） | ±5000 视速度目标 |
| `imu_g/b` | 帧数 | MS901M 解析好帧 / 坏帧 | bad 占比 < 0.1% |
| `mcu_g/b` | 帧数 | MCU 命令帧好帧 / 坏帧 | bad 占比 < 0.1% |

> 联调期曾临时打印 `uart_rx/last/avg_us/max_us/mcu_bad/last CRC` 等详细字段，
> 用于定位 UART timeout、MCU 心跳 CRC 查表错误等问题。MCU 侧 CRC 修复后，
> 这些字段已从常规 5 s 日志中移除；底层计数器仍保留，后续若再次排障可重新打开。

历史联调中曾用上述详细字段定位 UART timeout、MS901M 坏帧与 MCU 心跳 CRC 查表错误；根因与修复记录已归档到 `docs/chore/Stage4-K230-Communication.md`，常规运行日志不再输出这些临时字段。

### 9.2 OSD 文本行（阶段 D 增量，约 1Hz 更新）

```
MCU:ON   bat:11250mV  cps:+0
```

MCU 离线时该行标红显示 `MCU:OFF`。

---

## 10. 变更日志

| 日期 | 版本 | 内容 |
|------|------|------|
| 2026-05-17 | phaseD-0.1 | 初版：comms/ 四模块 + vision_line_tracking.py 集成 + tools/uart_dump.py |
| 2026-05-18 | phaseD-0.1 | 补充 §3 硬件接线指引：拓扑图、引脚汇总表、FPIOA 代码说明、上电前检查清单 |
| 2026-05-18 | phaseD-0.2 | 修复 UART1 FPIOA：IO_20 无 UART 功能，改为 IO_3(TX)+IO_4(RX)；uart_link.py 同时配置 TX+RX；更新 config.py、接线指引 |
| 2026-05-18 | phaseD-0.3 | 修复 IMU 坏帧率 ~23%：drain() 改为 while-loop 循环读至缓冲空；_open_uart 加 timeout=0 防循环末尾阻塞；McuLink.drain() 同步改写 |
| 2026-05-18 | phaseD-0.4 | 修复 drain() 阻塞主循环（FPS 12→21）：改用 uart.any()+read(n) 非阻塞模式；read()+default-timeout 阻塞 ~30ms，timeout=0 致 read(n) 在缓冲 <n 时返回 None；any()+read(n) 零阻塞且无副作用 |
| 2026-05-18 | phaseD-0.5 | 修复 imu_g=3/b=1：uart.any() 在 K230 CanMV 缓冲首次读空后始终返回 0（驱动缺陷），导致所有后续 drain() 立即 return；改为 read(_READ_N)+timeout_ms，缓冲有数据时立即返回，无数据时等待至多 timeout 防主循环卡死 |
| 2026-05-18 | phaseD-0.6 | UART 性能优化：IMU 从 512B/10ms 调整为 256B/3ms，MCU 从 128B/5ms 调整为 64B/1ms；新增 5s 日志中的 UART RX 字节与 drain avg/max us，便于确认 UART 不再抢占主循环预算 |
| 2026-05-18 | phaseD-0.7 | 修复 MS901MParser 抗噪声问题：非法 ID/LEN 会计入 bad 并重新同步，避免错位字节把 LEN 解析成大值后触发 bytearray 越界 |
| 2026-05-18 | phaseD-0.8 | 修复 MCU 在线判定：按最近一次合法 MCU 上行帧（VEHICLE_STATUS 或 HEARTBEAT_MCU）刷新在线时间，与 MCU 侧“无任何帧超时”语义对齐；心跳 CRC 异常继续通过 mcu_bad 统计追踪 |
| 2026-05-18 | phaseD-0.9 | MCU 串口联调稳定后收敛 5s 日志：移除 `uart_rx/last/avg_us/max_us/mcu_bad/CRC` 等临时诊断字段，仅保留在线、电压、速度与好坏帧计数 |
