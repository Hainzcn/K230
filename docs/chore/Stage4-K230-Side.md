# K230 端协同工作手册（Stage 4）

> **文档定位**：面向 K230（CanMV）侧开发者，描述与 MCU 通讯所需的全部硬件接线、协议实现和集成逻辑。本文是 [Stage4-K230-Communication.md](Stage4-K230-Communication.md)（MCU 侧实现记录）的对应文档。
>
> **平台假设**：K230 运行 CanMV MicroPython（固件 ≥ 2.x），代码示例均为 MicroPython；如使用 C SDK，数据结构与协议完全一致，可直接对照移植。
>
> **关联文档**：
> - MCU 侧实现记录：[Stage4-K230-Communication.md](Stage4-K230-Communication.md)
> - 引脚分配（MCU 侧）：[Stage0-PinAllocation.md](Stage0-PinAllocation.md)
> - 项目总览：[Overview.md](../Overview/Overview.md)

---

## 1. 硬件接线

K230 需要占用 **两路独立 UART**：

| UART 用途 | K230 引脚 | 连接目标 | 波特率 |
|-----------|----------|---------|--------|
| IMU 直通 | 某路 UART RX（见下） | MS901M TX 线 Y 分出来的支路 | **115200** |
| MCU 命令 | 某路 UART TX + RX | MCU PB6 (TX→K230 RX) / MCU PB7 (RX←K230 TX) | **921600** |

> CanMV K230 板载多路 UART，推荐分配：
> - `UART(1)`：115200，接 MS901M TX Y 分线（**仅 RX 使用**）
> - `UART(2)`：921600，接 MCU UART1（TX+RX 双向）

**电平**：MS901M / MCU / K230 全部 3.3 V，直连兼容，**严格共 GND**（一根 GND 线从 MCU GND 接到 K230 GND）。

**不要**把 K230 的 5 V 电源接到 MCU 或 MS901M 的任何引脚。

---

## 2. MS901M 帧协议（IMU 直通解析）

MS901M 以 115200 8N1 主动推送二进制帧，格式如下：

```
0x55 0x55  ID  LEN  DATA[LEN]  CHECKSUM
```

- **CHECKSUM** = `(0x55 + 0x55 + ID + LEN + DATA[0] + ... + DATA[LEN-1]) & 0xFF`
- 本工程仅需解析以下三个帧 ID（MCU 配置为 200 Hz 上报）：

| ID | 名称 | LEN | DATA 布局 | 量纲 |
|----|------|-----|-----------|------|
| 0x01 | 姿态 | 6 | roll, pitch, yaw（各 int16 LE） | `val / 32768 * 180` → 度 |
| 0x02 | 四元数 | 8 | q0, q1, q2, q3（各 int16 LE） | `val / 32768` |
| 0x03 | 陀螺+加速 | 12 | ax, ay, az, gx, gy, gz（各 int16 LE） | accel: `val / 32768 * 4 g`；gyro: `val / 32768 * 2000 dps` |

> 量程 ±4 g / ±2000 dps 是 MCU 出厂默认配置写入 MS901M Flash 的值，不要更改。

### 2.1 MicroPython 解析器

```python
import struct

class MS901MParser:
    """ATK-MS901M 二进制帧逐字节解析器（仅解析 ID 0x01/0x02/0x03）。"""

    GYRO_FSR_DPS  = 2000.0
    ACCEL_FSR_G   = 4.0

    def __init__(self):
        self._buf = bytearray()
        self._state = 0   # 0=等待0x55_1, 1=等待0x55_2, 2=等待ID,
                          # 3=等待LEN, 4=读DATA, 5=等待CHECKSUM
        self._id   = 0
        self._len  = 0
        self._data = bytearray(16)
        self._didx = 0
        self.roll_deg  = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg   = 0.0
        self.ax_g = self.ay_g = self.az_g = 0.0
        self.gx_dps = self.gy_dps = self.gz_dps = 0.0
        self.q0 = self.q1 = self.q2 = self.q3 = 0.0
        self.has_attitude = False
        self.has_gyro_acc = False
        self.good_frames  = 0
        self.bad_frames   = 0

    def feed(self, data: bytes):
        """喂入任意长度字节，内部逐字节处理，自动更新 pitch_deg 等字段。"""
        for b in data:
            self._process(b)

    def _process(self, b: int):
        s = self._state
        if s == 0:
            if b == 0x55:
                self._state = 1
        elif s == 1:
            self._state = 2 if b == 0x55 else (1 if b == 0x55 else 0)
        elif s == 2:
            self._id  = b
            self._state = 3
        elif s == 3:
            self._len  = b
            self._didx = 0
            self._state = 4 if b > 0 else 5
        elif s == 4:
            self._data[self._didx] = b
            self._didx += 1
            if self._didx >= self._len:
                self._state = 5
        elif s == 5:
            # 校验：sum(0x55,0x55,id,len,data...) & 0xFF
            chk = (0x55 + 0x55 + self._id + self._len) & 0xFF
            for i in range(self._len):
                chk = (chk + self._data[i]) & 0xFF
            if chk == b:
                self._dispatch()
                self.good_frames += 1
            else:
                self.bad_frames += 1
            self._state = 0

    def _dispatch(self):
        d = self._data
        if self._id == 0x01 and self._len == 6:
            r, p, y = struct.unpack_from('<hhh', d, 0)
            k = 180.0 / 32768.0
            self.roll_deg  = r * k
            self.pitch_deg = p * k
            self.yaw_deg   = y * k
            self.has_attitude = True
        elif self._id == 0x02 and self._len == 8:
            q0, q1, q2, q3 = struct.unpack_from('<hhhh', d, 0)
            k = 1.0 / 32768.0
            self.q0, self.q1 = q0 * k, q1 * k
            self.q2, self.q3 = q2 * k, q3 * k
        elif self._id == 0x03 and self._len == 12:
            ax, ay, az, gx, gy, gz = struct.unpack_from('<hhhhhh', d, 0)
            ka = self.ACCEL_FSR_G  / 32768.0
            kg = self.GYRO_FSR_DPS / 32768.0
            self.ax_g, self.ay_g, self.az_g = ax*ka, ay*ka, az*ka
            self.gx_dps, self.gy_dps, self.gz_dps = gx*kg, gy*kg, gz*kg
            self.has_gyro_acc = True
```

---

## 3. MCU 命令帧协议

### 3.1 帧格式

```
[0xAA][0x55][LEN][CMD][PAYLOAD × LEN][CRC16_LO][CRC16_HI][0x55][0xAA]
```

- **CRC16-CCITT**：多项式 `0x1021`，初始值 `0xFFFF`
- 校验范围：`LEN + CMD + PAYLOAD`（不含帧头尾）
- 最大 PAYLOAD：32 字节

### 3.2 CRC16-CCITT 实现

```python
def crc16_ccitt(data: bytes | bytearray) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
        crc &= 0xFFFF
    return crc
```

### 3.3 帧编码（K230 → MCU）

```python
def encode_frame(cmd: int, payload: bytes | bytearray = b'') -> bytes:
    """构造一帧完整数据（含帧头、CRC、帧尾）。"""
    length = len(payload)
    assert length <= 32, "payload too long"
    crc_data = bytes([length, cmd]) + bytes(payload)
    crc = crc16_ccitt(crc_data)
    return (bytes([0xAA, 0x55, length, cmd])
            + bytes(payload)
            + bytes([crc & 0xFF, crc >> 8])
            + bytes([0x55, 0xAA]))
```

### 3.4 帧解析器（MCU → K230）

```python
class MCUFrameParser:
    """逐字节解析 MCU → K230 帧。"""

    def __init__(self):
        self._state   = 0
        self._len     = 0
        self._cmd     = 0
        self._payload = bytearray(32)
        self._pidx    = 0
        self._crc_rx  = 0
        self.good = 0
        self.bad  = 0
        # 最近一帧
        self.last_cmd     = 0
        self.last_payload = bytes()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        """喂入字节流，返回本次解析出的所有完整帧列表 [(cmd, payload), ...]。"""
        frames = []
        for b in data:
            result = self._step(b)
            if result is not None:
                frames.append(result)
        return frames

    def _step(self, b: int):
        s = self._state
        if s == 0:
            if b == 0xAA: self._state = 1
        elif s == 1:
            self._state = 2 if b == 0x55 else (1 if b == 0xAA else 0)
        elif s == 2:
            if b > 32:
                self.bad += 1; self._state = 0
            else:
                self._len = b; self._state = 3
        elif s == 3:
            self._cmd = b
            self._pidx = 0
            self._state = 4 if self._len > 0 else 5
        elif s == 4:
            self._payload[self._pidx] = b
            self._pidx += 1
            if self._pidx >= self._len: self._state = 5
        elif s == 5:
            self._crc_rx = b; self._state = 6
        elif s == 6:
            self._crc_rx |= b << 8; self._state = 7
        elif s == 7:
            self._state = 8 if b == 0x55 else 0
        elif s == 8:
            self._state = 0
            if b != 0xAA:
                self.bad += 1; return None
            crc_data = bytes([self._len, self._cmd]) + bytes(self._payload[:self._len])
            if crc16_ccitt(crc_data) != self._crc_rx:
                self.bad += 1; return None
            self.good += 1
            self.last_cmd     = self._cmd
            self.last_payload = bytes(self._payload[:self._len])
            return (self._cmd, self.last_payload)
        return None
```

---

## 4. 业务帧定义

### 4.1 MCU → K230 帧（接收）

#### VEHICLE_STATUS (CMD=0x01, 20 Hz)

```python
import struct

def parse_vehicle_status(payload: bytes):
    """
    returns: (avg_cps: int, safety_state: int, bat_mv: int)
      avg_cps      -- 左右轮平均速度 counts/s（有符号，正=前进）
      safety_state -- 0=DISARMED, 1=ARMED, 2=BAT_WARN, 3=FALLEN, 4=BAT_STOP
      bat_mv       -- 电池电压 mV
    """
    avg_cps, safety_state, bat_mv = struct.unpack_from('<iBH', payload, 0)
    return avg_cps, safety_state, bat_mv
```

#### HEARTBEAT_MCU (CMD=0x02, 1 Hz)

```python
def parse_heartbeat_mcu(payload: bytes) -> int:
    """returns: uptime_ms"""
    return struct.unpack_from('<I', payload, 0)[0]
```

### 4.2 K230 → MCU 帧（发送）

#### MOTION_CMD (CMD=0x11, 20~50 Hz)

```python
def make_motion_cmd(target_v: int, target_omega: int, mode: int = 1) -> bytes:
    """
    target_v     -- 纵向速度（counts/s 已除以 SCALE=10，正=前进）
                    例：target_v=500 ≡ 5000 raw cps ≡ ~0.15 rev/s
    target_omega -- 转向差分量 permille（正=顺时针俯视）
    mode         -- 0=停止, 1=正常行驶
    """
    payload = struct.pack('<hhB', target_v, target_omega, mode)
    return encode_frame(0x11, payload)
```

#### HEARTBEAT_K230 (CMD=0x12, 1 Hz)

```python
def make_heartbeat_k230(uptime_ms: int) -> bytes:
    return encode_frame(0x12, struct.pack('<I', uptime_ms))
```

#### PID_INJECT (CMD=0x13, 按需)

```python
def make_pid_inject(pid_id: int, kp: float, ki: float, kd: float) -> bytes:
    """
    pid_id: 0=angle环, 1=rate环, 2=speed环, 3=yaw环
    """
    payload = struct.pack('<Bfff', pid_id, kp, ki, kd)
    return encode_frame(0x13, payload)
```

---

## 5. 完整集成示例

以下是一个完整的 K230 主控脚本骨架，演示两路 UART 并发工作的典型结构：

```python
"""
K230 平衡车控制端 —— Stage 4 通讯集成骨架
运行于 CanMV K230 MicroPython 环境
"""

import time
import struct
from machine import UART

# ---- 硬件初始化 ----
uart_imu = UART(1, baudrate=115200, bits=8, parity=None, stop=1)   # MS901M Y分线
uart_mcu = UART(2, baudrate=921600, bits=8, parity=None, stop=1)   # MCU UART1

imu = MS901MParser()
mcu_parser = MCUFrameParser()

# ---- 状态变量 ----
vehicle_avg_cps   = 0
vehicle_safety    = 0
vehicle_bat_mv    = 0
mcu_last_hb_ms    = time.ticks_ms()
MCU_TIMEOUT_MS    = 1000   # MCU 心跳超时阈值（K230 侧比 MCU 侧宽松）

uptime_ms = 0

def is_mcu_online() -> bool:
    return time.ticks_diff(time.ticks_ms(), mcu_last_hb_ms) < MCU_TIMEOUT_MS

# ---- 运动决策（占位，Stage 5 由循迹/视觉算法填充） ----
def compute_motion() -> tuple[int, int]:
    """
    返回 (target_v, target_omega)：
      target_v     整数，单位同 VEHICLE_STATUS avg_cps / SCALE
      target_omega permille，正=顺时针
    如果 MCU 离线则返回 (0, 0)
    """
    if not is_mcu_online():
        return (0, 0)
    # TODO: Stage 5 填入循迹、光斑算法
    return (0, 0)

# ---- 主循环 ----
last_cmd_ms  = time.ticks_ms()
last_hb_ms   = time.ticks_ms()
CMD_PERIOD   = 25    # 40 Hz 发送运动指令
HB_PERIOD    = 1000  # 1 Hz 心跳

while True:
    now = time.ticks_ms()
    uptime_ms = now

    # 1. drain IMU UART
    imu_data = uart_imu.read(64)
    if imu_data:
        imu.feed(imu_data)

    # 2. drain MCU UART → 解析帧
    mcu_data = uart_mcu.read(128)
    if mcu_data:
        frames = mcu_parser.feed(mcu_data)
        for cmd, payload in frames:
            if cmd == 0x01 and len(payload) == 7:   # VEHICLE_STATUS
                vehicle_avg_cps, vehicle_safety, vehicle_bat_mv = parse_vehicle_status(payload)
            elif cmd == 0x02:                        # HEARTBEAT_MCU
                mcu_last_hb_ms = now

    # 3. 20~50 Hz：发送运动指令
    if time.ticks_diff(now, last_cmd_ms) >= CMD_PERIOD:
        last_cmd_ms = now
        v, omega = compute_motion()
        uart_mcu.write(make_motion_cmd(v, omega, mode=1 if is_mcu_online() else 0))

    # 4. 1 Hz：心跳 + 状态打印
    if time.ticks_diff(now, last_hb_ms) >= HB_PERIOD:
        last_hb_ms = now
        uart_mcu.write(make_heartbeat_k230(uptime_ms))
        print(f"[hb] pitch={imu.pitch_deg:.2f} gy={imu.gy_dps:.1f} "
              f"avg_cps={vehicle_avg_cps} bat={vehicle_bat_mv}mV "
              f"mcu={'ON' if is_mcu_online() else 'OFF'} "
              f"imu_g={imu.good_frames}/b={imu.bad_frames} "
              f"mcu_g={mcu_parser.good}/b={mcu_parser.bad}")

    # 5. 云台控制（Stage 5 实现，此处占位）
    if imu.has_attitude:
        pitch_for_gimbal = imu.pitch_deg   # 直接用，无需 MCU 转发
        # gimbal.set_pitch_feedforward(-pitch_for_gimbal)
        pass

    time.sleep_ms(1)
```

---

## 6. 安全规则

| 规则 | 说明 |
|------|------|
| MCU 离线时发 `(v=0, omega=0, mode=0)` | MCU 侧有 500 ms 超时归零，但 K230 侧也应主动归零避免存留指令 |
| 不得发 `mode=1` 但 `safety_state >= 3` | FALLEN / BAT_STOP 时 MCU 会拒绝运动指令，K230 应等 `safety_state` 恢复后再发 |
| `bat_mv < 9500` 时减速 | 对应 MCU 的 LOW_BAT_WARN 提前量，减少 MCU 紧急停车风险 |
| 心跳必须每秒发送 | MCU 500 ms 无帧即归零，K230 侧心跳间隔建议 ≤ 400 ms |

---

## 7. 调试方法

### 7.0 K230 运行日志判读

K230 主循环每 5 s 会打印 MCU 链路统计。若出现类似：

```text
mcu=ON/OFF ... mcu_g=15/b=617 uart_rx=9390B
```

先看 `uart_rx`。若 30 s 约 9.3 KB，说明 UART2 实际收到了 MCU 上行流：

- `VEHICLE_STATUS`：20 Hz × 15 B = 300 B/s；
- `HEARTBEAT_MCU`：1 Hz × 12 B = 12 B/s；
- 合计约 312 B/s。

这时问题通常不是接线/波特率完全错误，而是帧解析不一致。K230 会继续打印：

```text
mcu_bad=lenX/t1Y/t2Z/crcW last=reason:Lxx:Cyy:rx/calc
```

判读规则：

- `crc` 高：核对 MCU 侧 CRC16-CCITT 参数、校验范围是否为 `LEN + CMD + PAYLOAD`，以及 CRC 是否低字节在前；
- `tail1/tail2` 高：核对帧尾是否严格为 `55 AA`，并检查是否多发/少发字节；
- `len` 高：核对 `LEN` 是否只表示 payload 长度，不包含 CMD/CRC/帧尾；
- `last` 中 `C01` 长期出现而 heartbeat 偶尔通过：重点检查 `VEHICLE_STATUS` 构帧。

### 7.1 单步验证 IMU 链路

```python
uart_imu = UART(1, baudrate=115200, bits=8, parity=None, stop=1)
imu = MS901MParser()
while True:
    d = uart_imu.read(64)
    if d:
        imu.feed(d)
        if imu.has_attitude:
            print(f"pitch={imu.pitch_deg:.2f} roll={imu.roll_deg:.2f}")
    time.sleep_ms(5)
```

正常情况下 pitch 以 200 Hz 刷新；若无数据，检查 MS901M TX → K230 RX 分线焊点和共地。

### 7.2 单步验证 MCU 命令链路（无 K230 侧主逻辑）

在 PC 上用任意串口工具连接 MCU PB6（921600 8N1），应能看到：
- 每 50 ms 一帧 `AA 55 07 01 ...` (VEHICLE_STATUS)
- 每 1000 ms 一帧 `AA 55 04 02 ...` (HEARTBEAT_MCU)

发送以下字节触发 MOTION_CMD（v=100, omega=0, mode=1）：

```python
# 在 PC Python 端生成测试帧
payload = struct.pack('<hhB', 100, 0, 1)
frame = encode_frame(0x11, payload)
print(frame.hex())
```

收到后 MCU 日志会显示 `k230_ON`，车辆以 v=100 (×10 = 1000 raw cps ≈ 低速前行)  运动。

### 7.3 回环自测（不接 MCU）

将 K230 命令 UART 的 TX 短接 RX，发送任意帧后解析器应立即收到同一帧，`good` 计数递增。

---

## 8. 变更日志

| 日期 | 版本 | 内容 |
|------|------|------|
| 2026-05-17 | v0.1 | 初版，完整描述 K230 端 IMU 解析、命令帧协议、集成示例与调试方法 |
