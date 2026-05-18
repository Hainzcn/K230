"""ATK-MS901M 串口姿态传感器流式二进制协议解析。

帧结构：
    0x55 0x55  ID  LEN  DATA[LEN]  CHECKSUM

校验：CHECKSUM = (0x55 + 0x55 + ID + LEN + DATA[0] + ... + DATA[LEN-1]) & 0xFF

本工程解析三个推送帧：
    ID 0x01  姿态   LEN=6   roll/pitch/yaw  (int16 LE / 32768 × 180°)
    ID 0x02  四元数 LEN=8   q0/q1/q2/q3    (int16 LE / 32768)
    ID 0x03  gyro+acc LEN=12  ax/ay/az/gx/gy/gz  (int16 LE，量纲见下)

量纲（出厂默认 ±4 g / ±2000 dps，MCU 侧 Flash 写入，不得更改）：
    加速度：val / 32768 × 4  → g
    陀螺：  val / 32768 × 2000 → °/s

MS901M 以 115200 8N1 主动推送，K230 通过 UART(1) 仅使用 RX 接收
MS901M TX 线 Y 分出来的支路（另一路接 MCU UART3 RX）。
"""

import struct as _struct


class MS901MParser:
    """ATK-MS901M 二进制帧逐字节解析器。

    典型用法::

        parser = MS901MParser()
        data = uart.read(64)
        if data:
            parser.feed(data)
        pitch = parser.pitch_deg
    """

    GYRO_FSR_DPS  = 2000.0
    ACCEL_FSR_G   = 4.0
    _EXPECTED_LEN = {
        0x01: 6,
        0x02: 8,
        0x03: 12,
    }

    # 状态机状态编号
    _S_SYNC1     = 0
    _S_SYNC2     = 1
    _S_ID        = 2
    _S_LEN       = 3
    _S_DATA      = 4
    _S_CHECKSUM  = 5

    def __init__(self):
        self._state  = self._S_SYNC1
        self._id     = 0
        self._len    = 0
        self._data   = bytearray(16)
        self._didx   = 0

        # 姿态（ID 0x01）
        self.roll_deg  = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg   = 0.0
        self.has_attitude = False

        # 四元数（ID 0x02）
        self.q0 = 1.0
        self.q1 = self.q2 = self.q3 = 0.0

        # 陀螺 + 加速（ID 0x03）
        self.ax_g   = self.ay_g   = self.az_g   = 0.0
        self.gx_dps = self.gy_dps = self.gz_dps = 0.0
        self.has_gyro_acc = False

        # 统计
        self.good_frames = 0
        self.bad_frames  = 0

    # ------------------------------------------------------------------

    def feed(self, data):
        """喂入任意长度字节序列，内部逐字节更新解析状态。"""
        for b in data:
            self._process(b)

    def reset(self):
        """重置状态机（不清数据字段）。"""
        self._state = self._S_SYNC1

    # ------------------------------------------------------------------

    def _resync_after_bad_byte(self, b):
        """丢弃坏帧后尽量保留当前 0x55 作为下一帧同步头。"""
        self.bad_frames += 1
        self._state = self._S_SYNC2 if b == 0x55 else self._S_SYNC1

    def _process(self, b):
        s = self._state
        if s == self._S_SYNC1:
            if b == 0x55:
                self._state = self._S_SYNC2
        elif s == self._S_SYNC2:
            if b == 0x55:
                self._state = self._S_ID
            elif b != 0x55:
                # 非 0x55 丢弃，但当前字节本身不能是 sync1 的一部分
                self._state = self._S_SYNC1
        elif s == self._S_ID:
            if b not in self._EXPECTED_LEN:
                self._resync_after_bad_byte(b)
                return
            self._id    = b
            self._state = self._S_LEN
        elif s == self._S_LEN:
            expected_len = self._EXPECTED_LEN.get(self._id)
            if b != expected_len or b > len(self._data):
                self._resync_after_bad_byte(b)
                return
            self._len  = b
            self._didx = 0
            self._state = self._S_DATA if b > 0 else self._S_CHECKSUM
        elif s == self._S_DATA:
            self._data[self._didx] = b
            self._didx += 1
            if self._didx >= self._len:
                self._state = self._S_CHECKSUM
        elif s == self._S_CHECKSUM:
            chk = (0x55 + 0x55 + self._id + self._len) & 0xFF
            for i in range(self._len):
                chk = (chk + self._data[i]) & 0xFF
            if chk == b:
                self._dispatch()
                self.good_frames += 1
            else:
                self._resync_after_bad_byte(b)
                return
            self._state = self._S_SYNC1

    def _dispatch(self):
        d   = self._data
        fid = self._id
        n   = self._len

        if fid == 0x01 and n == 6:
            r, p, y = _struct.unpack_from('<hhh', d, 0)
            k = 180.0 / 32768.0
            self.roll_deg  = r * k
            self.pitch_deg = p * k
            self.yaw_deg   = y * k
            self.has_attitude = True

        elif fid == 0x02 and n == 8:
            q0, q1, q2, q3 = _struct.unpack_from('<hhhh', d, 0)
            k = 1.0 / 32768.0
            self.q0, self.q1 = q0 * k, q1 * k
            self.q2, self.q3 = q2 * k, q3 * k

        elif fid == 0x03 and n == 12:
            ax, ay, az, gx, gy, gz = _struct.unpack_from('<hhhhhh', d, 0)
            ka = self.ACCEL_FSR_G  / 32768.0
            kg = self.GYRO_FSR_DPS / 32768.0
            self.ax_g,   self.ay_g,   self.az_g   = ax * ka, ay * ka, az * ka
            self.gx_dps, self.gy_dps, self.gz_dps = gx * kg, gy * kg, gz * kg
            self.has_gyro_acc = True

    # ------------------------------------------------------------------

    @staticmethod
    def self_test():
        """基本功能验证，返回 True 表示通过。在启动期调用。"""
        p = MS901MParser()
        # 构造一帧 ID=0x01 LEN=6：roll=0 pitch=16384(≈90°) yaw=0
        import struct
        data = bytearray([0, 0])               # roll=0
        data += struct.pack('<h', 16384)        # pitch ≈ 90°
        data += bytearray([0, 0])              # yaw=0
        chk  = (0x55 + 0x55 + 0x01 + 0x06) & 0xFF
        for b in data:
            chk = (chk + b) & 0xFF
        frame = bytes([0x55, 0x55, 0x01, 0x06]) + bytes(data) + bytes([chk])
        p.feed(frame)
        # 非法 LEN 必须丢弃并恢复同步，不能写爆内部 bytearray。
        p.feed(bytes([0x55, 0x55, 0x01, 0x40, 0x55, 0x55, 0x01, 0x06])
               + bytes(data) + bytes([chk]))
        ok = (
            p.good_frames == 2
            and p.bad_frames == 1
            and p.has_attitude
            and abs(p.pitch_deg - 90.0) < 0.01
            and p.roll_deg == 0.0
        )
        if not ok:
            print("[ms901m] self_test FAILED good=%d pitch=%.3f"
                  % (p.good_frames, p.pitch_deg))
        return ok
