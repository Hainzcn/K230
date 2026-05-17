"""MCU ↔ K230 命令帧编解码与 CRC16-CCITT。

帧格式：
    [0xAA][0x55][LEN][CMD][PAYLOAD × LEN][CRC16_LO][CRC16_HI][0x55][0xAA]

CRC16-CCITT：多项式 0x1021，初始值 0xFFFF
校验范围：LEN + CMD + PAYLOAD（不含帧头尾）
最大 PAYLOAD：32 字节
"""

_PAYLOAD_MAX = 32
_FRAME_HEAD  = bytes([0xAA, 0x55])
_FRAME_TAIL  = bytes([0x55, 0xAA])


# ---------------------------------------------------------------------------
# CRC16-CCITT
# ---------------------------------------------------------------------------

def crc16_ccitt(data):
    """计算 CRC16-CCITT（多项式 0x1021，初始值 0xFFFF）。

    Args:
        data: bytes 或 bytearray

    Returns:
        int，16 位 CRC 值
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
        crc &= 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# 帧编码（K230 → MCU）
# ---------------------------------------------------------------------------

def encode_frame(cmd, payload=b''):
    """构造一帧完整数据（含帧头、CRC、帧尾）。

    Args:
        cmd:     int，命令字节
        payload: bytes 或 bytearray，≤ 32 字节

    Returns:
        bytes，完整帧
    """
    length = len(payload)
    if length > _PAYLOAD_MAX:
        raise ValueError("payload too long: %d > %d" % (length, _PAYLOAD_MAX))
    crc_data = bytes([length, cmd]) + bytes(payload)
    crc = crc16_ccitt(crc_data)
    return (
        _FRAME_HEAD
        + bytes([length, cmd])
        + bytes(payload)
        + bytes([crc & 0xFF, crc >> 8])
        + _FRAME_TAIL
    )


# ---------------------------------------------------------------------------
# 帧解析器（MCU → K230）
# ---------------------------------------------------------------------------

class MCUFrameParser:
    """逐字节解析 MCU→K230 帧，支持连续字节流。

    典型用法::

        parser = MCUFrameParser()
        data = uart.read(128)
        if data:
            for cmd, payload in parser.feed(data):
                handle(cmd, payload)
    """

    # 状态机状态
    _S_HEAD1   = 0
    _S_HEAD2   = 1
    _S_LEN     = 2
    _S_CMD     = 3
    _S_PAYLOAD = 4
    _S_CRC_LO  = 5
    _S_CRC_HI  = 6
    _S_TAIL1   = 7
    _S_TAIL2   = 8

    def __init__(self):
        self._state   = self._S_HEAD1
        self._len     = 0
        self._cmd     = 0
        self._payload = bytearray(_PAYLOAD_MAX)
        self._pidx    = 0
        self._crc_rx  = 0
        # 统计
        self.good = 0
        self.bad  = 0
        # 最近一帧（调试用）
        self.last_cmd     = 0
        self.last_payload = bytes()

    def feed(self, data):
        """喂入字节流，返回本次解析出的完整帧列表。

        Args:
            data: bytes 或 bytearray

        Returns:
            list of (cmd: int, payload: bytes)
        """
        frames = []
        for b in data:
            result = self._step(b)
            if result is not None:
                frames.append(result)
        return frames

    def reset(self):
        """重置状态机。"""
        self._state = self._S_HEAD1

    def _step(self, b):
        s = self._state
        if s == self._S_HEAD1:
            if b == 0xAA:
                self._state = self._S_HEAD2
        elif s == self._S_HEAD2:
            if b == 0x55:
                self._state = self._S_LEN
            elif b == 0xAA:
                pass   # 连续 0xAA，保持等待第二字节
            else:
                self._state = self._S_HEAD1
        elif s == self._S_LEN:
            if b > _PAYLOAD_MAX:
                self.bad += 1
                self._state = self._S_HEAD1
            else:
                self._len  = b
                self._state = self._S_CMD
        elif s == self._S_CMD:
            self._cmd  = b
            self._pidx = 0
            self._state = self._S_PAYLOAD if self._len > 0 else self._S_CRC_LO
        elif s == self._S_PAYLOAD:
            self._payload[self._pidx] = b
            self._pidx += 1
            if self._pidx >= self._len:
                self._state = self._S_CRC_LO
        elif s == self._S_CRC_LO:
            self._crc_rx  = b
            self._state   = self._S_CRC_HI
        elif s == self._S_CRC_HI:
            self._crc_rx |= b << 8
            self._state   = self._S_TAIL1
        elif s == self._S_TAIL1:
            self._state = self._S_TAIL2 if b == 0x55 else self._S_HEAD1
            if b != 0x55:
                self.bad += 1
        elif s == self._S_TAIL2:
            self._state = self._S_HEAD1
            if b != 0xAA:
                self.bad += 1
                return None
            crc_data = bytes([self._len, self._cmd]) + bytes(
                self._payload[: self._len]
            )
            if crc16_ccitt(crc_data) != self._crc_rx:
                self.bad += 1
                return None
            self.good += 1
            self.last_cmd     = self._cmd
            self.last_payload = bytes(self._payload[: self._len])
            return (self._cmd, self.last_payload)
        return None

    # ------------------------------------------------------------------

    @staticmethod
    def self_test():
        """往返编解码自检，返回 True 表示通过。"""
        p = MCUFrameParser()
        # 构造 VEHICLE_STATUS 帧：avg_cps=1000, safety=1, bat=11100
        import struct
        payload = struct.pack('<iBH', 1000, 1, 11100)
        frame   = encode_frame(0x01, payload)
        frames  = p.feed(frame)
        ok = (
            len(frames) == 1
            and frames[0][0] == 0x01
            and frames[0][1] == payload
            and p.good == 1
            and p.bad  == 0
        )
        if not ok:
            print("[frame] self_test FAILED frames=%s good=%d bad=%d"
                  % (frames, p.good, p.bad))
        return ok
