"""K230 双路 UART 高层封装。

ImuLink  — UART(1, 115200)，接 MS901M TX Y 分线，仅 RX，200 Hz 原始姿态数据。
McuLink  — UART(2, 921600)，双向，与 MSPM0G3507 MCU UART1 互联。

在 bench（PC / CanMV IDE）环境下 machine 模块不存在时，两个 Link 以 stub
模式工作（init 不报错，drain/send 均为空操作），便于主循环在桌面调试。
"""

import time

from comms.ms901m import MS901MParser
from comms.frame  import MCUFrameParser
from comms.protocol import (
    parse_vehicle_status,
    parse_heartbeat_mcu,
    make_motion_cmd,
    make_heartbeat_k230,
    SAFETY_DISARMED,
    SAFETY_FALLEN,
    CMD_VEHICLE_STATUS,
    CMD_HEARTBEAT_MCU,
)

# ---------------------------------------------------------------------------
# UART 懒加载（兼容 bench 环境）
# ---------------------------------------------------------------------------

def _try_import_uart():
    """尝试导入 machine.UART；bench 环境返回 None。"""
    try:
        from machine import UART
        return UART
    except ImportError:
        return None


_UART_CLASS = _try_import_uart()


def _open_uart(uart_id, baudrate):
    """创建并返回 UART 实例；machine 不可用时返回 None。"""
    if _UART_CLASS is None:
        return None
    try:
        return _UART_CLASS(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1)
    except Exception as e:
        print("[uart_link] open UART(%d, %d) failed: %s" % (uart_id, baudrate, e))
        return None


# ---------------------------------------------------------------------------
# ImuLink：MS901M 直通 UART
# ---------------------------------------------------------------------------

class ImuLink:
    """封装 UART(1, 115200) + MS901MParser，提供 drain() 与 snapshot()。

    bench 模式（uart=None）下 drain() 是空操作，snapshot() 返回 None。
    """

    BAUD = 115200
    READ_BYTES = 64      # 200 Hz 时每 5 ms 约 ~72 B；每帧读 64 B 足够

    def __init__(self, uart_id=1):
        self._parser = MS901MParser()
        self._uart   = _open_uart(uart_id, self.BAUD)
        if self._uart is None:
            print("[imu_link] bench mode (no UART)")
        else:
            print("[imu_link] UART(%d, %d) opened" % (uart_id, self.BAUD))

    def drain(self):
        """读取 UART RX 缓冲并喂给解析器。主循环每帧调用一次。"""
        if self._uart is None:
            return
        data = self._uart.read(self.READ_BYTES)
        if data:
            self._parser.feed(data)

    @property
    def parser(self):
        """返回内部 MS901MParser，可直接访问 pitch_deg 等字段。"""
        return self._parser

    def pitch_deg(self):
        return self._parser.pitch_deg

    def is_ready(self):
        """0x01 帧至少收到一次才算 IMU 就绪。"""
        return self._parser.has_attitude

    def stats(self):
        """返回 (good_frames, bad_frames)。"""
        return self._parser.good_frames, self._parser.bad_frames


# ---------------------------------------------------------------------------
# McuLink：MCU 命令链路
# ---------------------------------------------------------------------------

class McuLink:
    """封装 UART(2, 921600) + MCUFrameParser，管理双向通讯与心跳超时。

    状态：
        vehicle_avg_cps  — 最新 VEHICLE_STATUS 中的平均速度（counts/s）
        vehicle_safety   — 最新安全状态（SAFETY_* 常量）
        vehicle_bat_mv   — 最新电池电压（mV）

    bench 模式（uart=None）下 drain/send 均为空操作。
    """

    BAUD       = 921600
    READ_BYTES = 128

    def __init__(self, uart_id=2, timeout_ms=500):
        self._parser   = MCUFrameParser()
        self._uart     = _open_uart(uart_id, self.BAUD)
        self._timeout_ms = timeout_ms
        self._last_hb_ms = time.ticks_ms()

        self.vehicle_avg_cps = 0
        self.vehicle_safety  = SAFETY_DISARMED
        self.vehicle_bat_mv  = 0

        if self._uart is None:
            print("[mcu_link] bench mode (no UART)")
        else:
            print("[mcu_link] UART(%d, %d) opened" % (uart_id, self.BAUD))

    # ------------------------------------------------------------------
    # 接收
    # ------------------------------------------------------------------

    def drain(self, now_ms=None):
        """读取 RX 缓冲，解析帧，更新内部状态。

        Args:
            now_ms: 当前 ticks_ms（传入可减少 ticks_ms 调用次数）
        """
        if self._uart is None:
            return
        if now_ms is None:
            now_ms = time.ticks_ms()

        data = self._uart.read(self.READ_BYTES)
        if not data:
            return

        for cmd, payload in self._parser.feed(data):
            if cmd == CMD_VEHICLE_STATUS and len(payload) >= 7:
                self.vehicle_avg_cps, self.vehicle_safety, self.vehicle_bat_mv = (
                    parse_vehicle_status(payload)
                )
            elif cmd == CMD_HEARTBEAT_MCU:
                self._last_hb_ms = now_ms

    # ------------------------------------------------------------------
    # 状态查询
    # ------------------------------------------------------------------

    def is_online(self, now_ms=None):
        """MCU 在线：最近一次心跳在 timeout_ms 之内。"""
        if now_ms is None:
            now_ms = time.ticks_ms()
        return time.ticks_diff(now_ms, self._last_hb_ms) < self._timeout_ms

    def is_safe_to_drive(self, now_ms=None):
        """可以发运动指令：MCU 在线 且 safety_state < FALLEN。"""
        return self.is_online(now_ms) and self.vehicle_safety < SAFETY_FALLEN

    def stats(self):
        """返回 (good_frames, bad_frames)。"""
        return self._parser.good, self._parser.bad

    # ------------------------------------------------------------------
    # 发送
    # ------------------------------------------------------------------

    def send_motion(self, target_v, target_omega, mode=None):
        """发送 MOTION_CMD。

        若 mode 未指定，则根据 is_online 自动选择：
            在线且安全 → mode=1（行驶）
            否则       → mode=0（停止）
        """
        if self._uart is None:
            return
        if mode is None:
            mode = 1 if self.is_safe_to_drive() else 0
        frame = make_motion_cmd(int(target_v), int(target_omega), mode)
        self._uart.write(frame)

    def send_heartbeat(self, uptime_ms):
        """发送 HEARTBEAT_K230。"""
        if self._uart is None:
            return
        self._uart.write(make_heartbeat_k230(uptime_ms))

    def send_stop(self):
        """发送 mode=0 紧急停止指令（不依赖在线状态）。"""
        if self._uart is None:
            return
        self._uart.write(make_motion_cmd(0, 0, 0))

    # ------------------------------------------------------------------
    # 自检
    # ------------------------------------------------------------------

    @staticmethod
    def self_test():
        """协议层 loopback 自检（不需要真实 UART）。"""
        from comms.frame import MCUFrameParser, encode_frame
        import struct

        frame  = make_motion_cmd(200, -100, 1)
        parser = MCUFrameParser()
        frames = parser.feed(frame)
        if not frames or frames[0][0] != 0x11:
            print("[mcu_link] self_test FAILED")
            return False
        v, omega, mode = struct.unpack_from('<hhB', frames[0][1], 0)
        ok = (v == 200 and omega == -100 and mode == 1)
        if not ok:
            print("[mcu_link] self_test FAILED fields v=%d omega=%d mode=%d"
                  % (v, omega, mode))
        return ok
