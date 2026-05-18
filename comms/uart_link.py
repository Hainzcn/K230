"""K230 双路 UART 高层封装。

ImuLink  — UART(1, 115200)，接 MS901M TX Y 分线，仅 RX，200 Hz 原始姿态数据。
McuLink  — UART(2, 921600)，双向，与 MSPM0G3507 MCU UART1 互联。

在 bench（PC / CanMV IDE）环境下 machine 模块不存在时，两个 Link 以 stub
模式工作（init 不报错，drain/send 均为空操作），便于主循环在桌面调试。
"""

import time

# MicroPython 兼容：CPython 无 ticks_ms / ticks_diff，用 time.monotonic_ns 模拟
if not hasattr(time, "ticks_ms"):
    def _ticks_ms():
        return int(time.monotonic_ns() // 1_000_000)
    def _ticks_us():
        return int(time.monotonic_ns() // 1_000)
    def _ticks_diff(new, old):
        return new - old
    time.ticks_ms   = _ticks_ms
    time.ticks_us   = _ticks_us
    time.ticks_diff = _ticks_diff

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
# FPIOA + UART 懒加载（兼容 bench 环境）
# ---------------------------------------------------------------------------
# K230 可用 UART：UART1 / UART2 / UART4
# （UART0=小核SH占用，UART3=大核SH占用，切勿使用）
# UART 引脚通过 FPIOA.set_function() 分配；LP 板出厂已将 IO_5/IO_6 默认
# 映射到 UART2_TXD/RXD，但显式设置更安全，且 UART1 必须显式配置。

def _try_import_machine():
    """尝试导入 machine 模块；bench 环境返回 (None, None)。"""
    try:
        from machine import UART, FPIOA
        return UART, FPIOA
    except ImportError:
        return None, None


_UART_CLASS, _FPIOA_CLASS = _try_import_machine()

# UART 通道号常量映射（UART.UART1 / UART.UART2 等值在 K230 MicroPython 中
# 等于整数 1 / 2，但通过属性取值更明确，且 bench 下退回整数 fallback）
_UART_ID_MAP = {}
if _UART_CLASS is not None:
    _UART_ID_MAP = {
        1: getattr(_UART_CLASS, "UART1", 1),
        2: getattr(_UART_CLASS, "UART2", 2),
        4: getattr(_UART_CLASS, "UART4", 4),
    }


def _fpioa_setup_uart2(tx_io=5, rx_io=6):
    """配置 UART2 TX/RX FPIOA 映射（MCU 命令链路）。

    LP 板出厂：IO_5=UART2_TXD (Header NO.17)，IO_6=UART2_RXD (Header NO.20)。
    """
    if _FPIOA_CLASS is None:
        return
    try:
        fpioa = _FPIOA_CLASS()
        fpioa.set_function(tx_io, getattr(_FPIOA_CLASS, "UART2_TXD"))
        fpioa.set_function(rx_io, getattr(_FPIOA_CLASS, "UART2_RXD"))
    except Exception as e:
        print("[uart_link] FPIOA UART2 setup failed: %s" % e)


def _fpioa_setup_uart1(tx_io=3, rx_io=4):
    """配置 UART1 TX+RX FPIOA 映射（IMU 直通）。

    K230 CanMV UART 驱动要求 TX 和 RX 引脚同时配置，否则报
    'tx not configured'，即使只使用 RX 也不例外。

    LP 板可用引脚（来自 fpioa.help() 实测）：
      IO_3 (Header NO.18) → UART1_TXD  (实际不发送，仅满足驱动要求)
      IO_4 (Header NO.13) → UART1_RXD  ← MS901M TX Y 分线
    """
    if _FPIOA_CLASS is None:
        return
    try:
        fpioa = _FPIOA_CLASS()
        fpioa.set_function(tx_io, getattr(_FPIOA_CLASS, "UART1_TXD"))
        fpioa.set_function(rx_io, getattr(_FPIOA_CLASS, "UART1_RXD"))
    except Exception as e:
        print("[uart_link] FPIOA UART1 setup failed: %s" % e)


def _open_uart(uart_id, baudrate, timeout_ms=None):
    """创建并返回 UART 实例；machine 不可用时返回 None。

    timeout_ms 含义（K230 CanMV 实测）：
      - 默认（不传）：read(n) 阻塞到 n 字节到齐；read() 无参追加 ~30ms 惰性等待
      - 显式数值：read(n) 最多等待该时长（ms），超时后返回已读到的部分字节
      - 0：read(n) 在缓冲 <n 时立即返回 None（无数据时完全无效，勿用）

    uart.any() 在 K230 CanMV 上缓冲首次读空后始终返回 0，
    不能用于非阻塞轮询，必须依赖 read(n)+timeout 控制阻塞时长。
    """
    if _UART_CLASS is None:
        return None
    hw_id = _UART_ID_MAP.get(uart_id, uart_id)
    try:
        kwargs = dict(
            baudrate=baudrate,
            bits=_UART_CLASS.EIGHTBITS,
            parity=_UART_CLASS.PARITY_NONE,
            stop=_UART_CLASS.STOPBITS_ONE,
        )
        if timeout_ms is not None:
            kwargs["timeout"] = timeout_ms
        return _UART_CLASS(hw_id, **kwargs)
    except Exception as e:
        print("[uart_link] open UART(%d, %d) failed: %s" % (uart_id, baudrate, e))
        return None


def _init_perf_counters(obj):
    """初始化 UART drain 轻量统计字段。"""
    obj._rx_bytes = 0
    obj._last_read_len = 0
    obj._drain_calls = 0
    obj._drain_us_total = 0
    obj._drain_us_max = 0


def _record_drain_perf(obj, t0_us, data):
    """记录一次 read/drain 耗时；用于 5s 日志确认 UART 不拖慢主循环。"""
    dt_us = time.ticks_diff(time.ticks_us(), t0_us)
    n = len(data) if data else 0
    obj._rx_bytes += n
    obj._last_read_len = n
    obj._drain_calls += 1
    obj._drain_us_total += dt_us
    if dt_us > obj._drain_us_max:
        obj._drain_us_max = dt_us


def _perf_stats(obj):
    """返回 (rx_bytes, last_read_len, drain_calls, avg_us, max_us)。"""
    if obj._drain_calls <= 0:
        return (obj._rx_bytes, obj._last_read_len, 0, 0, 0)
    avg_us = obj._drain_us_total // obj._drain_calls
    return (
        obj._rx_bytes,
        obj._last_read_len,
        obj._drain_calls,
        avg_us,
        obj._drain_us_max,
    )


# ---------------------------------------------------------------------------
# ImuLink：MS901M 直通 UART
# ---------------------------------------------------------------------------

class ImuLink:
    """封装 UART(1, 115200) + MS901MParser，提供 drain() 与 snapshot()。

    bench 模式（uart=None）下 drain() 是空操作，snapshot() 返回 None。
    """

    BAUD      = 115200
    # MS901M：200Hz × 41B ≈ 8200B/s。K230 snapshot API 已确认约 30FPS，
    # 每主循环自然积压约 270B；读 256B 通常能立即返回，避免旧 512B/10ms
    # 组合在积压不足时每帧等满 timeout。
    _READ_N     = 256
    _TIMEOUT_MS = 3

    def __init__(self, uart_id=1, tx_io=3, rx_io=4):
        """
        Args:
            uart_id: UART 通道号（K230 可用：1 / 2 / 4）
            tx_io:   FPIOA TX 引脚号（默认 IO_3，Header NO.18；驱动要求配置，实际不发送）
            rx_io:   FPIOA RX 引脚号（默认 IO_4，Header NO.13）
        """
        self._parser = MS901MParser()
        _init_perf_counters(self)
        # FPIOA：TX+RX 必须同时配置，否则 UART 驱动拒绝打开
        if uart_id == 1:
            _fpioa_setup_uart1(tx_io, rx_io)
        self._uart = _open_uart(uart_id, self.BAUD, timeout_ms=self._TIMEOUT_MS)
        if self._uart is None:
            print("[imu_link] bench mode (no UART)")
        else:
            print("[imu_link] UART%d @%d TX=IO_%d RX=IO_%d timeout=%dms opened"
                  % (uart_id, self.BAUD, tx_io, rx_io, self._TIMEOUT_MS))

    def drain(self):
        """读取 UART RX 缓冲，喂给解析器。主循环每帧调用一次。

        使用 read(_READ_N)：
          - 30FPS 主循环下 IMU 每帧积压约 270B，_READ_N=256 通常立即返回；
          - 缓冲不足时最多等待 _TIMEOUT_MS=3ms，避免 UART 抢走 snapshot/算法预算。

        注：uart.any() 在 K230 CanMV 上缓冲首次读空后始终返回 0（驱动缺陷），
        不可用于非阻塞轮询。
        """
        if self._uart is None:
            return
        t0_us = time.ticks_us()
        data = self._uart.read(self._READ_N)
        _record_drain_perf(self, t0_us, data)
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

    def perf_stats(self):
        """返回 (rx_bytes, last_read_len, drain_calls, avg_us, max_us)。"""
        return _perf_stats(self)


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

    BAUD         = 921600
    _READ_N      = 64    # MCU 单帧 ≤20B；64B 足以覆盖数帧积压
    _TIMEOUT_MS  = 1     # 921600 下小帧传输亚毫秒级；无数据时最多让出 1ms

    def __init__(self, uart_id=2, timeout_ms=500, tx_io=5, rx_io=6):
        """
        Args:
            uart_id:    UART 通道号（K230 可用：1 / 2 / 4）
            timeout_ms: MCU 心跳超时阈值
            tx_io:      FPIOA TX 引脚号（默认 IO_5，Header NO.17 = UART2_TXD）
            rx_io:      FPIOA RX 引脚号（默认 IO_6，Header NO.20 = UART2_RXD）
        """
        self._parser     = MCUFrameParser()
        self._timeout_ms = timeout_ms
        # MCU 在线性按"最近一次合法 MCU 上行帧"判断，而不仅是 HEARTBEAT_MCU。
        # Stage4 MCU 侧也是"无任何 K230 帧 500ms"才离线；这里保持对称，
        # 避免心跳帧 CRC 联调异常时 VEHICLE_STATUS 正常却 MCU:ON/OFF 抖动。
        self._last_rx_ms = time.ticks_ms()
        _init_perf_counters(self)

        self.vehicle_avg_cps = 0
        self.vehicle_safety  = SAFETY_DISARMED
        self.vehicle_bat_mv  = 0

        # FPIOA：先配置引脚再打开 UART（LP 板默认已映射，显式设置更安全）
        if uart_id == 2:
            _fpioa_setup_uart2(tx_io, rx_io)

        self._uart = _open_uart(uart_id, self.BAUD, timeout_ms=self._TIMEOUT_MS)
        if self._uart is None:
            print("[mcu_link] bench mode (no UART)")
        else:
            print("[mcu_link] UART%d @%d TX=IO_%d RX=IO_%d timeout=%dms opened"
                  % (uart_id, self.BAUD, tx_io, rx_io, self._TIMEOUT_MS))

    # ------------------------------------------------------------------
    # 接收
    # ------------------------------------------------------------------

    def drain(self, now_ms=None):
        """非阻塞读取 RX 缓冲，解析帧，更新内部状态。

        K230 的 uart.any() 在首次读空后可能失效，因此沿用 read(n)+短 timeout。
        MCU 上行仅 20Hz 状态 + 1Hz 心跳，_TIMEOUT_MS=1 可把离线/空读成本压低。

        Args:
            now_ms: 当前 ticks_ms（传入可减少 ticks_ms 调用次数）
        """
        if self._uart is None:
            return
        if now_ms is None:
            now_ms = time.ticks_ms()

        t0_us = time.ticks_us()
        data = self._uart.read(self._READ_N)
        _record_drain_perf(self, t0_us, data)
        if data:
            for cmd, payload in self._parser.feed(data):
                if cmd == CMD_VEHICLE_STATUS and len(payload) >= 7:
                    self._last_rx_ms = now_ms
                    self.vehicle_avg_cps, self.vehicle_safety, self.vehicle_bat_mv = (
                        parse_vehicle_status(payload)
                    )
                elif cmd == CMD_HEARTBEAT_MCU:
                    self._last_rx_ms = now_ms

    # ------------------------------------------------------------------
    # 状态查询
    # ------------------------------------------------------------------

    def is_online(self, now_ms=None):
        """MCU 在线：最近一次合法 MCU 上行帧在 timeout_ms 之内。"""
        if now_ms is None:
            now_ms = time.ticks_ms()
        return time.ticks_diff(now_ms, self._last_rx_ms) < self._timeout_ms

    def is_safe_to_drive(self, now_ms=None):
        """可以发运动指令：MCU 在线 且 safety_state < FALLEN。"""
        return self.is_online(now_ms) and self.vehicle_safety < SAFETY_FALLEN

    def stats(self):
        """返回 (good_frames, bad_frames)。"""
        return self._parser.good, self._parser.bad

    def perf_stats(self):
        """返回 (rx_bytes, last_read_len, drain_calls, avg_us, max_us)。"""
        return _perf_stats(self)

    def bad_stats(self):
        """返回 MCUFrameParser 的坏帧分类统计。"""
        return self._parser.bad_stats()

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
