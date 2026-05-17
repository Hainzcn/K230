"""MCU ↔ K230 业务帧定义（CMD 枚举、字段打包 / 解包）。

依赖 comms.frame.encode_frame 完成实际帧构造。

帧方向与频率：
    MCU → K230
        CMD_VEHICLE_STATUS  0x01   20 Hz
        CMD_HEARTBEAT_MCU   0x02    1 Hz
    K230 → MCU
        CMD_MOTION_CMD      0x11  20~50 Hz
        CMD_HEARTBEAT_K230  0x12    1 Hz
        CMD_PID_INJECT      0x13  按需
"""

import struct as _struct
from comms.frame import encode_frame

# ---------------------------------------------------------------------------
# CMD 常量
# ---------------------------------------------------------------------------

CMD_VEHICLE_STATUS  = 0x01
CMD_HEARTBEAT_MCU   = 0x02

CMD_MOTION_CMD      = 0x11
CMD_HEARTBEAT_K230  = 0x12
CMD_PID_INJECT      = 0x13

# ---------------------------------------------------------------------------
# 安全状态常量（VEHICLE_STATUS.safety_state 字段）
# ---------------------------------------------------------------------------

SAFETY_DISARMED = 0   # 未解锁（静止待命）
SAFETY_ARMED    = 1   # 正常行驶
SAFETY_BAT_WARN = 2   # 低电压警告
SAFETY_FALLEN   = 3   # 跌倒
SAFETY_BAT_STOP = 4   # 电压过低急停

# ---------------------------------------------------------------------------
# VEHICLE_STATUS 字段说明（payload 格式 '<iBH'，共 7 字节）
#
#   avg_cps      int32   LE   左右轮平均速度 counts/s（正=前进）
#   safety_state uint8       见 SAFETY_* 常量
#   bat_mv       uint16  LE  电池电压 mV
# ---------------------------------------------------------------------------

_VEHICLE_STATUS_FMT  = '<iBH'
_VEHICLE_STATUS_SIZE = _struct.calcsize(_VEHICLE_STATUS_FMT)   # 7

# ---------------------------------------------------------------------------
# MCU → K230 解包
# ---------------------------------------------------------------------------

def parse_vehicle_status(payload):
    """解析 VEHICLE_STATUS 帧 payload。

    Args:
        payload: bytes，长度必须 == 7

    Returns:
        (avg_cps: int, safety_state: int, bat_mv: int)
        解析失败时返回 (0, SAFETY_DISARMED, 0)
    """
    if len(payload) < _VEHICLE_STATUS_SIZE:
        return (0, SAFETY_DISARMED, 0)
    avg_cps, safety_state, bat_mv = _struct.unpack_from(
        _VEHICLE_STATUS_FMT, payload, 0
    )
    return avg_cps, safety_state, bat_mv


def parse_heartbeat_mcu(payload):
    """解析 HEARTBEAT_MCU 帧 payload，返回 uptime_ms (int)。"""
    if len(payload) < 4:
        return 0
    return _struct.unpack_from('<I', payload, 0)[0]


# ---------------------------------------------------------------------------
# K230 → MCU 构造
# ---------------------------------------------------------------------------

def make_motion_cmd(target_v, target_omega, mode=1):
    """构造 MOTION_CMD 帧。

    Args:
        target_v:     int，纵向速度（counts/s 已除以 SCALE=10，正=前进）
                      例：target_v=500 ≡ 5000 raw cps ≈ 低速前行
        target_omega: int，转向差分量 permille（正=顺时针俯视右转）
        mode:         int，0=停止（紧急），1=正常行驶

    Returns:
        bytes，完整帧
    """
    payload = _struct.pack('<hhB', target_v, target_omega, mode)
    return encode_frame(CMD_MOTION_CMD, payload)


def make_heartbeat_k230(uptime_ms):
    """构造 HEARTBEAT_K230 帧。

    Args:
        uptime_ms: int，K230 运行时间 ms（可直接传 time.ticks_ms()）

    Returns:
        bytes，完整帧
    """
    return encode_frame(CMD_HEARTBEAT_K230, _struct.pack('<I', uptime_ms & 0xFFFFFFFF))


def make_pid_inject(pid_id, kp, ki, kd):
    """构造 PID_INJECT 帧（远程调参，调试期用）。

    Args:
        pid_id: int，0=angle环, 1=rate环, 2=speed环, 3=yaw环
        kp, ki, kd: float

    Returns:
        bytes，完整帧
    """
    payload = _struct.pack('<Bfff', pid_id, kp, ki, kd)
    return encode_frame(CMD_PID_INJECT, payload)


# ---------------------------------------------------------------------------
# 自检
# ---------------------------------------------------------------------------

def self_test():
    """协议层往返自检，返回 True 表示通过。"""
    from comms.frame import MCUFrameParser, encode_frame as _enc

    # 1. make_motion_cmd 往返
    frame = make_motion_cmd(100, -50, 1)
    p = MCUFrameParser()
    frames = p.feed(frame)
    if len(frames) != 1 or frames[0][0] != CMD_MOTION_CMD:
        print("[protocol] self_test FAILED: make_motion_cmd decode")
        return False
    v, omega, mode_out = _struct.unpack_from('<hhB', frames[0][1], 0)
    if v != 100 or omega != -50 or mode_out != 1:
        print("[protocol] self_test FAILED: motion_cmd fields v=%d omega=%d mode=%d"
              % (v, omega, mode_out))
        return False

    # 2. parse_vehicle_status 往返
    import struct
    vs_payload = struct.pack(_VEHICLE_STATUS_FMT, 2000, SAFETY_ARMED, 11100)
    vs_frame   = _enc(CMD_VEHICLE_STATUS, vs_payload)
    p2 = MCUFrameParser()
    fs = p2.feed(vs_frame)
    if not fs:
        print("[protocol] self_test FAILED: vehicle_status encode/decode")
        return False
    avg_cps, safety, bat = parse_vehicle_status(fs[0][1])
    if avg_cps != 2000 or safety != SAFETY_ARMED or bat != 11100:
        print("[protocol] self_test FAILED: vehicle_status fields")
        return False

    # 3. make_heartbeat_k230
    hb = make_heartbeat_k230(123456)
    p3 = MCUFrameParser()
    fs3 = p3.feed(hb)
    if not fs3 or fs3[0][0] != CMD_HEARTBEAT_K230:
        print("[protocol] self_test FAILED: heartbeat_k230")
        return False

    return True
