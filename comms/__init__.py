"""K230 通讯子包（Phase D）。

公开接口：
    MS901MParser   — IMU 流式帧解析器（comms.ms901m）
    MCUFrameParser — MCU→K230 帧解析器（comms.frame）
    crc16_ccitt    — CRC16-CCITT 计算（comms.frame）
    encode_frame   — K230→MCU 帧编码（comms.frame）
    ImuLink        — UART(1,115200) + MS901MParser 封装（comms.uart_link）
    McuLink        — UART(2,921600) + 业务帧封装（comms.uart_link）

协议常量（comms.protocol）：
    CMD_VEHICLE_STATUS, CMD_HEARTBEAT_MCU
    CMD_MOTION_CMD, CMD_HEARTBEAT_K230, CMD_PID_INJECT
    SAFETY_DISARMED, SAFETY_ARMED, SAFETY_BAT_WARN, SAFETY_FALLEN, SAFETY_BAT_STOP

业务帧构造 / 解析：
    make_motion_cmd, make_heartbeat_k230, make_pid_inject
    parse_vehicle_status, parse_heartbeat_mcu
"""

from comms.ms901m import MS901MParser
from comms.frame  import crc16_ccitt, encode_frame, MCUFrameParser
from comms.uart_link import ImuLink, McuLink
from comms.protocol import (
    CMD_VEHICLE_STATUS,
    CMD_HEARTBEAT_MCU,
    CMD_MOTION_CMD,
    CMD_HEARTBEAT_K230,
    CMD_PID_INJECT,
    SAFETY_DISARMED,
    SAFETY_ARMED,
    SAFETY_BAT_WARN,
    SAFETY_FALLEN,
    SAFETY_BAT_STOP,
    parse_vehicle_status,
    parse_heartbeat_mcu,
    make_motion_cmd,
    make_heartbeat_k230,
    make_pid_inject,
)

__all__ = [
    "MS901MParser",
    "MCUFrameParser",
    "crc16_ccitt",
    "encode_frame",
    "ImuLink",
    "McuLink",
    "CMD_VEHICLE_STATUS",
    "CMD_HEARTBEAT_MCU",
    "CMD_MOTION_CMD",
    "CMD_HEARTBEAT_K230",
    "CMD_PID_INJECT",
    "SAFETY_DISARMED",
    "SAFETY_ARMED",
    "SAFETY_BAT_WARN",
    "SAFETY_FALLEN",
    "SAFETY_BAT_STOP",
    "parse_vehicle_status",
    "parse_heartbeat_mcu",
    "make_motion_cmd",
    "make_heartbeat_k230",
    "make_pid_inject",
]
