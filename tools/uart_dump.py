#!/usr/bin/env python3
"""PC 端 MCU ↔ K230 串口帧实时抓包 / 解码工具。

依赖：pyserial（pip install pyserial）

用法：
    # 监听 K230 → MCU 方向（921600，USB-TTL 接 MCU PB6/PB7）
    python tools/uart_dump.py --port COM3 --baud 921600

    # 监听 MS901M 直通（115200，USB-TTL 接 MS901M TX Y 分线）
    python tools/uart_dump.py --port COM4 --baud 115200 --imu

    # 回环自测（TX 短接 RX）：发 10 帧 MOTION_CMD 然后监听 10s
    python tools/uart_dump.py --port COM3 --baud 921600 --loopback

输出格式（CSV 到 stdout，同时打印到 stderr 方便 pipe 过滤）：
    ts_ms, dir, cmd_hex, length, decoded_fields...

Ctrl+C 退出，打印统计摘要。
"""

import sys
import time
import argparse
import struct

# ---------------------------------------------------------------------------
# 协议导入（优先从项目根导入，否则内联最小实现）
# ---------------------------------------------------------------------------
try:
    # 在项目根目录运行时，sys.path 已含项目根
    _project_root = __file__
    import os as _os
    _root = _os.path.dirname(_os.path.dirname(_os.path.abspath(_project_root)))
    if _root not in sys.path:
        sys.path.insert(0, _root)

    from comms.frame    import crc16_ccitt, MCUFrameParser, encode_frame
    from comms.ms901m   import MS901MParser
    from comms.protocol import (
        CMD_VEHICLE_STATUS, CMD_HEARTBEAT_MCU,
        CMD_MOTION_CMD, CMD_HEARTBEAT_K230, CMD_PID_INJECT,
        SAFETY_DISARMED, SAFETY_ARMED, SAFETY_BAT_WARN, SAFETY_FALLEN, SAFETY_BAT_STOP,
        parse_vehicle_status, parse_heartbeat_mcu,
        make_motion_cmd, make_heartbeat_k230,
    )
    _USING_PROJECT_COMMS = True
except ImportError as _e:
    print("[uart_dump] WARNING: cannot import comms package (%s), using inline impl" % _e,
          file=sys.stderr)
    _USING_PROJECT_COMMS = False

    # ---------- inline 最小实现 ----------
    def crc16_ccitt(data):
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
        return crc

    def encode_frame(cmd, payload=b''):
        length = len(payload)
        crc_data = bytes([length, cmd]) + bytes(payload)
        crc = crc16_ccitt(crc_data)
        return (bytes([0xAA, 0x55, length, cmd]) + bytes(payload)
                + bytes([crc & 0xFF, crc >> 8]) + bytes([0x55, 0xAA]))

    CMD_VEHICLE_STATUS  = 0x01
    CMD_HEARTBEAT_MCU   = 0x02
    CMD_MOTION_CMD      = 0x11
    CMD_HEARTBEAT_K230  = 0x12
    CMD_PID_INJECT      = 0x13

    SAFETY_DISARMED = 0; SAFETY_ARMED = 1; SAFETY_BAT_WARN = 2
    SAFETY_FALLEN = 3; SAFETY_BAT_STOP = 4

    _SAFETY_NAMES = {0: "DISARMED", 1: "ARMED", 2: "BAT_WARN", 3: "FALLEN", 4: "BAT_STOP"}

    def parse_vehicle_status(p):
        if len(p) < 7: return (0, 0, 0)
        return struct.unpack_from('<iBH', p, 0)

    def parse_heartbeat_mcu(p):
        if len(p) < 4: return 0
        return struct.unpack_from('<I', p, 0)[0]

    def make_motion_cmd(v, omega, mode=1):
        return encode_frame(CMD_MOTION_CMD, struct.pack('<hhB', v, omega, mode))

    def make_heartbeat_k230(uptime_ms):
        return encode_frame(CMD_HEARTBEAT_K230, struct.pack('<I', uptime_ms & 0xFFFFFFFF))

    class MCUFrameParser:
        _S = [0]  # 简化版，仅用于 loopback
        def __init__(self):
            self.good = 0; self.bad = 0
            self._buf = bytearray()
            self.last_cmd = 0; self.last_payload = bytes()
        def feed(self, data):
            self._buf += data
            frames = []
            while True:
                idx = self._buf.find(b'\xAA\x55')
                if idx < 0: self._buf = bytearray(); break
                self._buf = self._buf[idx:]
                if len(self._buf) < 6: break
                ln = self._buf[2]; cmd = self._buf[3]
                need = 4 + ln + 4
                if len(self._buf) < need: break
                payload = bytes(self._buf[4:4+ln])
                crc_rx = self._buf[4+ln] | (self._buf[4+ln+1] << 8)
                tail = self._buf[4+ln+2:4+ln+4]
                crc_data = bytes([ln, cmd]) + payload
                if crc16_ccitt(crc_data) == crc_rx and tail == b'\x55\xAA':
                    self.good += 1; frames.append((cmd, payload))
                else:
                    self.bad += 1
                self._buf = self._buf[need:]
            return frames

    class MS901MParser:
        def __init__(self):
            self.pitch_deg = 0.0; self.good_frames = 0; self.bad_frames = 0
        def feed(self, data): pass


_SAFETY_NAMES = {
    SAFETY_DISARMED: "DISARMED",
    SAFETY_ARMED:    "ARMED",
    SAFETY_BAT_WARN: "BAT_WARN",
    SAFETY_FALLEN:   "FALLEN",
    SAFETY_BAT_STOP: "BAT_STOP",
}

_CMD_NAMES = {
    CMD_VEHICLE_STATUS:  "VEHICLE_STATUS",
    CMD_HEARTBEAT_MCU:   "HEARTBEAT_MCU",
    CMD_MOTION_CMD:      "MOTION_CMD",
    CMD_HEARTBEAT_K230:  "HEARTBEAT_K230",
    CMD_PID_INJECT:      "PID_INJECT",
}

# ---------------------------------------------------------------------------
# 帧解码（字段级显示）
# ---------------------------------------------------------------------------

def decode_mcu_frame(cmd, payload):
    """将已解析帧翻译成可读字符串。"""
    if cmd == CMD_VEHICLE_STATUS:
        avg_cps, safety, bat_mv = parse_vehicle_status(payload)
        return "avg_cps=%+d safety=%s bat=%dmV" % (
            avg_cps, _SAFETY_NAMES.get(safety, str(safety)), bat_mv)
    if cmd == CMD_HEARTBEAT_MCU:
        return "uptime=%dms" % parse_heartbeat_mcu(payload)
    if cmd == CMD_MOTION_CMD and len(payload) >= 5:
        v, omega, mode = struct.unpack_from('<hhB', payload, 0)
        return "v=%+d omega=%+d mode=%d" % (v, omega, mode)
    if cmd == CMD_HEARTBEAT_K230:
        if len(payload) >= 4:
            return "uptime=%dms" % struct.unpack_from('<I', payload, 0)[0]
    if cmd == CMD_PID_INJECT and len(payload) >= 13:
        pid_id, kp, ki, kd = struct.unpack_from('<Bfff', payload, 0)
        return "pid_id=%d kp=%.4f ki=%.4f kd=%.4f" % (pid_id, kp, ki, kd)
    return "payload=%s" % payload.hex()


# ---------------------------------------------------------------------------
# 主入口
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="MCU↔K230 串口帧抓包解码工具")
    parser.add_argument("--port",     required=True, help="串口号，如 COM3 或 /dev/ttyUSB0")
    parser.add_argument("--baud",     type=int, default=921600, help="波特率（默认 921600）")
    parser.add_argument("--imu",      action="store_true",
                        help="监听 MS901M 115200 原始帧（不加此标志则监听 MCU 命令帧）")
    parser.add_argument("--loopback", action="store_true",
                        help="TX 短接 RX 回环自测：发 10 帧 MOTION_CMD，再监听 10s 统计")
    parser.add_argument("--duration", type=float, default=0,
                        help="监听时长 s（0=持续直到 Ctrl+C）")
    parser.add_argument("--csv",      action="store_true",
                        help="输出 CSV 格式（默认人可读格式）")
    args = parser.parse_args()

    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    print("[uart_dump] opened %s @ %d" % (args.port, args.baud), file=sys.stderr)

    if args.csv:
        print("ts_ms,dir,cmd_hex,cmd_name,length,decoded")

    # 回环自测：发若干帧
    if args.loopback:
        print("[uart_dump] loopback: sending 10 MOTION_CMD frames...", file=sys.stderr)
        for i in range(10):
            frame = make_motion_cmd(i * 10, 0, 1)
            ser.write(frame)
            time.sleep(0.01)

    frame_parser = MCUFrameParser()
    imu_parser   = MS901MParser() if args.imu else None

    t_start  = time.time()
    rx_bytes = 0
    total_frames = 0

    try:
        while True:
            if args.duration > 0 and (time.time() - t_start) >= args.duration:
                break

            data = ser.read(256)
            if not data:
                continue
            rx_bytes += len(data)
            ts_ms = int((time.time() - t_start) * 1000)

            if args.imu and imu_parser is not None:
                imu_parser.feed(data)
                if args.csv:
                    print("%d,IMU,-,-,%d,pitch=%.2f good=%d bad=%d" % (
                        ts_ms, len(data),
                        imu_parser.pitch_deg,
                        imu_parser.good_frames, imu_parser.bad_frames))
                else:
                    print("[%7dms] IMU  pitch=%+6.2f° good=%d bad=%d" % (
                        ts_ms, imu_parser.pitch_deg,
                        imu_parser.good_frames, imu_parser.bad_frames))
                continue

            for cmd, payload in frame_parser.feed(data):
                total_frames += 1
                cmd_name = _CMD_NAMES.get(cmd, "CMD_0x%02X" % cmd)
                decoded  = decode_mcu_frame(cmd, payload)
                if args.csv:
                    print("%d,MCU,0x%02X,%s,%d,%s" % (
                        ts_ms, cmd, cmd_name, len(payload), decoded))
                else:
                    print("[%7dms] %-20s 0x%02X  len=%-2d  %s" % (
                        ts_ms, cmd_name, cmd, len(payload), decoded))
                sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        good, bad = frame_parser.good, frame_parser.bad
        print(
            "\n[uart_dump] done  rx_bytes=%d frames=%d good=%d bad=%d elapsed=%.1fs"
            % (rx_bytes, total_frames, good, bad, time.time() - t_start),
            file=sys.stderr,
        )


if __name__ == "__main__":
    main()
