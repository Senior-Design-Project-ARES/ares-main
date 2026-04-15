#!/usr/bin/env python3
"""
Send ASCII command lines (or ad-hoc ASCII payloads) to a serial device.

Run Cmd: 
python3 /home/ares/tests/send_telem_frame.py --port /dev/ttyACM0 --baud 115200

python3 uart_over_usb.py \
  --port /dev/cu.usbmodem1403 \
  --baud 115200 \
  --rate 10 \
  --duration 20 \
  --read-debug

Help Cmd:
python3 /home/ares/tests/send_telem_frame.py --help

Default command line format:
  V<vx>,Y<yaw>\n
Example:
  V0.30,Y5.00\n
"""

import argparse
import os
import select
import struct
import sys
import termios
import time
import codecs


DEBUG_MAGIC = 0xD66D
DEBUG_FORMAT = "<HIIffBBHIIIIIIIIIIH"
DEBUG_LEN = struct.calcsize(DEBUG_FORMAT)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def baud_to_termios(baud: int) -> int:
    mapping = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }
    if hasattr(termios, "B460800"):
        mapping[460800] = termios.B460800
    if hasattr(termios, "B921600"):
        mapping[921600] = termios.B921600
    if baud not in mapping:
        raise ValueError(f"Unsupported baud: {baud}")
    return mapping[baud]


def open_serial(path: str, baud: int) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
    attrs = termios.tcgetattr(fd)

    # iflag, oflag, cflag, lflag, ispeed, ospeed, cc
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CREAD | termios.CLOCAL | termios.CS8
    attrs[3] = 0
    speed = baud_to_termios(baud)
    attrs[4] = speed
    attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0

    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def normalize_port(path: str) -> str:
    """On macOS, prefer cu.* over tty.* to avoid carrier-wait opens."""
    if sys.platform != "darwin":
        return path

    if path.startswith("/dev/tty."):
        candidate = path.replace("/dev/tty.", "/dev/cu.", 1)
        if os.path.exists(candidate):
            print(f"macOS: using {candidate} instead of {path}")
            return candidate

    return path


def build_ascii_cmd(vx: float, yaw_rate: float) -> bytes:
    return f"V{vx:.3f},Y{yaw_rate:.3f}\n".encode("ascii")

def parse_debug_frames(buf: bytearray):
    frames = []
    i = 0
    while i + DEBUG_LEN <= len(buf):
        magic = int.from_bytes(buf[i : i + 2], "little")
        if magic != DEBUG_MAGIC:
            i += 1
            continue
        raw = bytes(buf[i : i + DEBUG_LEN])
        fields = struct.unpack(DEBUG_FORMAT, raw)
        recv_crc = fields[-1]
        calc_crc = crc16_ccitt(raw[:-2])
        if recv_crc != calc_crc:
            i += 1
            continue
        (
            _,
            ms,
            seq,
            cmd_vx,
            cmd_yaw,
            rx_state,
            line_len,
            _reserved,
            parser_rx_bytes,
            parser_ok,
            parser_fail_parse,
            parser_fail_overflow,
            irq_rx_bytes,
            ring_overflow,
            err_overrun,
            err_framing,
            err_noise,
            err_parity,
            _,
        ) = fields
        frames.append(
            {
                "ms": ms,
                "seq": seq,
                "cmd_vx": cmd_vx,
                "cmd_yaw": cmd_yaw,
                "rx_state": rx_state,
                "line_len": line_len,
                "parser_rx_bytes": parser_rx_bytes,
                "parser_ok": parser_ok,
                "parser_fail_parse": parser_fail_parse,
                "parser_fail_overflow": parser_fail_overflow,
                "irq_rx_bytes": irq_rx_bytes,
                "ring_overflow": ring_overflow,
                "err_overrun": err_overrun,
                "err_framing": err_framing,
                "err_noise": err_noise,
                "err_parity": err_parity,
            }
        )
        i += DEBUG_LEN
    if i > 0:
        del buf[:i]
    return frames


def main() -> int:
    parser = argparse.ArgumentParser(description="Send compact command bytes to STM over serial.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial device path")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--rate", type=float, default=20.0, help="Frame rate in Hz")
    parser.add_argument("--vx", type=float, default=0.3, help="vx field (m/s)")
    parser.add_argument("--yaw-rate", type=float, default=0.0, help="yaw_rate field")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run (0 = forever)")
    parser.add_argument(
        "--ascii-test",
        default="",
        help=r'Send this ASCII payload each cycle instead of binary frame (supports escapes, e.g. "w\\n")',
    )
    parser.add_argument(
        "--read-debug",
        action="store_true",
        help="Also read and print STM debug bytes from the same serial port",
    )
    parser.add_argument(
        "--read-status",
        action="store_true",
        help="Parse binary debug telemetry frames sent by cmd.cpp",
    )
    parser.add_argument(
        "--rx-max-bytes-per-loop",
        type=int,
        default=256,
        help="Max debug bytes to drain per send loop when --read-debug is enabled",
    )
    parser.add_argument(
        "--rx-raw-stats",
        action="store_true",
        help="Print periodic raw RX byte counts (helps when no newline-terminated debug lines)",
    )
    parser.add_argument(
        "--rx-hex-preview",
        type=int,
        default=0,
        help="If >0, print up to N bytes/chunk as hex when --read-debug is enabled",
    )
    args = parser.parse_args()

    if args.rate <= 0:
        raise ValueError("--rate must be > 0")
    if args.rx_max_bytes_per_loop <= 0:
        raise ValueError("--rx-max-bytes-per-loop must be > 0")
    if args.rx_hex_preview < 0:
        raise ValueError("--rx-hex-preview must be >= 0")

    port = normalize_port(args.port)
    fd = open_serial(port, args.baud)
    ascii_payload = b""
    if args.ascii_test:
        ascii_payload = codecs.decode(args.ascii_test, "unicode_escape").encode("utf-8")
        if len(ascii_payload) == 0:
            raise ValueError("--ascii-test produced empty payload")
        print(
            f"Opened {port} @ {args.baud}, ascii_test_len={len(ascii_payload)} bytes payload={ascii_payload!r}"
        )
    else:
        print(f"Opened {port} @ {args.baud}, ascii_cmd=V{args.vx:.3f},Y{args.yaw_rate:.3f}")

    sent = 0
    start = time.monotonic()
    next_log = start + 0.5
    next_rx_log = start + 1.0
    period = 1.0 / args.rate
    debug_buf = bytearray()
    status_buf = bytearray()
    raw_rx_total = 0

    try:
        while True:
            now = time.monotonic()
            t_s = now - start
            if ascii_payload:
                os.write(fd, ascii_payload)
            else:
                cmd = build_ascii_cmd(args.vx, args.yaw_rate)
                os.write(fd, cmd)
            sent += 1

            if now >= next_log:
                if ascii_payload:
                    print(f"sent={sent} ascii_len={len(ascii_payload)} elapsed={t_s:.2f}s")
                else:
                    print(f"sent={sent} ascii_len={len(cmd)} elapsed={t_s:.2f}s")
                next_log += 0.5

            if args.read_debug or args.read_status:
                # Drain at most N bytes/loop so host debug handling cannot starve TX pacing.
                drained = 0
                while drained < args.rx_max_bytes_per_loop:
                    readable, _, _ = select.select([fd], [], [], 0.0)
                    if not readable:
                        break
                    chunk = os.read(fd, min(64, args.rx_max_bytes_per_loop - drained))
                    if not chunk:
                        break
                    drained += len(chunk)
                    raw_rx_total += len(chunk)
                    if args.rx_hex_preview > 0:
                        preview = chunk[: args.rx_hex_preview]
                        hex_bytes = " ".join(f"{b:02X}" for b in preview)
                        print(f"[stm-raw] {len(chunk)}B: {hex_bytes}")
                    if args.read_status:
                        status_buf.extend(chunk)
                        for sf in parse_debug_frames(status_buf):
                            print(
                                "[stm-status] "
                                f"ms={sf['ms']} seq={sf['seq']} cmd_vx={sf['cmd_vx']:.3f} cmd_yaw={sf['cmd_yaw']:.3f} "
                                f"rx_state={sf['rx_state']} line_len={sf['line_len']} parser_rx={sf['parser_rx_bytes']} "
                                f"ok={sf['parser_ok']} fail_parse={sf['parser_fail_parse']} fail_overflow={sf['parser_fail_overflow']} "
                                f"irq_rx={sf['irq_rx_bytes']} ring_ovf={sf['ring_overflow']} "
                                f"err(ore/fe/ne/pe)={sf['err_overrun']}/{sf['err_framing']}/{sf['err_noise']}/{sf['err_parity']}"
                            )
                    if args.read_debug:
                        debug_buf.extend(chunk)
                        while b"\n" in debug_buf:
                            line, _, rest = debug_buf.partition(b"\n")
                            debug_buf = bytearray(rest)
                            text = line.decode("utf-8", errors="replace").rstrip("\r")
                            if text:
                                print(f"[stm] {text}")
                if args.rx_raw_stats and now >= next_rx_log:
                    print(f"[stm-raw] total_rx_bytes={raw_rx_total}")
                    next_rx_log += 1.0

            if args.duration > 0 and t_s >= args.duration:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        if args.read_debug and debug_buf:
            text = debug_buf.decode("utf-8", errors="replace").rstrip("\r\n")
            if text:
                print(f"[stm] {text}")
        os.close(fd)
        print("Closed serial port")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
