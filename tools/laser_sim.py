#!/usr/bin/env python3
# -*- coding: ascii -*-
"""PC-side frame generator / simulator for the C30D UART4 laser link.

Wire format (10 bytes, distances little endian in mm):

    0xAA 0x55 | dL(2) dR(2) dF(2) | flags | CRC8

* 0xFFFF marks an invalid distance.
* CRC8: polynomial 0x07, init 0x00, computed over 0xAA 0x55 + the 7 payload
  bytes (header, 7 payload bytes, checksum -- in that order).

Examples
--------
Print one frame::

    python tools/laser_sim.py --l 150 --r 140 --f 300
    AA 55 96 00 8C 00 2C 01 07 93

Verify the protocol against the known-good test vectors::

    python tools/laser_sim.py --selftest

Repeatedly send frames to the laser F103 board (or a USB-UART adapter) from
the PC -- pyserial is optional and only imported when --port is used::

    python tools/laser_sim.py --l 150 --r 140 --f 300 --port COM7 --repeat 0

Only the Python 3 standard library is required for frame generation and
selftest; no third-party package is needed unless --port is used.
"""

import argparse
import struct
import sys
import time

HEADER = b"\xAA\x55"
CRC8_POLY = 0x07
INVALID_MM = 0xFFFF

FLAG_FRONT_OK = 0x01
FLAG_LEFT_OK = 0x02
FLAG_RIGHT_OK = 0x04
FLAG_TIMEOUT = 0x08
FLAG_ALL_OK = FLAG_FRONT_OK | FLAG_LEFT_OK | FLAG_RIGHT_OK

# (dL, dR, dF, flags, expected frame hex) -- fixed acceptance vectors.
SELFTEST_VECTORS = (
    (150, 140, 300, FLAG_ALL_OK, "AA 55 96 00 8C 00 2C 01 07 93"),
    (200, 200, 500, FLAG_ALL_OK, "AA 55 C8 00 C8 00 F4 01 07 1A"),
    (100, 300, 800, FLAG_ALL_OK, "AA 55 64 00 2C 01 20 03 07 A2"),
)


def crc8(data, crc=0):
    """CRC8 poly 0x07, init 0x00, no reflection, no final xor."""
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(dl, dr, df, flags=FLAG_ALL_OK):
    """Return the 10-byte frame for three distances (mm) and a flags byte."""
    for name, value in (("dL", dl), ("dR", dr), ("dF", df), ("flags", flags)):
        if not 0 <= value <= 0xFFFF:
            raise ValueError("%s out of range 0..65535: %r" % (name, value))
    body = HEADER + struct.pack("<HHHB", dl, dr, df, flags)
    return body + bytes((crc8(body),))


def frame_hex(frame):
    """'AA 55 ...' upper-case hex representation."""
    return " ".join("%02X" % byte for byte in frame)


def verify_frame(frame):
    """True when frame is exactly 10 bytes with a correct CRC8."""
    return (
        len(frame) == 10
        and frame[0:2] == HEADER
        and crc8(frame[:9]) == frame[9]
    )


def selftest():
    """Check CRC/frame construction against the known-good vectors."""
    failures = 0
    for dl, dr, df, flags, expected in SELFTEST_VECTORS:
        got = frame_hex(build_frame(dl, dr, df, flags))
        ok = got == expected and verify_frame(build_frame(dl, dr, df, flags))
        print("[%s] %3d/%3d/%3d flags=0x%02X -> %s" %
              ("OK" if ok else "FAIL", dl, dr, df, flags, got))
        if not ok:
            failures += 1
            print("       expected: %s" % expected)

    # A corrupted CRC must be rejected by verify_frame().
    good = build_frame(150, 140, 300)
    bad_crc = good[:9] + bytes((good[9] ^ 0x01,))
    bad_payload = bytes((good[0], good[1], good[2] ^ 0x01)) + good[3:]
    for name, bad in (("crc", bad_crc), ("payload", bad_payload)):
        ok = not verify_frame(bad)
        print("[%s] corrupted %s rejected" % ("OK" if ok else "FAIL", name))
        if not ok:
            failures += 1

    print("SELFTEST %s (%d failure(s))" %
          ("PASSED" if failures == 0 else "FAILED", failures))
    return 0 if failures == 0 else 1


def send_serial(frames, port, baud, repeat, interval):
    """Send frames to a serial port; pyserial is imported lazily and guarded."""
    try:
        import serial  # type: ignore
    except ImportError:
        sys.stderr.write(
            "pyserial is not available, so --port %s cannot be opened.\n"
            "The generated frame hex above is still valid; install pyserial\n"
            "or use an existing serial-terminal tool to send those bytes.\n"
            % port
        )
        return 2

    try:
        with serial.Serial(port, baud, timeout=0) as ser:
            sent = 0
            try:
                while repeat == 0 or sent < repeat:
                    frame = frames[sent % len(frames)]
                    ser.write(frame)
                    ser.flush()
                    sent += 1
                    if repeat != 0 and sent >= repeat:
                        break
                    time.sleep(interval)
            except KeyboardInterrupt:
                sys.stderr.write("\nstopped by user\n")
    except (OSError, serial.SerialException) as exc:
        sys.stderr.write("serial error on %s: %s\n" % (port, exc))
        return 3

    sys.stderr.write("sent %d frame(s) to %s @ %d 8N1\n" % (sent, port, baud))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Build/send C30D UART4 laser frames (AA 55 ... CRC8).")
    parser.add_argument("--l", "--left", dest="left", type=int, default=150,
                        help="left distance in mm (default 150)")
    parser.add_argument("--r", "--right", dest="right", type=int, default=140,
                        help="right distance in mm (default 140)")
    parser.add_argument("--f", "--front", dest="front", type=int, default=300,
                        help="front distance in mm (default 300)")
    parser.add_argument("--flags", type=lambda s: int(s, 0), default=FLAG_ALL_OK,
                        help="flags byte, e.g. 0x07 (default 0x07)")
    parser.add_argument("--repeat", type=int, default=1,
                        help="frames to print/send; 0 = forever when --port is "
                             "used, otherwise print one (default 1)")
    parser.add_argument("--interval", type=float, default=0.05,
                        help="seconds between frames on --port (default 0.05)")
    parser.add_argument("--port", default=None,
                        help="optional serial port, e.g. COM7 (needs pyserial)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="baud rate for --port (default 115200)")
    parser.add_argument("--selftest", action="store_true",
                        help="verify CRC against known frames and exit")
    args = parser.parse_args(argv)

    if args.selftest:
        return selftest()

    if args.repeat < 0:
        parser.error("--repeat must be >= 0 (0 = forever)")

    try:
        frame = build_frame(args.left, args.right, args.front, args.flags)
    except ValueError as exc:
        parser.error(str(exc))

    sys.stderr.write("frame: %s\n" % frame_hex(frame))
    sys.stderr.write("       dL=%d dR=%d dF=%d flags=0x%02X\n" %
                     (args.left, args.right, args.front, args.flags))

    if args.port is None:
        count = 1 if args.repeat == 0 else args.repeat
        for _ in range(count):
            print(frame_hex(frame))
        return 0

    return send_serial([frame], args.port, args.baud, args.repeat, args.interval)


if __name__ == "__main__":
    sys.exit(main())
