#!/usr/bin/env python3
# -*- coding: ascii -*-
"""Host-side tests for the C30D UART4 laser frame parser.

The parser class below is a byte-for-byte mirror of the state machine in
USER/laser.c (laser_parse_byte): hunt 0xAA, expect 0x55, collect 7 payload
bytes + 1 CRC8 byte, accept only a CRC-good frame.  The frame builder mirrors
the F103 sender described in USER/app_if.h.

Run with::

    python tools/test_laser_parse.py -v
"""

import struct
import unittest

HEADER = (0xAA, 0x55)
PAYLOAD_LEN = 7
FRAME_TAIL = 8              # 7 payload bytes + 1 CRC byte
CRC8_POLY = 0x07
INVALID_MM = 0xFFFF


def crc8(data, crc=0):
    """CRC8 poly 0x07, init 0x00."""
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(dl, dr, df, flags=0x07):
    """10-byte frame: AA 55 dL dR dF flags CRC8 (little endian, mm)."""
    body = bytes(HEADER) + struct.pack("<HHHB", dl, dr, df, flags)
    return body + bytes((crc8(body),))


class LaserParser(object):
    """Mirror of laser_parse_byte() in USER/laser.c."""

    HUNT0, HUNT1, PAYLOAD = 0, 1, 2

    def __init__(self):
        self.state = self.HUNT0
        self.idx = 0
        self.buf = bytearray()

    def feed(self, data):
        """Feed bytes; return the list of decoded frames (0 or more)."""
        out = []
        for byte in data:
            frame = self._feed_byte(byte)
            if frame is not None:
                out.append(frame)
        return out

    def _feed_byte(self, byte):
        if self.state == self.HUNT0:
            if byte == HEADER[0]:
                self.state = self.HUNT1
            return None

        if self.state == self.HUNT1:
            if byte == HEADER[1]:
                self.state = self.PAYLOAD
                self.idx = 0
                self.buf = bytearray()
            elif byte != HEADER[0]:
                self.state = self.HUNT0
            return None

        self.buf.append(byte)
        self.idx += 1
        if self.idx < FRAME_TAIL:
            return None

        self.state = self.HUNT0
        self.idx = 0
        frame = bytes(HEADER) + bytes(self.buf[:PAYLOAD_LEN])
        if crc8(frame) != self.buf[PAYLOAD_LEN]:
            return None

        return {
            "dL": self.buf[0] | (self.buf[1] << 8),
            "dR": self.buf[2] | (self.buf[3] << 8),
            "dF": self.buf[4] | (self.buf[5] << 8),
            "flags": self.buf[6],
        }


def decode(frame):
    return {
        "dL": frame[2] | (frame[3] << 8),
        "dR": frame[4] | (frame[5] << 8),
        "dF": frame[6] | (frame[7] << 8),
        "flags": frame[8],
    }


class CrcVectorTests(unittest.TestCase):
    def test_known_vectors(self):
        vectors = (
            ((150, 140, 300), "AA 55 96 00 8C 00 2C 01 07 93"),
            ((200, 200, 500), "AA 55 C8 00 C8 00 F4 01 07 1A"),
            ((100, 300, 800), "AA 55 64 00 2C 01 20 03 07 A2"),
        )
        for (dl, dr, df), expected in vectors:
            frame = build_frame(dl, dr, df)
            self.assertEqual(" ".join("%02X" % b for b in frame), expected)


class ParserTests(unittest.TestCase):
    def setUp(self):
        self.parser = LaserParser()

    def test_valid_frame(self):
        frame = build_frame(150, 140, 300, 0x07)
        frames = self.parser.feed(frame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0], {"dL": 150, "dR": 140, "dF": 300,
                                     "flags": 0x07})

    def test_known_vectors_parse(self):
        for dl, dr, df in ((150, 140, 300), (200, 200, 500), (100, 300, 800)):
            frame = build_frame(dl, dr, df)
            frames = self.parser.feed(frame)
            self.assertEqual(len(frames), 1)
            self.assertEqual((frames[0]["dL"], frames[0]["dR"],
                              frames[0]["dF"]), (dl, dr, df))

    def test_bad_crc_payload_rejected(self):
        frame = bytearray(build_frame(150, 140, 300))
        frame[3] ^= 0x01          # corrupt dL high byte, keep old CRC
        self.assertEqual(self.parser.feed(bytes(frame)), [])

    def test_bad_crc_checksum_rejected(self):
        frame = bytearray(build_frame(150, 140, 300))
        frame[9] ^= 0x01
        self.assertEqual(self.parser.feed(bytes(frame)), [])

    def test_garbage_before_header(self):
        garbage = b"\x00\xFF\xAA\x00\x55\x12\xAA\xAA"
        good = build_frame(10, 20, 30)
        frames = self.parser.feed(garbage + good)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0], {"dL": 10, "dR": 20, "dF": 30,
                                     "flags": 0x07})

    def test_garbage_only_no_frame(self):
        self.assertEqual(self.parser.feed(b"\x00\xAA\x00\x55\xAA\x12"), [])

    def test_split_frame_byte_by_byte(self):
        frame = build_frame(111, 222, 333)
        for i, byte in enumerate(frame):
            frames = self.parser.feed(bytes((byte,)))
            if i < len(frame) - 1:
                self.assertEqual(frames, [])
        self.assertEqual(frames, [{"dL": 111, "dR": 222, "dF": 333,
                                   "flags": 0x07}])

    def test_multiple_frames_one_chunk(self):
        a = build_frame(1, 2, 3)
        b = build_frame(1000, 2000, 3000, 0x05)
        frames = self.parser.feed(a + b)
        self.assertEqual([(f["dL"], f["dR"], f["dF"], f["flags"]) for f in frames],
                         [(1, 2, 3, 0x07), (1000, 2000, 3000, 0x05)])

    def test_multiple_frames_split_chunks(self):
        a = build_frame(1, 2, 3)
        b = build_frame(4, 5, 6)
        stream = a + b
        frames = self.parser.feed(stream[:4]) + self.parser.feed(stream[4:13])
        self.assertEqual(len(frames), 1)
        frames += self.parser.feed(stream[13:])
        self.assertEqual(len(frames), 2)

    def test_invalid_distance_sentinel_preserved(self):
        frame = build_frame(INVALID_MM, 0, INVALID_MM)
        frames = self.parser.feed(frame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["dL"], 0xFFFF)
        self.assertEqual(frames[0]["dR"], 0)
        self.assertEqual(frames[0]["dF"], 0xFFFF)

    def test_payload_contains_header_bytes(self):
        # AA / 55 inside the distances must not be treated as a header: the
        # parser only trusts the 0xAA 0x55 sequence that starts a frame.
        frames = self.parser.feed(build_frame(0x55AA, 0xAA55, 0xFF00))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["dL"], 0x55AA)
        self.assertEqual(frames[0]["dR"], 0xAA55)
        self.assertEqual(frames[0]["dF"], 0xFF00)

    def test_max_distance(self):
        frames = self.parser.feed(build_frame(65534, 1, 65535, 0xFF))
        self.assertEqual(frames[0]["dL"], 65534)
        self.assertEqual(frames[0]["flags"], 0xFF)

    def test_incomplete_trailing_bytes(self):
        frame = build_frame(150, 140, 300)
        self.assertEqual(self.parser.feed(frame[:9]), [])
        frames = self.parser.feed(frame[9:])
        self.assertEqual(len(frames), 1)

    def test_truncated_frame_then_resync(self):
        good = build_frame(150, 140, 300)
        stream = good[:5] + good + good
        frames = self.parser.feed(stream)
        # The first truncated frame eats bytes of the next one, so that frame
        # fails CRC; the final complete frame must still be decoded.
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0], decode(good))

    def test_raw_byte_count(self):
        # The C module counts bytes separately; the parser itself must not
        # require a pre-aligned stream, and all bytes must be consumed.
        data = b"\xAA\x55" + bytes(range(8))
        frames = self.parser.feed(data)
        self.assertEqual(frames, [])
        self.assertEqual(self.parser.state, LaserParser.HUNT0)


if __name__ == "__main__":
    unittest.main()
