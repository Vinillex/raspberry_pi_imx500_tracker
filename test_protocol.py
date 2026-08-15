#!/usr/bin/env python3
"""
Unit tests for crsf_protocol. Runs anywhere - no Pi, no serial, no camera.

    python3 test_protocol.py
"""

import random
import sys

from config import CRSF_SYNC, TYPE_RC_CHANNELS, CRSF_MIN, CRSF_MID, CRSF_MAX
from crsf_protocol import (crc8_dvb_s2, unpack_channels, pack_channels,
                           build_frame, parse_frames, crsf_to_us,
                           clamp_channel, crsf_to_range)

failures = []


def check(name, cond):
    if cond:
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}")
        failures.append(name)


print("\nCRC")
# Known CRSF property: CRC of a frame body validates to the frame's CRC byte.
check("crc of empty is 0", crc8_dvb_s2(b"") == 0)
check("crc is deterministic", crc8_dvb_s2(b"\x16\x01\x02") == crc8_dvb_s2(b"\x16\x01\x02"))
check("crc differs on change", crc8_dvb_s2(b"\x16\x01") != crc8_dvb_s2(b"\x16\x02"))
check("crc in byte range", all(0 <= crc8_dvb_s2(bytes([i])) <= 255 for i in range(256)))

print("\nChannel pack/unpack round-trip")
cases = {
    "all centre": [CRSF_MID] * 16,
    "all min": [CRSF_MIN] * 16,
    "all max": [CRSF_MAX] * 16,
    "ascending": [(172 + i * 100) & 0x7FF for i in range(16)],
    "alternating": [CRSF_MIN if i % 2 else CRSF_MAX for i in range(16)],
}
for name, ch in cases.items():
    packed = pack_channels(ch)
    check(f"{name}: 22 bytes", len(packed) == 22)
    check(f"{name}: round-trips", unpack_channels(packed) == ch)

random.seed(42)
ok = True
for _ in range(2000):
    ch = [random.randint(0, 0x7FF) for _ in range(16)]
    if unpack_channels(pack_channels(ch)) != ch:
        ok = False
        break
check("2000 random vectors round-trip", ok)

print("\nFrame construction")
frame = build_frame(TYPE_RC_CHANNELS, pack_channels([CRSF_MID] * 16))
check("starts with sync", frame[0] == CRSF_SYNC)
check("total length 26", len(frame) == 26)
check("length byte correct", frame[1] == 24)
check("type byte correct", frame[2] == TYPE_RC_CHANNELS)
check("crc validates", crc8_dvb_s2(frame[2:-1]) == frame[-1])

print("\nFrame parsing")
buf = bytearray(frame)
got = list(parse_frames(buf))
check("one frame parsed", len(got) == 1)
check("type preserved", got[0][0] == TYPE_RC_CHANNELS)
check("payload preserved", unpack_channels(got[0][1]) == [CRSF_MID] * 16)
check("buffer drained", len(buf) == 0)

# Three frames back to back
buf = bytearray(frame * 3)
check("three frames parsed", len(list(parse_frames(buf))) == 3)

# Leading garbage before a valid frame
buf = bytearray(b"\x01\x02\x03" + frame)
got = list(parse_frames(buf))
check("resyncs past garbage", len(got) == 1 and got[0][0] == TYPE_RC_CHANNELS)

# Partial frame is retained for the next read
buf = bytearray(frame[:10])
got = list(parse_frames(buf))
check("partial frame yields nothing", len(got) == 0)
check("partial frame retained", len(buf) == 10)

# Split across two reads reassembles
buf = bytearray(frame[:10])
list(parse_frames(buf))
buf.extend(frame[10:])
got = list(parse_frames(buf))
check("split frame reassembles", len(got) == 1 and got[0][0] == TYPE_RC_CHANNELS)

# Corrupted CRC is flagged, not silently accepted
bad = bytearray(frame)
bad[-1] ^= 0xFF
got = list(parse_frames(bytearray(bad)))
check("bad crc flagged as None", len(got) == 1 and got[0][0] is None)

print("\nScaling and clamping")
check("centre -> 1500us", crsf_to_us(CRSF_MID) == 1500)
check("min -> ~988us", abs(crsf_to_us(CRSF_MIN) - 988) <= 1)
check("max -> ~2012us", abs(crsf_to_us(CRSF_MAX) - 2012) <= 1)
check("clamp below min", clamp_channel(-500) == CRSF_MIN)
check("clamp above max", clamp_channel(9999) == CRSF_MAX)
check("clamp passes valid", clamp_channel(1000) == 1000)
check("clamp returns int", isinstance(clamp_channel(1000.7), int))

print("\ncrsf_to_range")
check("min -> lo", crsf_to_range(CRSF_MIN, 1.0, 4.0) == 1.0)
check("max -> hi", crsf_to_range(CRSF_MAX, 1.0, 4.0) == 4.0)
check("mid -> midpoint", abs(crsf_to_range(CRSF_MID, 1.0, 4.0) - 2.5) < 0.01)
check("clamps below min", crsf_to_range(0, 1.0, 4.0) == 1.0)
check("clamps above max", crsf_to_range(9999, 1.0, 4.0) == 4.0)

print()
if failures:
    print(f"{len(failures)} FAILED: {', '.join(failures)}")
    sys.exit(1)
print("All tests passed.")
