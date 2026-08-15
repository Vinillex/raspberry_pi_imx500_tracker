"""
CRSF protocol primitives.

Pure functions only - no serial I/O, no threads, no global state.
This module can be imported and unit tested on any machine.
"""

from config import CRSF_SYNC, CRSF_MIN, CRSF_MID, CRSF_MAX


def crc8_dvb_s2(data: bytes) -> int:
    """CRC-8/DVB-S2: polynomial 0xD5, init 0x00."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def unpack_channels(payload: bytes):
    """22 bytes -> list of 16 channels, 11 bits each, LSB-first."""
    acc = 0
    acc_bits = 0
    channels = []
    for byte in payload:
        acc |= byte << acc_bits
        acc_bits += 8
        while acc_bits >= 11:
            channels.append(acc & 0x7FF)
            acc >>= 11
            acc_bits -= 11
    return channels[:16]


def pack_channels(channels) -> bytes:
    """List of 16 channels -> 22 bytes. Exact inverse of unpack_channels."""
    acc = 0
    acc_bits = 0
    out = bytearray()
    for ch in list(channels)[:16]:
        acc |= (int(ch) & 0x7FF) << acc_bits
        acc_bits += 11
        while acc_bits >= 8:
            out.append(acc & 0xFF)
            acc >>= 8
            acc_bits -= 8
    if acc_bits:
        out.append(acc & 0xFF)
    return bytes(out[:22])


def build_frame(ftype: int, payload: bytes) -> bytes:
    """Assemble a complete CRSF frame with correct length byte and CRC."""
    body = bytes([ftype]) + payload
    return bytes([CRSF_SYNC, len(body) + 1]) + body + bytes([crc8_dvb_s2(body)])


def parse_frames(buf: bytearray):
    """
    Consume complete, CRC-valid frames from `buf` (modified in place).

    Yields (frame_type, payload, raw_frame_bytes).
    Invalid frames are dropped and counted by the caller if desired.
    Leaves any partial trailing frame in the buffer for the next call.
    """
    while True:
        i = buf.find(bytes([CRSF_SYNC]))
        if i == -1:
            buf.clear()
            return
        if i > 0:
            del buf[:i]
        if len(buf) < 2:
            return

        length = buf[1]
        total = 2 + length

        if length < 2 or length > 62:
            del buf[0]          # bogus length, resync
            continue
        if len(buf) < total:
            return              # wait for the rest

        frame = bytes(buf[:total])
        del buf[:total]

        ftype = frame[2]
        payload = frame[3:total - 1]
        if crc8_dvb_s2(frame[2:total - 1]) != frame[total - 1]:
            yield (None, None, frame)   # signals a CRC error
            continue

        yield (ftype, payload, frame)


def crsf_to_us(value: int) -> int:
    """Raw CRSF (172-1811) -> servo microseconds (988-2012)."""
    return round((value - CRSF_MID) * 5 / 8 + 1500)


def clamp_channel(value) -> int:
    """Clamp any number into the valid CRSF range and return an int."""
    return int(max(CRSF_MIN, min(CRSF_MAX, value)))


def crsf_to_range(value: int, lo: float, hi: float) -> float:
    """Map a raw CRSF channel value (CRSF_MIN-CRSF_MAX) onto [lo, hi],
    clamped at both ends. value=CRSF_MIN -> lo, value=CRSF_MAX -> hi."""
    frac = (value - CRSF_MIN) / (CRSF_MAX - CRSF_MIN)
    frac = max(0.0, min(1.0, frac))
    return lo + frac * (hi - lo)
