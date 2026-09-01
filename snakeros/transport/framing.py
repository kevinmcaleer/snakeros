"""HDLC-style stream framing for XRCE over byte streams.

Serial is a byte stream with no message boundaries, so XRCE frames each
message:

    0x7E | stuffed( src, dst, len_lo, len_hi, payload..., crc_lo, crc_hi )

Everything after the begin flag is byte-stuffed: ``0x7E`` becomes
``0x7D 0x5E`` and ``0x7D`` becomes ``0x7D 0x5D`` (escape byte, then the value
XOR ``0x20``).

The CRC is **CRC-16/ARC** -- reflected polynomial ``0xA001``, initial value 0
-- computed over the **payload only**, not the header.

Note that eProsima's own documentation states the polynomial as
``x^16 + x^12 + x^5 + 1`` (CCITT, 0x1021). That is wrong: the table in
``stream_framing_protocol.c`` is unambiguously CRC-16/ARC. The code is the
authority here, and an implementation that trusted the prose would produce
frames the Agent silently discards.
"""

BEGIN_FLAG = 0x7E
ESC_FLAG = 0x7D
XOR_FLAG = 0x20

_CRC_TABLE = None


def _table():
    """Build the CRC-16/ARC table on first use.

    Generated rather than embedded: 512 bytes of table is worth having on a
    board that uses serial, and worth *not* having on one that does not.
    """
    global _CRC_TABLE
    if _CRC_TABLE is None:
        t = []
        for i in range(256):
            c = i
            for _ in range(8):
                c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
            t.append(c)
        _CRC_TABLE = t
    return _CRC_TABLE


def crc16(data, crc=0):
    t = _table()
    for b in data:
        crc = (crc >> 8) ^ t[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def _stuff(out, octet):
    if octet == BEGIN_FLAG or octet == ESC_FLAG:
        out.append(ESC_FLAG)
        out.append(octet ^ XOR_FLAG)
    else:
        out.append(octet)


def encode_frame(payload, src=0, dst=0):
    """Wrap a payload in a framed, byte-stuffed message."""
    n = len(payload)
    out = bytearray()
    out.append(BEGIN_FLAG)
    _stuff(out, src & 0xFF)
    _stuff(out, dst & 0xFF)
    _stuff(out, n & 0xFF)
    _stuff(out, (n >> 8) & 0xFF)
    for b in payload:
        _stuff(out, b)
    crc = crc16(payload)
    _stuff(out, crc & 0xFF)
    _stuff(out, (crc >> 8) & 0xFF)
    return bytes(out)


class FrameParser:
    """Incremental unstuffing parser for a serial byte stream.

    Fed arbitrary chunks; yields complete, CRC-checked payloads. Resynchronises
    on the next begin flag after any error, which is what makes it survive line
    noise and a half-written frame at startup.

    Consumed bytes are tracked with an offset rather than removed, because
    MicroPython's ``bytearray`` supports neither slice deletion nor ``del``.
    The buffer is compacted once the offset gets large.
    """

    def __init__(self, max_payload=2048):
        self.max_payload = max_payload
        self.buf = bytearray()
        self.start = 0
        self.crc_errors = 0
        self.resyncs = 0

    def _compact(self):
        if self.start:
            self.buf = bytearray(self.buf[self.start:])
            self.start = 0

    def feed(self, data):
        """Add bytes; return a list of complete payloads."""
        if data:
            self.buf.extend(data)
        out = []
        while True:
            frame = self._try_one()
            if frame is None:
                break
            if frame is not False:
                out.append(frame)
        if self.start > 512:
            self._compact()
        return out

    def _try_one(self):
        buf = self.buf
        n = len(buf)
        i = self.start

        # find the begin flag
        while i < n and buf[i] != BEGIN_FLAG:
            i += 1
            self.resyncs += 1
        if i >= n:
            self.start = n
            return None
        flag_at = i

        vals = bytearray()
        j = flag_at + 1
        esc = False
        while j < n:
            b = buf[j]
            if b == BEGIN_FLAG:
                break  # a new frame started; this one was truncated
            if esc:
                vals.append(b ^ XOR_FLAG)
                esc = False
            elif b == ESC_FLAG:
                esc = True
            else:
                vals.append(b)
            j += 1
            if len(vals) >= 4:
                payload_len = vals[2] | (vals[3] << 8)
                if payload_len > self.max_payload:
                    # implausible length: skip this flag and resync
                    self.start = flag_at + 1
                    self.resyncs += 1
                    return False
                need = 4 + payload_len + 2
                if len(vals) == need:
                    self.start = j
                    payload = bytes(vals[4:4 + payload_len])
                    got = vals[need - 2] | (vals[need - 1] << 8)
                    if crc16(payload) != got:
                        self.crc_errors += 1
                        return False
                    return payload

        if j < n and buf[j] == BEGIN_FLAG:
            # truncated frame; drop it and try from the next flag
            self.start = j
            self.resyncs += 1
            return False
        self.start = flag_at
        return None  # need more bytes
