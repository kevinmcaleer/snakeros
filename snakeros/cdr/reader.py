"""CDR (XCDR1) deserialisation.

Mirrors :mod:`snakeros.cdr.writer`, with the same origin-relative alignment
rule. Every read is bounds-checked: a truncated or corrupt payload raises
:class:`~snakeros.errors.CDRTruncated` rather than reading past the end of the
buffer or allocating something enormous off a corrupt length field.
"""

import struct

from ..errors import CDRTruncated, CDRError

# Refuse to allocate more than this off a single length field. A corrupt
# uint32 would otherwise ask for up to 4 GB, which on a microcontroller is an
# instant hard fault rather than a catchable error.
MAX_ALLOC = 1 << 20


class CDRReader:
    """Reads XCDR1 from a buffer, honouring the encapsulation endianness flag."""

    __slots__ = ("buf", "pos", "_pfx", "little_endian")

    def __init__(self, buf, little_endian=True):
        self.buf = buf
        self.pos = 0
        self.little_endian = little_endian
        self._pfx = "<" if little_endian else ">"

    # -- framing -----------------------------------------------------------

    def remaining(self):
        return len(self.buf) - self.pos

    def align(self, size):
        self.pos += (-self.pos) % size

    def encapsulation(self):
        """Consume a 4-byte encapsulation header and adopt its endianness."""
        if self.remaining() < 4:
            raise CDRTruncated("no room for encapsulation header")
        b = self.buf
        p = self.pos
        # bytes 0-1 are the representation id; bit 0 of byte 1 selects LE
        self.little_endian = bool(b[p + 1] & 0x01)
        self._pfx = "<" if self.little_endian else ">"
        self.pos = p + 4

    def _need(self, n):
        if self.pos + n > len(self.buf):
            raise CDRTruncated(
                "need {} bytes at offset {}, only {} left".format(
                    n, self.pos, self.remaining()
                )
            )

    def _unpack(self, fmt, size, align):
        if align > 1:
            self.align(align)
        self._need(size)
        v = struct.unpack_from(self._pfx + fmt, self.buf, self.pos)[0]
        self.pos += size
        return v

    # -- primitives --------------------------------------------------------

    def u8(self):
        self._need(1)
        v = self.buf[self.pos]
        self.pos += 1
        return v

    def i8(self):
        return self._unpack("b", 1, 1)

    def boolean(self):
        return self.u8() != 0

    def char(self):
        return self.u8()

    def u16(self):
        return self._unpack("H", 2, 2)

    def i16(self):
        return self._unpack("h", 2, 2)

    def u32(self):
        return self._unpack("I", 4, 4)

    def i32(self):
        return self._unpack("i", 4, 4)

    def u64(self):
        return self._unpack("Q", 8, 8)

    def i64(self):
        return self._unpack("q", 8, 8)

    def f32(self):
        return self._unpack("f", 4, 4)

    def f64(self):
        return self._unpack("d", 8, 8)

    # -- compound ----------------------------------------------------------

    def string(self):
        n = self.u32()
        if n > MAX_ALLOC:
            raise CDRError("implausible string length {}".format(n))
        if n == 0:
            return ""
        self._need(n)
        # n includes the trailing null
        s = bytes(self.buf[self.pos:self.pos + n - 1])
        self.pos += n
        return s.decode("utf-8")

    def raw(self, n):
        self._need(n)
        v = bytes(self.buf[self.pos:self.pos + n])
        self.pos += n
        return v

    def byte_array(self, n):
        return self.raw(n)

    def byte_seq(self):
        n = self.u32()
        if n > MAX_ALLOC:
            raise CDRError("implausible sequence length {}".format(n))
        return self.raw(n)

    def seq_len(self):
        n = self.u32()
        if n > MAX_ALLOC:
            raise CDRError("implausible sequence length {}".format(n))
        return n
