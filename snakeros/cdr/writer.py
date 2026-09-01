"""CDR (XCDR1) serialisation.

Alignment is relative to the *origin of this buffer*, not to any enclosing one.
That matters: micro-ROS hands the ROS message serialiser a fresh CDR buffer
starting at the payload, so a message's own alignment restarts at zero
regardless of where in the datagram it lands. Getting this wrong misplaces
every ``float64`` and produces garbage that decodes cleanly as the wrong
numbers -- the worst kind of bug.

See ``uxr_prepare_output_stream`` in Micro-XRCE-DDS-Client, which calls
``ucdr_init_buffer(ub, ub->iterator, ...)`` for exactly this reason.
"""

import struct

_LE = "<"
_BE = ">"


class CDRWriter:
    """Little-endian XCDR1 writer over a growable bytearray."""

    __slots__ = ("buf", "_le", "_pfx")

    def __init__(self, little_endian=True, initial=None):
        self.buf = bytearray() if initial is None else initial
        self._le = little_endian
        self._pfx = _LE if little_endian else _BE

    # -- framing -----------------------------------------------------------

    def __len__(self):
        return len(self.buf)

    def bytes(self):
        return bytes(self.buf)

    def align(self, size):
        """Pad to the next ``size`` boundary, relative to this buffer's start."""
        pad = (-len(self.buf)) % size
        if pad:
            self.buf.extend(b"\x00" * pad)

    def encapsulation(self):
        """Write the 4-byte CDR encapsulation header.

        ``0x0001`` is CDR_LE, ``0x0000`` CDR_BE, followed by a 2-byte options
        field. Only used where a standalone CDR blob is required; XRCE
        submessage payloads do not carry one.
        """
        self.buf.extend(b"\x00\x01\x00\x00" if self._le else b"\x00\x00\x00\x00")

    # -- primitives --------------------------------------------------------

    def u8(self, v):
        self.buf.append(v & 0xFF)

    def i8(self, v):
        self.buf.extend(struct.pack("b", v))

    def boolean(self, v):
        self.buf.append(1 if v else 0)

    def char(self, v):
        self.buf.append(v & 0xFF) if isinstance(v, int) else self.buf.extend(v[:1].encode())

    def u16(self, v):
        self.align(2)
        self.buf.extend(struct.pack(self._pfx + "H", v & 0xFFFF))

    def i16(self, v):
        self.align(2)
        self.buf.extend(struct.pack(self._pfx + "h", v))

    def u32(self, v):
        self.align(4)
        self.buf.extend(struct.pack(self._pfx + "I", v & 0xFFFFFFFF))

    def i32(self, v):
        self.align(4)
        self.buf.extend(struct.pack(self._pfx + "i", v))

    def u64(self, v):
        self.align(8)
        self.buf.extend(struct.pack(self._pfx + "Q", v & 0xFFFFFFFFFFFFFFFF))

    def i64(self, v):
        self.align(8)
        self.buf.extend(struct.pack(self._pfx + "q", v))

    def f32(self, v):
        self.align(4)
        self.buf.extend(struct.pack(self._pfx + "f", v))

    def f64(self, v):
        self.align(8)
        self.buf.extend(struct.pack(self._pfx + "d", v))

    # -- compound ----------------------------------------------------------

    def string(self, s):
        """CDR string: uint32 length *including* the null, bytes, then the null."""
        if s is None:
            s = ""
        if isinstance(s, str):
            s = s.encode("utf-8")
        self.u32(len(s) + 1)
        self.buf.extend(s)
        self.buf.append(0)

    def raw(self, b):
        """Append opaque bytes with no length prefix and no alignment."""
        self.buf.extend(b)

    def byte_array(self, b):
        """Fixed-size octet array: no length prefix."""
        self.buf.extend(b)

    def byte_seq(self, b):
        """Octet sequence: uint32 count then the bytes."""
        self.u32(len(b))
        self.buf.extend(b)

    def seq_len(self, n):
        self.u32(n)
