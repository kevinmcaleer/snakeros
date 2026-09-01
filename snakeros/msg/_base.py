"""Message type system.

A message is described by a compact tuple-of-tuples schema rather than by
per-field Python objects, because on a Pico W the per-type overhead is the
thing that decides whether a program fits at all.

Where a message is made entirely of fixed-size primitives, the schema is
compiled once into a single ``struct`` format string (with explicit CDR
padding baked in) so serialising costs one ``struct.pack`` rather than a
Python-level loop over fields. That fast path covers most of the small,
high-rate messages a robot actually publishes -- Twist, Vector3, Quaternion,
Point.

Field type codes
----------------
=====  ==================  ====
code   ROS type            size
=====  ==================  ====
``b``  bool                1
``o``  byte / uint8        1
``1``  int8                1
``c``  char                1
``s``  int16               2
``S``  uint16              2
``i``  int32               4
``I``  uint32              4
``l``  int64               8
``L``  uint64              8
``f``  float32             4
``d``  float64             8
``T``  string              var
=====  ==================  ====

Composite fields are tuples: ``('a', elem, n)`` a fixed array, ``('q', elem)``
an unbounded sequence, ``('q', elem, bound)`` a bounded one. A nested message
is the message class itself.
"""

import struct

from ..cdr import CDRWriter, CDRReader
from ..errors import MessageDefinitionError

# code -> (struct char, size, default)
# MicroPython's struct has no '?' (bool) or 'c' (char) typecode, so both are
# packed as unsigned bytes and converted at the edges. Diverging from CPython
# here would work on the Unix port and fail on the board, which is the worst
# place to find out.
_PRIM = {
    "b": ("B", 1, False),
    "o": ("B", 1, 0),
    "1": ("b", 1, 0),
    "c": ("B", 1, 0),
    "s": ("h", 2, 0),
    "S": ("H", 2, 0),
    "i": ("i", 4, 0),
    "I": ("I", 4, 0),
    "l": ("q", 8, 0),
    "L": ("Q", 8, 0),
    "f": ("f", 4, 0.0),
    "d": ("d", 8, 0.0),
}

_WRITE = {
    "b": "boolean", "o": "u8", "1": "i8", "c": "u8",
    "s": "i16", "S": "u16", "i": "i32", "I": "u32",
    "l": "i64", "L": "u64", "f": "f32", "d": "f64",
    "T": "string",
}


def _is_msg(t):
    return isinstance(t, type) and issubclass(t, Msg)


def _default_for(t):
    if isinstance(t, str):
        if t == "T":
            return ""
        p = _PRIM.get(t)
        if p is None:
            raise MessageDefinitionError("unknown field code " + repr(t))
        return p[2]
    if isinstance(t, tuple):
        if t[0] == "a":
            n = t[2]
            return [_default_for(t[1]) for _ in range(n)]
        return []
    if _is_msg(t):
        return t()
    raise MessageDefinitionError("unsupported field type " + repr(t))


def _write_value(w, t, v):
    if isinstance(t, str):
        getattr(w, _WRITE[t])(v)
        return
    if isinstance(t, tuple):
        kind = t[0]
        elem = t[1]
        if kind == "q":
            w.seq_len(len(v))
        for item in v:
            _write_value(w, elem, item)
        return
    if _is_msg(t):
        v.serialize_into(w)
        return
    raise MessageDefinitionError("unsupported field type " + repr(t))


def _read_value(r, t):
    if isinstance(t, str):
        if t == "T":
            return r.string()
        return getattr(r, _WRITE[t])()
    if isinstance(t, tuple):
        kind = t[0]
        elem = t[1]
        n = t[2] if kind == "a" else r.seq_len()
        return [_read_value(r, elem) for _ in range(n)]
    if _is_msg(t):
        return t.deserialize_from(r)
    raise MessageDefinitionError("unsupported field type " + repr(t))


class Msg(object):
    """Base class for every ROS 2 message."""

    _package_ = ""
    _kind_ = "msg"
    _name_ = ""
    _fields_ = ()

    __slots__ = ()

    def __init__(self, **kw):
        for name, t in self._fields_:
            if name in kw:
                setattr(self, name, kw.pop(name))
            else:
                setattr(self, name, _default_for(t))
        if kw:
            raise TypeError(
                "{} has no field(s) {}".format(
                    self.__class__.__name__, ", ".join(kw)
                )
            )

    # -- identity ----------------------------------------------------------

    @classmethod
    def type_name(cls):
        """The DDS type name, e.g. ``std_msgs::msg::dds_::String_``."""
        return "{}::{}::dds_::{}_".format(cls._package_, cls._kind_, cls._name_)

    @classmethod
    def ros_name(cls):
        """The ROS 2 type name, e.g. ``std_msgs/msg/String``."""
        return "{}/{}/{}".format(cls._package_, cls._kind_, cls._name_)

    # -- fast path ---------------------------------------------------------

    @classmethod
    def _fast(cls):
        """Return a compiled ``struct`` format, or ``None`` if not applicable.

        Computed once and cached on the class. Only messages made entirely of
        fixed-size primitives qualify; anything with a string, sequence or
        nested message falls back to the field loop.

        CDR padding is baked into the format as ``x`` bytes, so a Twist costs
        one ``struct.pack`` instead of six aligned writes.
        """
        f = cls.__dict__.get("_fast_fmt_")
        if f is not None:
            return f[0]
        fmt = "<"
        off = 0
        ok = True
        bools = []
        for idx, (_name, t) in enumerate(cls._fields_):
            if not isinstance(t, str) or t == "T":
                ok = False
                break
            ch, size, _d = _PRIM[t]
            pad = (-off) % size
            if pad:
                fmt += "x" * pad
                off += pad
            fmt += ch
            off += size
            if t == "b":
                bools.append(idx)
        result = fmt if ok and cls._fields_ else None
        cls._fast_fmt_ = (result, tuple(bools))
        return result

    @classmethod
    def _fast_bools(cls):
        cls._fast()
        return cls.__dict__["_fast_fmt_"][1]

    # -- serialisation -----------------------------------------------------

    def serialize_into(self, w):
        fast = self._fast()
        if fast is not None:
            w.align(self._max_align())
            vals = [getattr(self, n) for n, _ in self._fields_]
            for i in self._fast_bools():
                vals[i] = 1 if vals[i] else 0
            w.raw(struct.pack(fast, *vals))
            return
        for name, t in self._fields_:
            _write_value(w, t, getattr(self, name))

    @classmethod
    def _max_align(cls):
        m = cls.__dict__.get("_max_align_")
        if m is not None:
            return m[0]
        best = 1
        for _n, t in cls._fields_:
            if isinstance(t, str) and t != "T":
                sz = _PRIM[t][1]
                if sz > best:
                    best = sz
        cls._max_align_ = (best,)
        return best

    def serialize(self):
        """Serialise to a standalone CDR blob (no encapsulation header).

        XRCE carries the payload without an encapsulation header -- the Agent
        adds one when forwarding to DDS. Alignment starts at zero here, which
        is what micro-ROS does too.
        """
        w = CDRWriter()
        self.serialize_into(w)
        return w.bytes()

    @classmethod
    def deserialize_from(cls, r):
        # MicroPython has no ``cls.__new__``; ``object.__new__`` works on
        # both and skips default construction, which matters for nested
        # messages -- otherwise every Vector3 in a Twist is built twice.
        obj = object.__new__(cls)
        fast = cls._fast()
        if fast is not None:
            r.align(cls._max_align())
            size = struct.calcsize(fast)
            vals = list(struct.unpack(fast, r.raw(size)))
            for i in cls._fast_bools():
                vals[i] = bool(vals[i])
            for (name, _t), v in zip(cls._fields_, vals):
                setattr(obj, name, v)
            return obj
        for name, t in cls._fields_:
            setattr(obj, name, _read_value(r, t))
        return obj

    @classmethod
    def deserialize(cls, data):
        return cls.deserialize_from(CDRReader(data))

    # -- niceties ----------------------------------------------------------

    def __eq__(self, other):
        if type(self) is not type(other):
            return NotImplemented
        for name, _t in self._fields_:
            if getattr(self, name) != getattr(other, name):
                return False
        return True

    def __ne__(self, other):
        r = self.__eq__(other)
        return r if r is NotImplemented else not r

    def __repr__(self):
        parts = []
        for name, _t in self._fields_:
            parts.append("{}={!r}".format(name, getattr(self, name)))
        return "{}({})".format(self.__class__.__name__, ", ".join(parts))
