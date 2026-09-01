"""Reliable XRCE streams: HEARTBEAT, ACKNACK and bounded retransmission.

Best-effort is right for streaming sensor data -- a dropped IMU sample at
50 Hz genuinely does not matter. It is wrong for a command that has to
arrive: an e-stop, a mode change, a configuration write.

Reliability is therefore **opt-in per publisher**, because it costs real RAM
and that cost should be paid deliberately.

Memory cost
-----------
An output reliable stream costs roughly::

    max_buffered * (MTU + ~40 bytes of dict overhead)

With the default ``max_buffered=8`` and a 512-byte MTU that is about **4.4 KB**
per reliable stream. On a Pico 2 W (520 KB SRAM) that is nothing. On a Pico W
with ~150-190 KB of heap it is worth counting, and it is why the default is 8
rather than the 32 a desktop implementation would pick.

The buffer is **hard-bounded**. An unbounded retransmit queue is an
out-of-memory bug waiting for a bad network day; when the window is full,
:meth:`OutputReliableStream.add` returns ``False`` and the caller decides
whether to block, drop or raise.
"""

import struct

# Sequence numbers are 16-bit and wrap. Comparisons must be modular: a naive
# ``a < b`` breaks catastrophically at the wrap point, once every 65536
# messages -- about 20 minutes at 50 Hz.
SEQNUM_MAX = 0xFFFF
_HALF = 0x8000


def seq_lt(a, b):
    """True if ``a`` precedes ``b`` in modular sequence order."""
    return ((b - a) & SEQNUM_MAX) != 0 and ((b - a) & SEQNUM_MAX) < _HALF


def seq_le(a, b):
    return a == b or seq_lt(a, b)


def seq_add(a, n):
    return (a + n) & SEQNUM_MAX


class OutputReliableStream:
    """Buffers sent datagrams until the Agent acknowledges them."""

    def __init__(self, stream_id, max_buffered=8, heartbeat_ms=200,
                 max_retries=10):
        self.stream_id = stream_id
        self.max_buffered = max_buffered
        self.heartbeat_ms = heartbeat_ms
        self.max_retries = max_retries
        self.next_seq = 0
        self._buf = {}       # seq -> datagram bytes
        self._tries = {}     # seq -> retransmit count
        self.sent = 0
        self.retransmits = 0
        self.dropped = 0
        self.last_heartbeat = 0

    # -- window ------------------------------------------------------------

    def full(self):
        return len(self._buf) >= self.max_buffered

    def pending(self):
        return len(self._buf)

    def take_seq(self):
        s = self.next_seq
        self.next_seq = seq_add(s, 1)
        return s

    def add(self, seq, datagram):
        """Remember a datagram for possible retransmission.

        Returns ``False`` if the window is full, rather than growing without
        bound.
        """
        if len(self._buf) >= self.max_buffered:
            return False
        self._buf[seq] = datagram
        self._tries[seq] = 0
        self.sent += 1
        return True

    # -- acknowledgement ---------------------------------------------------

    def on_acknack(self, first_unacked, bitmap):
        """Apply an ACKNACK; return the datagrams that need resending.

        Everything before ``first_unacked`` is acknowledged and dropped. Bits
        set in ``bitmap`` mark sequence numbers the Agent is missing, counting
        up from ``first_unacked``.
        """
        for s in list(self._buf.keys()):
            if seq_lt(s, first_unacked):
                self._buf.pop(s, None)
                self._tries.pop(s, None)

        resend = []
        for bit in range(16):
            if bitmap & (1 << bit):
                s = seq_add(first_unacked, bit)
                dg = self._buf.get(s)
                if dg is None:
                    continue
                n = self._tries.get(s, 0) + 1
                if n > self.max_retries:
                    # Give up on this one rather than resending for ever.
                    self._buf.pop(s, None)
                    self._tries.pop(s, None)
                    self.dropped += 1
                    continue
                self._tries[s] = n
                self.retransmits += 1
                resend.append(dg)
        return resend

    def heartbeat_range(self):
        """``(first, last)`` unacked sequence numbers, or ``None`` if idle."""
        if not self._buf:
            return None
        keys = list(self._buf.keys())
        first = keys[0]
        last = keys[0]
        for k in keys[1:]:
            if seq_lt(k, first):
                first = k
            if seq_lt(last, k):
                last = k
        return (first, last)


class InputReliableStream:
    """Tracks what we have received so we can ACKNACK the Agent's heartbeats."""

    def __init__(self, stream_id, window=16):
        self.stream_id = stream_id
        self.window = window
        self.first_unacked = 0
        self._received = set()

    def on_data(self, seq):
        """Record a received sequence number; True if it is new and in order."""
        if seq_lt(seq, self.first_unacked):
            return False  # already delivered
        self._received.add(seq)
        while self.first_unacked in self._received:
            self._received.discard(self.first_unacked)
            self.first_unacked = seq_add(self.first_unacked, 1)
        return True

    def compute_acknack(self):
        """``(first_unacked, bitmap)`` naming what we are still missing."""
        bitmap = 0
        for bit in range(16):
            s = seq_add(self.first_unacked, bit)
            if s not in self._received:
                bitmap |= (1 << bit)
        return self.first_unacked, bitmap


def pack_heartbeat(first, last, stream_id):
    return struct.pack("<HHB", first, last, stream_id)


def unpack_heartbeat(payload):
    if len(payload) < 5:
        return None
    first, last, sid = struct.unpack_from("<HHB", payload, 0)
    return first, last, sid


def pack_acknack(first_unacked, bitmap, stream_id):
    # The bitmap goes out most-significant byte first, unlike almost
    # everything else on the wire.
    return struct.pack("<H", first_unacked) + bytes(
        ((bitmap >> 8) & 0xFF, bitmap & 0xFF)
    ) + bytes((stream_id,))


def unpack_acknack(payload):
    if len(payload) < 5:
        return None
    first = struct.unpack_from("<H", payload, 0)[0]
    bitmap = (payload[2] << 8) | payload[3]
    sid = payload[4]
    return first, bitmap, sid
