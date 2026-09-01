"""Feasibility benchmark for high-bandwidth sensor data (LD06 lidar).

An LD06 at 10 Hz produces 450 points per revolution. Published as a
``sensor_msgs/LaserScan`` with Python lists -- which is what SnakeROS does
today -- that is expensive enough to be impractical on a microcontroller.

This measures the alternative: hold numeric arrays as ``array`` and exploit
the fact that **a CDR little-endian float32 sequence body is byte-identical to
an ``array('f')`` buffer.** After the uint32 length the elements pack
contiguously with no padding, so encoding is a length plus a memcpy rather
than a loop over 450 boxed floats.

    micropython tests/bench_bulk.py
"""

import gc
import sys
import time
from array import array

sys.path.insert(0, ".")

from snakeros.cdr import CDRWriter          # noqa: E402
from snakeros.msg.sensor_msgs import LaserScan  # noqa: E402

N = 450  # LD06 at 10 Hz: 4500 samples/s fixed rate / 10 Hz

_us = getattr(time, "ticks_us", None)
if _us is None:
    def us():
        return int(time.time() * 1e6)

    def dt(a, b):
        return a - b
else:
    def us():
        return time.ticks_us()

    def dt(a, b):
        return time.ticks_diff(a, b)


def timeit(fn, n=200):
    gc.collect()
    t0 = us()
    for _ in range(n):
        fn()
    return dt(us(), t0) / n


vals = [i * 0.001 for i in range(N)]
arr = array("f", vals)

print("=" * 62)
print("Bulk sensor data feasibility (%d-point scan)" % N)
print("=" * 62)

# -- the enabling claim ---------------------------------------------------

w = CDRWriter()
w.seq_len(N)
for v in vals:
    w.f32(v)
loop_bytes = w.bytes()

w2 = CDRWriter()
w2.seq_len(N)
w2.raw(memoryview(arr))
memcpy_bytes = w2.bytes()

print("\n-- claim: CDR float32[] body == array('f') buffer --")
print("  per-element loop : %d bytes" % len(loop_bytes))
print("  length + memcpy  : %d bytes" % len(memcpy_bytes))
print("  IDENTICAL        : %s" % (loop_bytes == memcpy_bytes))
if loop_bytes != memcpy_bytes:
    print("  !! the fast path is NOT valid on this platform")

# -- encode cost ----------------------------------------------------------

def enc_loop():
    w = CDRWriter()
    w.seq_len(N)
    for v in vals:
        w.f32(v)
    return w.bytes()


def enc_memcpy():
    w = CDRWriter()
    w.seq_len(N)
    w.raw(memoryview(arr))
    return w.bytes()


a = timeit(enc_loop)
b = timeit(enc_memcpy)
print("\n-- encode cost --")
print("  per-element loop : %8.1f us" % a)
print("  length + memcpy  : %8.1f us   (%.0fx faster)" % (b, a / b if b else 0))

scan = LaserScan()
scan.header.frame_id = "laser"
scan.ranges = vals
scan.intensities = vals
c = timeit(lambda: scan.serialize(), 50)
print("  full LaserScan today: %6.1f us  (%d bytes)" % (c, len(scan.serialize())))
print("  ...which at 10 Hz is %.2f%% of a second on THIS machine" % (c * 10 / 10000.0))

# -- heap -----------------------------------------------------------------

gc.collect()
base = gc.mem_free()
lst = [0.0] * N
for i in range(N):
    lst[i] = i * 0.001
gc.collect()
list_cost = base - gc.mem_free()
del lst
gc.collect()

base = gc.mem_free()
ar = array("f", bytes(4 * N))
for i in range(N):
    ar[i] = i * 0.001
gc.collect()
array_cost = base - gc.mem_free()

print("\n-- heap for one %d-element field --" % N)
print("  list of floats   : %6d bytes" % list_cost)
print("  array('f')       : %6d bytes   (%.1fx smaller)" % (
    array_cost, list_cost / max(array_cost, 1)))
print("  a LaserScan holds ranges AND intensities:")
print("    today          : %6d bytes" % (2 * list_cost))
print("    with arrays    : %6d bytes" % (2 * array_cost))

# -- LD06 decode ----------------------------------------------------------

raw = array("H", [1000 + (i % 5000) for i in range(N)])


def convert():
    out = array("f", bytes(4 * N))
    for i in range(N):
        out[i] = raw[i] * 0.001
    return out


d = timeit(convert)
print("\n-- LD06 decode: uint16 mm -> float32 m --")
print("  %d points        : %8.1f us  (%.3f us/point)" % (N, d, d / N))

print("\n" + "=" * 62)
print("projected per-scan cost with arrays: ~%.0f us (vs ~%.0f us today)" % (b + d, c))
print("NOTE: host numbers. A Pico is far slower -- scale accordingly, and")
print("      see docs/memory.md on why host figures are not board figures.")
