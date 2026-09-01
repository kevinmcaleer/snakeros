"""SnakeROS benchmarks: heap cost, encode speed, publish rate, timer jitter.

Runs unmodified on the MicroPython Unix port and on a board::

    micropython tests/bench.py                 # encode/heap only, no Agent
    micropython tests/bench.py 192.168.1.10    # adds live publish-rate tests

**Unix-port numbers are not board numbers.** A desktop has 64-bit pointers,
a vastly faster CPU and effectively unlimited heap; treat host results as a
regression check and a relative profile, and quote only real hardware
numbers for the memory and rate tables.
"""

import gc
import sys
import time

sys.path.insert(0, ".")

AGENT = sys.argv[1] if len(sys.argv) > 1 else None

_ticks_us = getattr(time, "ticks_us", None)
if _ticks_us is None:
    def _us():
        return int(time.time() * 1000000)

    def _dus(a, b):
        return a - b
else:
    def _us():
        return time.ticks_us()

    def _dus(a, b):
        return time.ticks_diff(a, b)


def heap():
    gc.collect()
    return gc.mem_free()


def measure_heap(fn):
    before = heap()
    obj = fn()
    after = heap()
    return before - after, obj


def bar(label, value, unit, width=42):
    print("  {:<34} {:>10.1f} {}".format(label, value, unit))


print("=" * 62)
print("SnakeROS benchmarks")
print("  platform:", sys.implementation)
print("=" * 62)

# -- heap: import cost ---------------------------------------------------

print("\n-- heap cost of imports --")
base = heap()
import snakeros  # noqa: E402
core = base - heap()
print("  {:<34} {:>10,} bytes".format("snakeros core", core))

for pack in ("builtin_interfaces", "std_msgs", "geometry_msgs",
             "sensor_msgs", "nav_msgs", "std_srvs"):
    b = heap()
    __import__("snakeros.msg." + pack, None, None, (pack,))
    print("  {:<34} {:>10,} bytes".format("msg." + pack, b - heap()))

from snakeros.msg.std_msgs import String  # noqa: E402
from snakeros.msg.geometry_msgs import Twist, Vector3, PoseStamped  # noqa: E402
from snakeros.msg.sensor_msgs import Imu, JointState, LaserScan  # noqa: E402
from snakeros.msg.nav_msgs import Odometry  # noqa: E402

# -- heap: per message instance ------------------------------------------

print("\n-- heap cost per message instance --")
for name, cls in (("std_msgs/String", String), ("geometry_msgs/Twist", Twist),
                  ("sensor_msgs/Imu", Imu), ("nav_msgs/Odometry", Odometry)):
    used, _obj = measure_heap(cls)
    print("  {:<34} {:>10,} bytes".format(name, used))

# -- CDR encode / decode speed -------------------------------------------

print("\n-- CDR encode speed (per message) --")


def timeit(fn, n):
    gc.collect()
    t0 = _us()
    for _ in range(n):
        fn()
    return _dus(_us(), t0) / n


scan = LaserScan()
scan.ranges = [1.0] * 100
js = JointState()
js.name = ["left_wheel", "right_wheel"]
js.position = [0.0, 0.0]
js.velocity = [0.0, 0.0]

cases = [
    ("std_msgs/String (17 ch)", String(data="hello from a Pico"), 2000),
    ("geometry_msgs/Vector3", Vector3(), 2000),
    ("geometry_msgs/Twist", Twist(), 2000),
    ("geometry_msgs/PoseStamped", PoseStamped(), 1000),
    ("sensor_msgs/Imu", Imu(), 1000),
    ("sensor_msgs/JointState (2)", js, 1000),
    ("sensor_msgs/LaserScan (100)", scan, 300),
    ("nav_msgs/Odometry", Odometry(), 500),
]
encoded = {}
for name, msg, n in cases:
    us = timeit(msg.serialize, n)
    encoded[name] = msg.serialize()
    print("  {:<34} {:>8.1f} us   {:>5} bytes   {:>7.0f} msg/s".format(
        name, us, len(encoded[name]), 1000000.0 / us if us else 0))

print("\n-- CDR decode speed (per message) --")
for name, msg, n in cases:
    cls = type(msg)
    blob = encoded[name]
    us = timeit(lambda c=cls, b=blob: c.deserialize(b), n)
    print("  {:<34} {:>8.1f} us   {:>7.0f} msg/s".format(
        name, us, 1000000.0 / us if us else 0))

# -- fast path benefit ----------------------------------------------------

print("\n-- struct fast path --")
print("  {:<34} {}".format("Vector3 compiled format", Vector3._fast()))
print("  {:<34} {}".format("Twist (nested, no fast path)", Twist._fast()))

# -- live tests -----------------------------------------------------------

if AGENT:
    from snakeros import Node

    print("\n-- live publish rate (Agent at {}) --".format(AGENT))
    b = heap()
    node = Node("snakeros_bench", agent=AGENT)
    print("  {:<34} {:>10,} bytes".format("Node + session + participant", b - heap()))

    b = heap()
    pub = node.create_publisher(Twist, "bench_twist")
    print("  {:<34} {:>10,} bytes".format("one publisher", b - heap()))

    b = heap()
    node.create_subscription(Twist, "bench_sub", lambda m: None)
    print("  {:<34} {:>10,} bytes".format("one subscription", b - heap()))

    for label, cls, topic in (("Twist", Twist, "bench_twist"),):
        msg = cls()
        n = 500
        gc.collect()
        t0 = _us()
        for _ in range(n):
            pub.publish(msg)
        us = _dus(_us(), t0) / n
        print("  {:<34} {:>8.1f} us   {:>7.0f} msg/s".format(
            "publish " + label + " (no spin)", us, 1000000.0 / us if us else 0))

    # timer jitter at a nominal 50 Hz
    print("\n-- timer jitter at 50 Hz --")
    fires = []
    node.create_timer(0.02, lambda: fires.append(_us()))
    t0 = time.time()
    while time.time() - t0 < 5:
        node.spin_once(2)
    if len(fires) > 2:
        deltas = [_dus(fires[i + 1], fires[i]) for i in range(len(fires) - 1)]
        deltas.sort()
        mean = sum(deltas) / len(deltas)
        print("  {:<34} {:>8.0f} us".format("target period", 20000))
        print("  {:<34} {:>8.0f} us".format("mean period", mean))
        print("  {:<34} {:>8.0f} us".format("median", deltas[len(deltas) // 2]))
        print("  {:<34} {:>8.0f} us".format("p95", deltas[int(len(deltas) * 0.95)]))
        print("  {:<34} {:>8.0f} us".format("worst", deltas[-1]))
        print("  {:<34} {:>8d}".format("fires in 5 s", len(fires)))
    node.destroy()
else:
    print("\n(pass an Agent address to add live publish-rate and jitter tests)")

print("\n" + "=" * 62)
print("free heap at exit: {:,} bytes".format(heap()))
