"""Long-running soak test: leaks, fragmentation and sustained rate.

Heap exhaustion fails loudly. **Fragmentation does not** -- it degrades
slowly, over tens of minutes, and is the failure mode that turns up after a
robot has been running all afternoon. That is what this looks for.

    micropython -X heapsize=190K tests/soak.py 127.0.0.1 1800

Run it under a constrained heap to model a real board:
190K approximates a Pico W, 400K a Pico 2 W.
"""

# Note: MicroPython's print() has no flush= keyword, so output is
# unbuffered by default rather than explicitly flushed.
import gc
import sys
import time

sys.path.insert(0, ".")

from snakeros import Node
from snakeros.msg.geometry_msgs import Twist
from snakeros.msg.sensor_msgs import Imu
from snakeros.msg.std_msgs import String

AGENT = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 1800.0
# Each client needs a distinct XRCE key. Two clients sharing one fight over
# entities on the Agent and the second fails to create a participant -- which
# is exactly what happens if you flash the same firmware to two boards.
KEY = int(sys.argv[3], 0) if len(sys.argv) > 3 else 0xAABBCCDD
NAME = sys.argv[4] if len(sys.argv) > 4 else "snakeros_soak"
REPORT_S = 60.0


def free():
    gc.collect()
    return gc.mem_free()


gc.collect()
total = gc.mem_free() + gc.mem_alloc()
print("soak: %d s, heap total %d bytes" % (SECONDS, total))

node = Node(NAME, agent=AGENT, key=KEY)
pub_imu = node.create_publisher(Imu, NAME + "_imu")
pub_str = node.create_publisher(String, NAME + "_str")
received = [0]
node.create_subscription(Twist, NAME + "_cmd", lambda m: received.__setitem__(0, received[0] + 1))

imu = Imu()
imu.header.frame_id = "imu_link"
imu.orientation_covariance = [0.01] * 9

baseline = free()
print("baseline free after setup: %d" % baseline)
print("")
print("  %8s %10s %10s %8s %8s %8s" % ("elapsed", "free", "drift", "pub", "recv", "min_free"))

start = time.time()
last_report = start
n = 0
min_free = baseline
worst_drift = 0

while time.time() - start < SECONDS:
    imu.header.stamp.sec = int(time.time())
    imu.angular_velocity.z = (n % 100) / 100.0
    pub_imu.publish(imu)
    # a varying-length string exercises the allocator rather than reusing
    # one convenient size for ever -- fragmentation needs varied sizes
    pub_str.publish(String(data="soak " * (1 + n % 12)))
    node.spin_once(5)
    n += 1

    now = time.time()
    if now - last_report >= REPORT_S:
        last_report = now
        f = free()
        drift = baseline - f
        if f < min_free:
            min_free = f
        if drift > worst_drift:
            worst_drift = drift
        print("  %7.0fs %10d %10d %8d %8d %8d" % (
            now - start, f, drift, n, received[0], min_free))
    time.sleep(0.02)

final = free()
print("")
print("RESULT")
print("  published        %d" % n)
print("  received         %d" % received[0])
print("  baseline free    %d" % baseline)
print("  final free       %d" % final)
print("  net drift        %d bytes" % (baseline - final))
print("  worst drift      %d bytes" % worst_drift)
print("  lowest free      %d bytes" % min_free)
verdict = "STABLE" if abs(baseline - final) < 4096 else "DRIFTING"
print("  verdict          %s" % verdict)
node.destroy()
