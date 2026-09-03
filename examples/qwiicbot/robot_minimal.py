"""QwiicBot, cut down for a memory-tight board.

The full ``robot.py`` needs roughly 100 KB of Python heap once its message
packs are loaded. On an ESP32 that is a problem, and not for the reason it
first appears: MicroPython's GC heap on ESP32 **grows by claiming blocks from
the ESP-IDF heap**, which is the same heap lwIP allocates its network buffers
from. Grow the Python heap far enough and sends start failing with ENOMEM even
though ``gc.mem_free()`` looks healthy. Check the real figure with::

    import esp32
    print(esp32.idf_heap_info(esp32.HEAP_DATA))   # (total, free, largest, min)

``largest`` is what matters -- lwIP needs a contiguous block.

This version keeps the robot useful and drops everything that costs heap
without earning it:

* **no parameters** -- saves ``rcl_interfaces`` and six services
* **no services** -- saves ``std_srvs``
* **no IMU, face or buzzer topics** -- three fewer entities
* **smaller MTU** -- 256 rather than 512, halving the transport buffers

What is left: drive it with ``/cmd_vel``, read the distance sensor on
``/range``, and the obstacle veto and fail-safe still work.

    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.4}}'
    ros2 topic echo /range
"""

import gc
import sys
import time

# Collect aggressively rather than letting the heap grow. On ESP32 heap growth
# is permanent -- it never gives blocks back to the IDF heap -- so keeping it
# small is the only lever that helps lwIP.
gc.collect()
gc.threshold(gc.mem_alloc() + gc.mem_free() // 4)

from snakeros import Node                          # noqa: E402
from snakeros.msg.geometry_msgs import Twist       # noqa: E402
from snakeros.msg.sensor_msgs import Range         # noqa: E402

try:
    from hardware import Drive, Rangefinder
except ImportError:
    sys.path.insert(0, "examples/qwiicbot")
    from hardware import Drive, Rangefinder

CMD_TIMEOUT_S = 0.6
STOP_DISTANCE = 0.15
MAX_SPEED = 0.6
WHEEL_SEPARATION = 0.09


def _report_version():
    """Print the SnakeROS version, tolerating an older install.

    Deliberately defensive: a diagnostic must never be the thing that stops a
    robot starting. If the library on the device predates this helper, say so
    -- that mismatch is itself the most useful thing to know.
    """
    try:
        import snakeros

        print("[snakeros] version", getattr(snakeros, "__version__", "?"))
    except Exception as e:
        print("[snakeros] version unknown:", e)
    try:
        from snakeros.board import version_report  # noqa: F401
    except ImportError:
        print("[snakeros] NOTE: lib/snakeros is older than these example "
              "files -- re-copy build/mpy/snakeros to :lib/")


def main(agent="127.0.0.1", port=8888, mtu=256):
    _report_version()
    gc.collect()
    print("[minimal] python heap free:", gc.mem_free())
    try:
        import esp32
        print("[minimal] idf heap:", esp32.idf_heap_info(esp32.HEAP_DATA)[-1])
    except ImportError:
        pass

    drive = Drive()
    rng = Rangefinder(drive)

    node = Node("qwiicbot", agent=agent, port=port, mtu=mtu, key=0xC0FFEE02)
    pub = node.create_publisher(Range, "range")

    state = {"lin": 0.0, "ang": 0.0, "t": 0.0}

    def on_cmd(msg):
        state["lin"] = msg.linear.x
        state["ang"] = msg.angular.z
        state["t"] = time.time()

    node.create_subscription(Twist, "cmd_vel", on_cmd)

    msg = Range()
    msg.header.frame_id = "distance_sensor"
    msg.radiation_type = Range.INFRARED
    msg.field_of_view = 0.44
    msg.min_range = Rangefinder.MIN_M
    msg.max_range = Rangefinder.MAX_M

    def publish():
        msg.header.stamp.sec = int(time.time())
        msg.range = rng.read()
        pub.publish(msg)

    def control():
        if time.time() - state["t"] > CMD_TIMEOUT_S:
            drive.stop()
            return
        lin, ang = state["lin"], state["ang"]
        if lin > 0 and rng.read() < STOP_DISTANCE:
            lin = 0.0                       # veto forward only, never turns
        left = lin - ang * WHEEL_SEPARATION / 2.0
        right = lin + ang * WHEEL_SEPARATION / 2.0
        scale = max(1.0, abs(left), abs(right))
        drive.set(MAX_SPEED * left / scale, MAX_SPEED * right / scale)

    node.create_timer(0.1, publish)
    node.create_timer(0.05, control)

    gc.collect()
    print("[minimal] ready, heap free:", gc.mem_free())
    try:
        node.spin(10)
    finally:
        drive.stop()
        node.destroy()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1")
