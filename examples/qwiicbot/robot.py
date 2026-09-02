"""QwiicBot -- a SMARS robot built entirely from Arduino Modulinos, on ROS 2.

The robot is a standard SMARS tracked chassis with its Arduino Uno bay
replaced by a Modulino holder. Five Modulinos give it five senses, chained
over Qwiic/I2C with no soldering:

===========  =====================  ==============================
Sense        Modulino               ROS 2
===========  =====================  ==============================
Move         Motors                 subscribes ``/cmd_vel``
See          Distance               publishes ``/range``
Measure      Movement (IMU)         publishes ``/imu/data``
Show         LED Matrix             subscribes ``/face``
Hear/Speak   Buzzer                 serves ``/beep``
===========  =====================  ==============================

The point of the demo: the hardware clicks together, the Modulino drivers
install themselves -- and SnakeROS puts the whole thing on a ROS 2 graph with
no bridge, no glue code and no C toolchain. Nothing is wired, and nothing is
configured.

Run it on a board::

    from robot import main
    main(agent="192.168.1.10", ssid="my-wifi", password="secret")

Or on a laptop, with simulated Modulinos and a real Agent::

    micropython examples/qwiicbot/robot.py 127.0.0.1

**No odometry.** SMARS's N20 gearmotors have no encoders, so there is no
honest ``/odom`` to publish and none is faked. The IMU gives orientation
rates; wheel odometry would need encoders the robot does not have.
"""

import sys
import time

# MicroPython already has "" (the working directory) on sys.path, so an
# installed SnakeROS -- and hardware.py sitting beside this file on a device --
# are both importable with no path juggling at all.
#
# Do NOT insert "." here. On a device that puts the working directory ahead of
# /lib, letting a stale or partial snakeros/ shadow the real install. It fails
# as:
#
#     ImportError: no module named 'snakeros.Node'
#
# which is confusing, because the shadowing package imports perfectly well and
# simply defines nothing. The capital N is the tell: a missing *name*. A
# lowercase 'snakeros.node' would mean a missing *module*.

try:
    from snakeros import Node
except ImportError:
    # Distinguish "not installed" from "shadowed by a stale copy" -- the raw
    # error ("no module named 'snakeros.Node'") suggests neither. Note the
    # probe must not raise inside its own except clause, or the shadow case
    # gets swallowed and misreported as "not installed".
    _sr = None
    try:
        import snakeros as _sr
    except ImportError:
        pass
    if _sr is not None:
        raise ImportError(
            "SnakeROS imported from %s but has no 'Node'. A stale or empty "
            "snakeros/ is shadowing your real install -- MicroPython searches "
            "the working directory before /lib, so the broken copy wins. "
            "Delete it, then retry. sys.path = %r"
            % (getattr(_sr, "__file__", "?"), sys.path))
    raise ImportError(
        "SnakeROS is not installed. On the device: "
        "import mip; mip.install('github:kevinmcaleer/snakeros'). "
        "sys.path = %r" % (sys.path,))
from snakeros.board import ResilientNode, connect_wifi, heap_report, preallocate  # noqa: E402
from snakeros.msg.geometry_msgs import Twist                       # noqa: E402
from snakeros.msg.sensor_msgs import Imu, Range                    # noqa: E402
from snakeros.msg.std_msgs import String                           # noqa: E402
from snakeros.msg.std_srvs import SetBool, Trigger                 # noqa: E402

def _find_hardware():
    """Import hardware.py, which must sit beside this file.

    Handles three layouts: a repo checkout (cwd is the repo root), a device
    with both files in the working directory, and `mpremote run`, where the
    script arrives on stdin so the working directory is / rather than wherever
    the file lives.
    """
    candidates = ["examples/qwiicbot", "/qwiicbot", "/lib/qwiicbot"]
    here = globals().get("__file__")
    if here and "/" in here:
        candidates.insert(0, here.rsplit("/", 1)[0])
    for path in candidates:
        if path not in sys.path:
            sys.path.insert(0, path)
        try:
            return __import__("hardware")
        except ImportError:
            sys.path.remove(path)
    raise ImportError(
        "hardware.py not found. It must sit beside robot.py -- copy both:\n"
        "    mpremote fs cp examples/qwiicbot/hardware.py :\n"
        "    mpremote fs cp examples/qwiicbot/robot.py :\n"
        "then run it from that directory. Looked in: %r" % (sys.path,))


_hw = _find_hardware()
Buzzer = _hw.Buzzer
Drive = _hw.Drive
Face = _hw.Face
ImuSensor = _hw.Imu
Rangefinder = _hw.Rangefinder

# Stop if no command arrives within this long. A robot that keeps driving on
# a stale command after the link drops is the failure that breaks things.
CMD_TIMEOUT_S = 0.6


class QwiicBot:
    def __init__(self, node):
        self.node = node
        self.drive = Drive()
        self.range = Rangefinder(self.drive)
        self.imu = ImuSensor()
        self.buzzer = Buzzer()
        self.face = Face()

        self.target_linear = 0.0
        self.target_angular = 0.0
        self._last_cmd = 0.0
        self._blocked = False
        self._auto_state = "drive"
        self._auto_until = 0.0

        # -- parameters, tunable live with ros2 param ----------------------
        node.declare_parameter("max_speed", 0.6, "motor effort ceiling (0-1)",
                               minimum=0.0, maximum=1.0)
        node.declare_parameter("wheel_separation", 0.09, "track width (m)",
                               minimum=0.01, maximum=1.0)
        node.declare_parameter("stop_distance", 0.15, "obstacle stop distance (m)",
                               minimum=0.02, maximum=2.0)
        node.declare_parameter("autonomous", False, "run the wander behaviour",
                               callback=self._on_autonomous)
        node.declare_parameter("publish_rate", 10.0, "sensor rate (Hz)",
                               minimum=1.0, maximum=50.0)

        # -- ROS interface -------------------------------------------------
        self.pub_range = node.create_publisher(Range, "range")
        self.pub_imu = node.create_publisher(Imu, "imu/data")
        node.create_subscription(Twist, "cmd_vel", self.on_cmd_vel)
        node.create_subscription(String, "face", self.on_face)
        node.create_service(Trigger, "beep", self.on_beep)
        node.create_service(SetBool, "autonomous", self.on_autonomous_srv)

        # reusable message objects: allocating per publish invites a GC pause
        # in the middle of the control loop
        self._range_msg = Range()
        self._range_msg.header.frame_id = "distance_sensor"
        self._range_msg.radiation_type = Range.INFRARED
        self._range_msg.field_of_view = 0.44          # ~25 deg, typical ToF cone
        self._range_msg.min_range = Rangefinder.MIN_M
        self._range_msg.max_range = Rangefinder.MAX_M

        self._imu_msg = Imu()
        self._imu_msg.header.frame_id = "imu_link"
        # No orientation estimate from a raw 6-axis IMU. ROS convention is to
        # set element 0 of the covariance to -1 to say so, rather than
        # publishing a made-up identity quaternion.
        self._imu_msg.orientation_covariance[0] = -1.0

        rate = node.get_parameter("publish_rate", 10.0)
        node.create_timer(1.0 / rate, self.publish_sensors)
        node.create_timer(0.05, self.control_step)

        self.face.show("happy")

    # -- ROS callbacks -----------------------------------------------------

    def on_cmd_vel(self, msg):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        self._last_cmd = time.time()

    def _express(self, face, sound=None):
        """Change expression and optionally sound, narrating it to the console.

        A demo that does things silently is hard to follow, and on real
        hardware this is the only way to see what the robot decided when the
        LED matrix is facing away from you.
        """
        if face and face != self.face.current:
            self.face.show(face)
            print("[qwiicbot] face -> %s" % face)
        if sound == "alarm":
            self.buzzer.alarm()
            print("[qwiicbot] buzzer: alarm")
        elif sound == "chirp":
            self.buzzer.chirp()
        elif sound == "off":
            self.buzzer.off()

    def on_face(self, msg):
        if not self.face.show(msg.data):
            print("[qwiicbot] unknown face %r, try: %s" % (
                msg.data, ", ".join(self.face.names())))

    def on_beep(self, _req):
        self._express(None, "chirp")
        print("[qwiicbot] beep requested")
        res = Trigger.Response()
        res.success = True
        res.message = "beeped"
        return res

    def on_autonomous_srv(self, req):
        self.node.set_parameter("autonomous", req.data)
        res = SetBool.Response()
        res.success = True
        res.message = "wandering" if req.data else "stopped"
        return res

    def _on_autonomous(self, value):
        if not value:
            self.drive.stop()
            self.face.show("happy")
        else:
            self._auto_state = "drive"
        return True

    # -- control -----------------------------------------------------------

    def control_step(self):
        if self.node.get_parameter("autonomous", False):
            self._wander()
            return

        # teleop: stop if commands have gone quiet
        if time.time() - self._last_cmd > CMD_TIMEOUT_S:
            if self.drive.left or self.drive.right:
                self.drive.stop()
            return
        self._apply(self.target_linear, self.target_angular)

    def _apply(self, linear, angular):
        """Differential-drive mixing, with an obstacle veto on forward motion."""
        sep = self.node.get_parameter("wheel_separation", 0.09)
        top = self.node.get_parameter("max_speed", 0.6)
        stop_d = self.node.get_parameter("stop_distance", 0.15)

        # The robot may always reverse or turn on the spot; only forward
        # motion is vetoed, or it would get stuck against a wall for ever.
        if linear > 0 and self.range.read() < stop_d:
            if not self._blocked:
                self._blocked = True
                print("[qwiicbot] obstacle at %.2f m -- forward vetoed" % self.range.read())
                self._express("startled", "alarm")
            linear = 0.0
        elif self._blocked:
            self._blocked = False
            print("[qwiicbot] path clear")
            self._express("happy", "off")

        left = linear - angular * sep / 2.0
        right = linear + angular * sep / 2.0
        scale = max(1.0, abs(left), abs(right))
        self.drive.set(top * left / scale, top * right / scale)

    def _wander(self):
        """The cold-open behaviour: drive, meet a wall, react, back off, turn."""
        now = time.time()
        top = self.node.get_parameter("max_speed", 0.6)
        stop_d = self.node.get_parameter("stop_distance", 0.15)

        if self._auto_state == "drive":
            if self.range.read() < stop_d:
                print("[qwiicbot] wander: wall at %.2f m -> back" % self.range.read())
                self._express("startled", "alarm")
                self.drive.stop()
                self._auto_state = "back"
                self._auto_until = now + 0.6
            else:
                self.drive.set(top, top)
        elif self._auto_state == "back":
            self.drive.set(-top * 0.7, -top * 0.7)
            if now >= self._auto_until:
                print("[qwiicbot] wander: back -> turn")
                self._auto_state = "turn"
                self._auto_until = now + 0.5
        elif self._auto_state == "turn":
            self.drive.set(top * 0.7, -top * 0.7)
            if now >= self._auto_until:
                print("[qwiicbot] wander: turn -> drive")
                self._express("happy", "off")
                self._auto_state = "drive"

    def stop(self):
        self.drive.stop()
        self.buzzer.off()
        self.face.show("sleepy")

    # -- publishing --------------------------------------------------------

    def publish_sensors(self):
        secs = int(time.time())

        r = self._range_msg
        r.header.stamp.sec = secs
        r.range = self.range.read()
        self.pub_range.publish(r)

        ax, ay, az, gx, gy, gz = self.imu.read()
        m = self._imu_msg
        m.header.stamp.sec = secs
        m.linear_acceleration.x = ax
        m.linear_acceleration.y = ay
        m.linear_acceleration.z = az
        m.angular_velocity.x = gx
        m.angular_velocity.y = gy
        m.angular_velocity.z = gz
        self.pub_imu.publish(m)


def main(agent="127.0.0.1", port=8888, ssid=None, password=None, resilient=True):
    if ssid:
        connect_wifi(ssid, password, hostname="qwiicbot")

    from hardware import HAVE_MODULINO

    print("[qwiicbot] Modulinos present:", HAVE_MODULINO)
    heap_report("before node")

    bot = {}

    def factory():
        # Each board needs its own XRCE key -- two clients sharing one
        # fight over entities on the Agent.
        return Node("qwiicbot", agent=agent, port=port, key=0xC0FFEE01)

    def setup(node):
        bot["r"] = QwiicBot(node)
        preallocate(node)
        heap_report("after setup")
        print("[qwiicbot] ready")
        print("           ros2 topic echo /range")
        print("           ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'")
        print("           ros2 topic pub /face std_msgs/msg/String '{data: startled}' --once")
        print("           ros2 service call /beep std_srvs/srv/Trigger")
        print("           ros2 param set /qwiicbot autonomous true")

    def on_disconnect():
        # The most important line in this file.
        if "r" in bot:
            bot["r"].stop()
            print("[qwiicbot] Agent lost -- motors stopped")

    if resilient:
        rn = ResilientNode(factory, setup=setup, on_disconnect=on_disconnect)
        rn.connect()
        rn.spin(10)
    else:
        node = factory()
        setup(node)
        try:
            node.spin(10)
        finally:
            bot["r"].stop()
            node.destroy()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1")
