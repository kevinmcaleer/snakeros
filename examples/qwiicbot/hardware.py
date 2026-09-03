"""Modulino hardware for the QwiicBot, with a simulated fallback.

Wraps Arduino's official ``arduino-modulino-mpy`` library so ``robot.py``
can stay pure ROS logic. Everything here speaks I2C; nothing above it does.

Off-hardware -- the MicroPython Unix port, or a board with no Modulinos
attached -- each wrapper falls back to a simple simulation. That is what lets
the whole robot node be run and tested against a real micro-ROS Agent before
the Modulinos are plugged in, which is exactly how the rest of SnakeROS was
built.

Units, converted here so the ROS layer never has to think about them:

===================  =================  ==================
Modulino gives       ROS 2 wants        conversion
===================  =================  ==================
distance in cm       metres             / 100
acceleration in g    m/s^2              * 9.80665
angular vel in dps   rad/s              * pi / 180
speed 0-100 + flag   signed -1.0..1.0   sign -> invert_x
===================  =================  ==================
"""

import math
import time

# Load each driver **on first use**, not at import.
#
# Two reasons. Arduino's package __init__ eagerly imports all seventeen
# drivers (~131 KB of source), which raises MemoryError on an ESP32 with
# SnakeROS resident -- see modulino_lazy_init.py for a drop-in that fixes
# that. And even one-at-a-time, loading all five costs a robot that only
# drives and ranges the two it never uses.
#
# So a wrapper that is never constructed costs nothing, and one whose driver
# will not load falls back to simulation rather than taking the robot down:
# no LED matrix is a robot without a face, not a robot that will not start.

_LOADED = {}

# A caller-supplied I2C bus, for boards whose Qwiic pins are not MicroPython's
# defaults or whose connector is behind a power gate. See board_setup.py.
_I2C = None


def set_i2c(bus):
    """Use ``bus`` for every Modulino created from now on.

    Call before constructing any wrapper::

        from board_setup import setup_i2c
        from hardware import set_i2c
        set_i2c(setup_i2c("feather_esp32_v2"))
    """
    global _I2C
    _I2C = bus


def _make(name, **kw):
    """Construct a Modulino driver, passing the shared bus if one was set."""
    cls = _driver(name)
    if cls is None:
        return None
    if _I2C is not None:
        kw["i2c_bus"] = _I2C
    try:
        return cls(**kw)
    except Exception as e:
        print("[hardware] %s failed to initialise (%s) -- simulating it"
              % (name, e))
        return None


def _driver(name):
    """Return a Modulino class, or None if it is unavailable."""
    if name in _LOADED:
        return _LOADED[name]
    try:
        module = __import__("modulino", None, None, (name,))
        cls = getattr(module, name)
    except (ImportError, MemoryError, AttributeError) as e:
        print("[hardware] %s unavailable (%s) -- simulating it"
              % (name, type(e).__name__))
        cls = None
    _LOADED[name] = cls
    return cls


def modulinos_present():
    """True if the drive module is available. Probes without loading the rest."""
    return _driver("ModulinoMotors") is not None


G_TO_MS2 = 9.80665
DPS_TO_RADS = math.pi / 180.0


class Drive:
    """Modulino Motors driving the SMARS twin N20 gearmotors.

    The Modulino API takes an unsigned 0-100 speed plus a separate direction
    flag per channel, so a signed value has to be split into magnitude and
    ``invert``.
    """

    def __init__(self):
        self.left = 0.0
        self.right = 0.0
        self._m = None
        self._m = _make("ModulinoMotors")
        if self._m is not None:
            self._m.stepper_mode_enabled = False

    def set(self, left, right):
        """``left``/``right`` in [-1.0, 1.0]. Negative is reverse."""
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        self.left = left
        self.right = right
        if self._m is None:
            return
        self._m.invert_a = left < 0
        self._m.invert_b = right < 0
        self._m.speed_a = int(abs(left) * 100)
        self._m.speed_b = int(abs(right) * 100)

    def stop(self):
        self.left = self.right = 0.0
        if self._m is not None:
            self._m.stop()

    def release(self):
        """Coast rather than brake -- used on shutdown."""
        self.left = self.right = 0.0
        if self._m is not None:
            self._m.release()

    def current(self):
        """``(a, b)`` motor current in mA. Useful for stall detection."""
        if self._m is None:
            # simulate: current roughly tracks commanded effort
            return (abs(self.left) * 180.0, abs(self.right) * 180.0)
        try:
            a, b = self._m.sensed_current
            return (float(a), float(b))
        except Exception:
            return (0.0, 0.0)


class Rangefinder:
    """Modulino Distance -- a time-of-flight sensor, in place of SMARS's
    usual HC-SR04.

    The library returns ``None`` when there is no valid return. That becomes
    ``+inf`` in ROS, **not** zero: zero means "an obstacle at the sensor" to
    every consumer downstream, which is the classic rangefinder bug.
    """

    MIN_M = 0.02
    MAX_M = 2.0

    def __init__(self, drive=None):
        self._d = _make("ModulinoDistance")
        self._drive = drive
        self._sim_m = 1.2

    def read(self):
        """Distance in metres, or ``+inf`` for no reading."""
        if self._d is not None:
            cm = self._d.distance
            if cm is None:
                return float("inf")
            return cm / 100.0
        # simulation: a wall the robot approaches as it drives forward
        fwd = (self._drive.left + self._drive.right) / 2.0 if self._drive else 0.0
        self._sim_m -= fwd * 0.05
        if self._sim_m < 0.05:
            self._sim_m = 0.05
        if self._sim_m > 2.5:
            return float("inf")
        return self._sim_m


class Imu:
    """Modulino Movement -- a 6-axis IMU."""

    def __init__(self):
        self._m = _make("ModulinoMovement")
        self._t0 = time.time()

    def read(self):
        """``(ax, ay, az, gx, gy, gz)`` in m/s^2 and rad/s."""
        if self._m is None:
            # simulation: gravity on z, a little noise on the rest
            t = time.time() - self._t0
            return (0.0, 0.0, G_TO_MS2, 0.0, 0.0, 0.05 * math.sin(t))
        acc = self._m.acceleration
        gyro = self._m.angular_velocity
        return (
            acc.x * G_TO_MS2, acc.y * G_TO_MS2, acc.z * G_TO_MS2,
            gyro.x * DPS_TO_RADS, gyro.y * DPS_TO_RADS, gyro.z * DPS_TO_RADS,
        )


class Buzzer:
    """Modulino Buzzer."""

    def __init__(self):
        self._b = _make("ModulinoBuzzer")
        self.last = None

    def tone(self, frequency, ms=120):
        self.last = (frequency, ms)
        if self._b is not None:
            self._b.tone(frequency, ms, blocking=False)

    def chirp(self):
        self.tone(1800, 90)

    def alarm(self):
        self.tone(440, 300)

    def off(self):
        self.last = None
        if self._b is not None:
            self._b.no_tone()


class Face:
    """Modulino LED Matrix -- a 12x8 display, used here as the robot's face.

    Frames are 12 bytes, one per column, 8 bits per column.
    """

    WIDTH = 12
    HEIGHT = 8

    # 12 columns x 8 rows, LSB at the top of each column
    FACES = {
        "happy":    b"\x00\x00\x0c\x0c\x00\x00\x00\x00\x0c\x0c\x00\x00",
        "startled": b"\x00\x1e\x12\x1e\x00\x00\x00\x00\x1e\x12\x1e\x00",
        "sleepy":   b"\x00\x00\x18\x18\x00\x00\x00\x00\x18\x18\x00\x00",
        "cross":    b"\x00\x22\x14\x08\x14\x22\x00\x22\x14\x08\x14\x22",
        "blank":    b"\x00" * 12,
    }

    def __init__(self):
        self._m = _make("ModulinoLEDMatrix", use_grayscale=False)
        self.current = None
        self.show("happy")

    def show(self, name):
        """Display a named expression. Unknown names are ignored."""
        frame = self.FACES.get(name)
        if frame is None:
            return False
        self.current = name
        if self._m is not None:
            self._m.clear()
            self._m.load_frame(frame)
            self._m.show()
        return True

    def names(self):
        return sorted(self.FACES.keys())

    def clear(self):
        self.current = "blank"
        if self._m is not None:
            self._m.clear()
            self._m.show()
