"""Hardware bindings for the diff-drive robot, with a simulated fallback.

The split matters. Everything in ``robot.py`` -- odometry integration, the
ROS interface, parameters, fail-safe -- is portable logic that can be tested
on the MicroPython Unix port with no robot present. Only this file touches
pins.

On a board it drives real motors and reads real encoders. Anywhere else it
falls back to a simple kinematic simulation, so the example runs, publishes
plausible ``/odom``, and can be exercised end to end against a real Agent
before any hardware exists.
"""

import math
import time

try:
    from machine import Pin, PWM, I2C

    HAVE_HARDWARE = True
except ImportError:
    HAVE_HARDWARE = False


class Motor:
    """A single motor on an H-bridge (two direction pins + PWM enable)."""

    def __init__(self, in1, in2, en, freq=20000):
        self.sim_speed = 0.0
        if not HAVE_HARDWARE:
            return
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.pwm = PWM(Pin(en))
        self.pwm.freq(freq)

    def set(self, duty):
        """``duty`` in [-1.0, 1.0]. Negative is reverse."""
        duty = max(-1.0, min(1.0, duty))
        self.sim_speed = duty
        if not HAVE_HARDWARE:
            return
        if duty >= 0:
            self.in1.value(1)
            self.in2.value(0)
        else:
            self.in1.value(0)
            self.in2.value(1)
        self.pwm.duty_u16(int(abs(duty) * 65535))

    def stop(self):
        self.sim_speed = 0.0
        if not HAVE_HARDWARE:
            return
        self.in1.value(0)
        self.in2.value(0)
        self.pwm.duty_u16(0)


class Encoder:
    """Quadrature encoder counted with pin interrupts.

    In simulation, integrates the motor's commanded duty so odometry still
    moves and the maths can be checked without a robot.
    """

    def __init__(self, pin_a, pin_b, ticks_per_rev=1440, motor=None,
                 sim_max_rps=3.0):
        self.count = 0
        self.ticks_per_rev = ticks_per_rev
        self._motor = motor
        self._sim_max_rps = sim_max_rps
        self._sim_last = time.time()
        if not HAVE_HARDWARE:
            return
        self._a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self._b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self._a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._tick)

    def _tick(self, _pin):
        self.count += 1 if self._a.value() == self._b.value() else -1

    def read(self):
        if not HAVE_HARDWARE and self._motor is not None:
            now = time.time()
            dt = now - self._sim_last
            self._sim_last = now
            revs = self._motor.sim_speed * self._sim_max_rps * dt
            self.count += int(revs * self.ticks_per_rev)
        return self.count

    def revolutions(self):
        return self.read() / float(self.ticks_per_rev)


class IMU:
    """MPU6050-style IMU over I2C, or a quiet simulated one."""

    ADDR = 0x68

    def __init__(self, scl=None, sda=None, bus_id=0):
        self.ok = False
        if not HAVE_HARDWARE or scl is None:
            return
        try:
            self.i2c = I2C(bus_id, scl=Pin(scl), sda=Pin(sda))
            self.i2c.writeto_mem(self.ADDR, 0x6B, b"\x00")  # wake
            self.ok = True
        except Exception:
            self.ok = False

    def read(self):
        """Return ``(ax, ay, az, gx, gy, gz)`` in m/s^2 and rad/s."""
        if not self.ok:
            return (0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
        d = self.i2c.readfrom_mem(self.ADDR, 0x3B, 14)

        def s16(hi, lo):
            v = (d[hi] << 8) | d[lo]
            return v - 65536 if v > 32767 else v

        ax = s16(0, 1) / 16384.0 * 9.80665
        ay = s16(2, 3) / 16384.0 * 9.80665
        az = s16(4, 5) / 16384.0 * 9.80665
        gx = s16(8, 9) / 131.0 * math.pi / 180.0
        gy = s16(10, 11) / 131.0 * math.pi / 180.0
        gz = s16(12, 13) / 131.0 * math.pi / 180.0
        return (ax, ay, az, gx, gy, gz)
