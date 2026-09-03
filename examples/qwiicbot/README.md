# QwiicBot — a SMARS robot on ROS 2, with nothing wired

A [SMARS](https://www.kevsrobots.com/blog/smars.html) tracked chassis whose
entire electronics stack is Arduino Modulino modules, joined to a ROS 2 graph
by SnakeROS.

The point of the demo is what *isn't* there. The Modulinos click together over
Qwiic — no soldering, no wiring diagram. Their MicroPython drivers install
themselves. And SnakeROS puts the whole robot on a ROS 2 graph with no bridge,
no glue node and no C toolchain. Nothing is wired, and nothing is configured.

## Five senses, five ROS interfaces

| Sense | Modulino | ROS 2 |
|---|---|---|
| **Move** | Motors | subscribes `/cmd_vel` — `geometry_msgs/Twist` |
| **See** | Distance | publishes `/range` — `sensor_msgs/Range` |
| **Measure** | Movement (IMU) | publishes `/imu/data` — `sensor_msgs/Imu` |
| **Show** | LED Matrix | subscribes `/face` — `std_msgs/String` |
| **Hear/Speak** | Buzzer | serves `/beep` — `std_srvs/Trigger` |

Plus live parameters, an `/autonomous` service, and a fail-safe that stops the
motors when the Agent disappears.

## Try it without the robot

The hardware layer falls back to a simulation, so the whole node runs on a
laptop against a real micro-ROS Agent — no Modulinos, no chassis:

```console
$ docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:jazzy udp4 --port 8888
$ micropython examples/qwiicbot/robot.py 127.0.0.1
```

```console
$ ros2 topic list
/cmd_vel
/face
/imu/data
/range

$ ros2 topic echo /range
range: 1.2000000476837158

$ ros2 topic pub /face std_msgs/msg/String '{data: startled}' --once
$ ros2 service call /beep std_srvs/srv/Trigger
$ ros2 param set /qwiicbot autonomous true
```

Turn on `autonomous` and the console narrates the behaviour:

```
[qwiicbot] wander: wall at 0.12 m -> back
[qwiicbot] face -> startled
[qwiicbot] buzzer: alarm
[qwiicbot] wander: back -> turn
[qwiicbot] wander: turn -> drive
[qwiicbot] face -> happy
```

## On the real robot

**Bring WiFi up first, in `boot.py`.** The radio needs a large contiguous
allocation; started after these imports it fails with `Wifi Out of Memory` on
an ESP32 or a Pico W. Copy `boot.py` to the device, set your SSID and password
in it, then:

```python
from robot import main
main(agent="192.168.1.10")      # note: no ssid= -- boot.py already did it
```

`main(ssid=..., password=...)` only works on a board with heap to spare.

Files to copy to the device:

```console
$ mpremote fs cp examples/qwiicbot/boot.py :
$ mpremote fs cp examples/qwiicbot/hardware.py :
$ mpremote fs cp examples/qwiicbot/robot.py :
```

## Boards that gate Qwiic power

Some boards put their Qwiic / STEMMA QT connector behind a **GPIO-controlled
regulator**, so the modules are unpowered until you raise that pin. The
symptom looks exactly like a hardware fault: no LEDs on the modules, an empty
`i2c.scan()`, and I2C errors.

The **Adafruit ESP32 Feather V2** is the notable one. Its STEMMA QT port has
its own regulator enabled by **GPIO 2** (`NEOPIXEL_I2C_POWER`). CircuitPython
and Arduino raise it automatically during board init; **MicroPython does
not**, so with generic firmware the connector is simply dead.

Just pass `board=` and the demo handles it:

```python
from robot_minimal import main
main(agent="192.168.1.149", board="feather_esp32_v2")
```

It powers the rail, scans, names what it found, and points the drivers at the
right bus:

```
[board] I2C power enabled on GPIO 2
[board] SoftI2C on sda=22 scl=20
[board] found 0x48  Motors
[board] found 0x29  Distance
[board] found 0x6A  Movement (IMU)
[board] found 0x3C  Buzzer
[board] found 0x72  LED Matrix
``` `board_setup.py` also knows `generic_esp32` and
`pico`; add your own to `BOARDS` as `(power_pin, sda, scl, active_high)`.

Two notes specific to the Feather V2: its SCL is **GPIO 20**, which
MicroPython's hardware I2C peripheral will not accept, so `setup_i2c` uses
`SoftI2C` there. And the power gate exists for a reason — dropping that pin
low cuts the modules' draw for deep sleep.

## If the board runs out of memory

A plain ESP32 with WiFi is the tightest target this has been run on. If sends
fail with `ENOMEM`, use [`robot_minimal.py`](robot_minimal.py) — same robot,
`/cmd_vel` in and `/range` out, with parameters, services, the IMU, the face
and the buzzer removed, and a 256-byte MTU.

Check which heap is actually exhausted first:

```python
import esp32
print(esp32.idf_heap_info(esp32.HEAP_DATA))   # (total, free, largest, min)
```

`largest` is what lwIP needs. See
[troubleshooting](../../docs/troubleshooting.md) for the full explanation.

## Parts

| Part | Notes |
|---|---|
| SMARS chassis | 7 printed parts, no screws — [kevsrobots.com/blog/smars.html](https://www.kevsrobots.com/blog/smars.html) |
| 2 × N20 150RPM micro gearmotors | the SMARS standard drivetrain |
| 9V battery | as stock |
| Modulino Motors / Distance / Movement / LED Matrix / Buzzer | ~€55 for the five |
| Qwiic cables | daisy-chained |
| A MicroPython host with I2C | see below |
| 2 custom printed parts | Modulino holder (replaces the Uno bay) and a Distance mount |

### Choosing the host board

The Modulinos work with any I2C/Qwiic-capable board, but **SnakeROS needs
more RAM than an Arduino UNO R4 offers** — its RA4M1 has 32 KB of SRAM.

Use a **Pico 2 W** (RP2350, 520 KB) or a **Pico W**, wired to the Qwiic chain
via the Modulinos' I2C breakout pins. That is also SnakeROS's primary target,
so the memory figures in [docs/memory.md](../../docs/memory.md) apply directly.

## Install

```python
import mip
mip.install("github:kevinmcaleer/snakeros")
mip.install("github:kevinmcaleer/snakeros/packages/sensor_msgs.json")
mip.install("github:kevinmcaleer/snakeros/packages/std_srvs.json")
mip.install("github:arduino/arduino-modulino-mpy")
```

## Parameters

| Parameter | Default | |
|---|---|---|
| `max_speed` | 0.6 | motor effort ceiling, 0–1 |
| `wheel_separation` | 0.09 | track width, m |
| `stop_distance` | 0.15 | obstacle stop distance, m |
| `autonomous` | false | run the wander behaviour |
| `publish_rate` | 10.0 | sensor rate, Hz |

All live-tunable:

```console
$ ros2 param set /qwiicbot stop_distance 0.25
```

## Two honest notes

**There is no `/odom`.** SMARS's N20 gearmotors have no encoders, so there is
no wheel odometry to publish and none is faked. The IMU gives rotation rates,
not position. Adding encoders would be a genuine upgrade — and would make this
robot a candidate for the lidar and SLAM work planned in
[epic #47](https://github.com/kevinmcaleer/snakeros/issues/47).

**The IMU publishes no orientation.** A raw 6-axis IMU measures acceleration
and rotation rate, not attitude. Per ROS convention the node sets
`orientation_covariance[0] = -1` to say so, rather than publishing a made-up
identity quaternion that downstream nodes would trust.

## Safety

Two independent defences, and you want both:

1. **A command timeout.** No `cmd_vel` for 0.6 s and the motors stop, so the
   robot does not run on a stale command.
2. **`ResilientNode`.** A UDP publish to a dead Agent *succeeds silently*, so
   Agent loss is only detectable by probing. When the ping fails, the motors
   stop and the node reconnects.

The obstacle veto only blocks **forward** motion — reverse and turn stay
available, or the robot would wedge itself against a wall permanently.
