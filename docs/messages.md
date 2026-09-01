---
title: Messages
---

# Messages

## What ships

| Package | Types |
|---|---|
| `std_msgs` | 30 |
| `geometry_msgs` | 33 |
| `sensor_msgs` | 12 (curated) |
| `nav_msgs` | `Odometry`, `Path` |
| `std_srvs` | `SetBool`, `Trigger`, `Empty` |
| `builtin_interfaces` | `Time`, `Duration` |
| `rcl_interfaces` | parameter types and services |

`sensor_msgs` is deliberately curated. `Image` and `PointCloud2` are not worth
shipping to a microcontroller and are omitted rather than included as a trap.

**These are generated from the real ROS 2 `.msg` definitions**, not written by
hand — so field order, constants and covariance array sizes are right by
construction.

```python
from snakeros.msg.geometry_msgs import Twist
from snakeros.msg.sensor_msgs import Imu

t = Twist()
t.linear.x = 0.5
t.angular.z = -1.25

imu = Imu()
imu.header.frame_id = 'imu_link'
imu.orientation_covariance = [0.1] * 9
```

Packs import lazily, so you pay heap only for what you use. Message constants
come through from the `.msg`:

```python
from snakeros.msg.sensor_msgs import BatteryState
BatteryState.POWER_SUPPLY_STATUS_CHARGING   # 1
```

## Generating your own

Any `.msg` — including custom robot interfaces — becomes a Python module:

```console
$ python3 tools/snakeros_gen.py --search-path /opt/ros/jazzy/share \
      --package my_robot --out snakeros/msg
$ python3 tools/snakeros_gen.py MyThing.msg --package my_robot --out ./gen
```

`--only Imu,JointState` prunes to a set of types plus their in-package
dependencies.

**It runs on plain CPython and needs no ROS 2 installation** — a `share/`
directory, or even a single `.msg` file, is enough. Requiring a full ROS
install on the dev machine would defeat much of the point.

## How a message is represented

A compact tuple-of-tuples schema, not per-field Python objects, because on a
Pico W per-type overhead decides whether a program fits:

```python
class Vector3(Msg):
    _package_ = "geometry_msgs"
    _kind_ = "msg"
    _name_ = "Vector3"
    _fields_ = (("x", "d"), ("y", "d"), ("z", "d"))
```

| Code | Type | | Code | Type |
|---|---|---|---|---|
| `b` | bool | | `I` | uint32 |
| `o` | byte / uint8 | | `l` | int64 |
| `1` | int8 | | `L` | uint64 |
| `c` | char | | `f` | float32 |
| `s` | int16 | | `d` | float64 |
| `S` | uint16 | | `T` | string |
| `i` | int32 | | | |

Composites are tuples: `('a', elem, n)` a fixed array, `('q', elem)` a
sequence. A nested message is the class itself.

## The struct fast path

A message made entirely of fixed-size primitives is compiled **once** into a
single `struct` format string with CDR padding baked in as `x` bytes:

```python
Vector3._fast()   # '<ddd'
Twist._fast()     # None -- nested, so it uses the field loop
```

One `struct.pack` instead of a Python loop over fields. It covers most of the
small, high-rate messages a robot actually publishes.

## Verification

The encoder and decoder are diffed against `rclpy` in **both directions**
across 29 cases covering every ROS 2 field type:

```console
$ make cdr-diff
29/29 cases passed
29/29 cases readable by rclpy
```

Byte-equality is deliberately not the test — rclpy leaves uninitialised stack
garbage in CDR padding and can emit trailing slack, so two correct encoders
still differ byte for byte. The test is semantic.
