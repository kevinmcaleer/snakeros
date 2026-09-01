# SnakeROS

**A pure-Python ROS 2 client for MicroPython.**

SnakeROS lets a MicroPython board — a Pico 2 W, a Pico W, an ESP32 — publish and
subscribe to real ROS 2 topics. No C toolchain. No colcon. No custom firmware.
No reflashing to add a message type.

```python
from snakeros import Node
from snakeros.msg.std_msgs import String

node = Node('pico_node', agent='192.168.1.10')
pub = node.create_publisher(String, 'chatter')

while True:
    pub.publish(String(data='hello from a Pico'))
    node.spin_once()
```

On the host, that's just a ROS 2 topic:

```console
$ ros2 topic echo /chatter
data: hello from a Pico
```

## How it works

SnakeROS speaks [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/) directly, in
Python, to the **stock micro-ROS Agent**. The Agent doesn't care what's on the
other end of the wire — so there is no bridge to run, no relay node to write,
and no host-side software of ours at all.

```
┌──────────────────┐         ┌───────────────────┐         ┌─────────────┐
│  MicroPython     │  XRCE   │  micro-ROS Agent  │  DDS    │  ROS 2      │
│  board           │ ──────► │  (stock, unmod'd) │ ──────► │  graph      │
│  + snakeros      │  UDP /  │                   │         │             │
└──────────────────┘  serial └───────────────────┘         └─────────────┘
```

XRCE creates its entities from **XML strings sent at runtime**, which is the
trick that makes a pure-Python client practical: any ROS 2 message type becomes
reachable by building a string, so adding a type never means rebuilding
firmware.

## Why pure Python

Every previous attempt at MicroPython + ROS 2 went the C-bindings route, and
every one of them is now archived. Bindings mean custom firmware per board, a
CMake/colcon toolchain, and "new message type = write C and reflash" — the
friction that killed each project in turn.

SnakeROS trades throughput for reach. It installs with `mip`, runs on the
firmware you already have, works on any board, and is readable by the people
who use it.

## Status

**Working.** Publishes and subscribes to real ROS 2 topics, runs services and
parameters, over UDP and serial. Verified continuously against a **stock,
unmodified micro-ROS Agent** with `ros2 topic echo`, `ros2 service call` and
`ros2 param` doing the asserting.

| | |
|---|---|
| Publishers, subscriptions, timers | ✅ |
| CDR encode + decode | ✅ 29/29 diffed against `rclpy`, both directions |
| Services (client and server) | ✅ |
| Parameters (`ros2 param`) | ✅ live get/set with validation |
| UDP transport | ✅ |
| Serial transport (HDLC + CRC-16) | ✅ 0 CRC errors over a real Agent |
| Reliable streams + fragmentation | ✅ 1656-byte message through a 512-byte MTU |
| Message packs + `.msg` codegen | ✅ generated from real ROS 2 definitions |
| Diff-drive robot example | ✅ odometry integrates, fail-safe fires |
| Runs on 32-bit ARM under a 190 KB heap | ✅ 113 KB free with every pack imported |
| Core on bare-metal ARM Cortex-M3 | ✅ frozen firmware, no OS |
| 30-minute soak (leaks/fragmentation) | ✅ ~60,000 msgs, ~1 KB drift, stable |
| **Physical silicon** | ⚠️ **never run** — [checklist](docs/bringup.md) |

Everything above is tested on the MicroPython **Unix port** against a real
Agent, which exercises the whole protocol stack. It has not yet been run on a
physical Pico. The board-facing code is written; the numbers below are host
numbers.

## Measured

```
geometry_msgs/Twist      5.4 us encode   48 bytes
sensor_msgs/Imu         43.1 us encode  312 bytes
nav_msgs/Odometry       96.8 us encode  704 bytes

timer jitter at 50 Hz:  mean 20005 us against a 20000 us target
.mpy build:             38% of source size
```

Full tables, and an honest warning about host-vs-board, in
[docs/memory.md](docs/memory.md).

## Try it

```console
$ docker run -it --rm -p 8888:8888/udp microros/micro-ros-agent:jazzy udp4 --port 8888
$ make rig-up && make test
```

## Documentation

[Getting started](docs/index.md) ·
[Architecture](docs/architecture.md) ·
[Why not DDS?](docs/architecture.md#why-not-dds-directly) ·
[API](docs/api.md) ·
[Messages](docs/messages.md) ·
[Transports](docs/transports.md) ·
[Memory](docs/memory.md) ·
[Troubleshooting](docs/troubleshooting.md) ·
[Limitations](docs/limitations.md)

## Scope

**In:** publishers, subscriptions, timers, services, parameters, UDP and serial
transports, `std_msgs` / `geometry_msgs` / `sensor_msgs` / `nav_msgs`, and a
`.msg` codegen tool for everything else.

**Out:** actions, TF, the ROS graph API (the Agent owns discovery), security,
and real-time guarantees. Built for `cmd_vel`, `odom`, IMU and joint states at
10–50 Hz — not camera frames or lidar scans. See
[limitations](docs/limitations.md).

### Planned for v2

[**Epic #26**](https://github.com/kevinmcaleer/snakeros/issues/26) — 20 issues
across six milestones:

| | |
|---|---|
| **Correctness gaps** | ROS time sync (timestamps are currently meaningless without an RTC), `uint8[]` as bytes, full QoS, `/rosout` logging |
| **Missing ROS 2 concepts** | actions, TF broadcasting, lifecycle nodes |
| **Deployment** | persistent parameters, Agent discovery, power management, transport failover |
| **Performance** | viper CDR hot paths, generated serialisers, zero-copy decode |
| **Documentation** | a [Diátaxis](https://diataxis.fr/) restructure — tutorials, how-to, reference, explanation |
| **Ecosystem** | Zenoh transport, hardware CI |

Still out of scope in v2: security (SROS2/DDS-Security), cameras, hard
real-time, and a full tf2 tree.

### Planned for v3 — lidar

[**Epic #47**](https://github.com/kevinmcaleer/snakeros/issues/47) — 14 issues
bringing the cheap [LDROBOT LD06](https://www.yahboom.net/xiazai/LiDar-LD06/LDROBOT_LD06_Datasheet.pdf)
into scope, plus RPLIDAR and solid-state ToF sensors.

v1 ruled lidar out on measurement: a 450-point `LaserScan` costs **~1186 µs
and 28.6 KB** as SnakeROS stands. v3 changes that, because a CDR
little-endian `float32[]` body turns out to be **byte-identical to an
`array('f')` buffer** — so a scan encodes as a length plus a memcpy rather
than a loop over 450 boxed floats:

```
$ micropython tests/bench_bulk.py
IDENTICAL        : True
per-element loop :    411.7 us
length + memcpy  :      9.7 us   (43x faster)
list of floats   :  14304 bytes
array('f')       :   1856 bytes  (7.7x smaller)
```

Still out of scope in v3: cameras (`sensor_msgs/Image` — a 320×240 frame is
76 kB), on-board SLAM, and 3D spinning lidar.

## Licence

Apache 2.0, matching the wider ROS ecosystem.

## A note on naming

SnakeROS is an independent project. It is **not** affiliated with or endorsed by
[micro-ROS](https://micro.ros.org/), eProsima, or Open Robotics. It is
*compatible with* the micro-ROS Agent, which is a different thing.
