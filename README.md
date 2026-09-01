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

**Pre-alpha — nothing works yet.** See the [tracking
epic](https://github.com/kevinmcaleer/snakeros/issues/1) and the [project
board](https://github.com/users/kevinmcaleer/projects/30) for what's being built
and in what order.

The critical path is short: [issue
#9](https://github.com/kevinmcaleer/snakeros/issues/9) — UDP plus an XRCE
handshake plus one `std_msgs/String` reaching `ros2 topic echo` — proves or
kills the whole approach.

## Scope

**In scope for v1:** publishers, subscriptions, timers, services, UDP and serial
transports, `std_msgs` / `geometry_msgs` / `sensor_msgs`, and a `.msg` codegen
tool for everything else.

**Out of scope for v1:** actions, TF, the ROS graph API (the Agent owns
discovery), and real-time guarantees. SnakeROS is built for `cmd_vel`, `odom`,
IMU and joint states at 10–50 Hz — not for camera frames or lidar scans.

## Licence

Apache 2.0, matching the wider ROS ecosystem.

## A note on naming

SnakeROS is an independent project. It is **not** affiliated with or endorsed by
[micro-ROS](https://micro.ros.org/), eProsima, or Open Robotics. It is
*compatible with* the micro-ROS Agent, which is a different thing.
