---
title: Limitations
---

# Limitations

SnakeROS trades throughput for reach. Here is exactly what you give up.

Several items below are planned for v2 —
[epic #26](https://github.com/kevinmcaleer/snakeros/issues/26) — and are marked
**(v2)**. Those without a marker are deliberate long-term non-goals.

## Not implemented, by design

**Actions** *(v2: [#31](https://github.com/kevinmcaleer/snakeros/issues/31))*. No `rclpy.action` equivalent. micro-ROS does not have them
natively either. Long-running goals with feedback and cancellation are a lot
of machinery for a microcontroller; a service plus a status topic covers most
real cases.

**TF / `tf2`** *(v2, broadcast only: [#32](https://github.com/kevinmcaleer/snakeros/issues/32))*. No transform broadcaster or listener. Publish
`geometry_msgs/TransformStamped` on `/tf` yourself if you need it — the
message type ships — but the transform *tree*, buffering and interpolation
belong on the host.

**The ROS graph API.** No `get_node_names()`, no topic introspection. The
Agent owns discovery. This is not a gap so much as the architecture.

**Executors, callback groups, the rclpy lifecycle.** Deliberately absent: that
machinery buys nothing on a microcontroller and costs RAM.

**Real-time guarantees.** MicroPython has a garbage collector. Timer jitter is
good (p95 within ~1 ms of a 20 ms period on the host) but there is no bound.
Do not put a motor commutation loop in Python.

**Parameter persistence to flash** *(v2: [#34](https://github.com/kevinmcaleer/snakeros/issues/34))*. Parameters are runtime-only; they reset on
power cycle. This is the one genuinely board-specific piece and is a known gap
rather than a decision.

## Throughput ceiling

Built for `cmd_vel`, `odom`, IMU and joint states at **10-50 Hz**.

Not for camera frames, and not for lidar scans at rate. A 400-point
`LaserScan` costs roughly 4 fragments and hundreds of microseconds to encode;
you can publish one occasionally, not at 20 Hz. `sensor_msgs/Image` and
`PointCloud2` are deliberately not shipped.

**Lidar is planned for v3** *([epic #47](https://github.com/kevinmcaleer/snakeros/issues/47))*.
Holding numeric fields as `array` and encoding them as a length plus a memcpy
measures **43× faster and 7.7× less heap**, which moves an LD06 at 10 Hz from
impossible to plausible. Cameras stay out: a 320×240 grayscale frame is 76 kB,
which is not a memcpy away from feasible.

See [Memory and performance](memory.md) for measured numbers.

## QoS

Only reliability and history depth are exposed through the entity XML. No
deadline, liveliness, lifespan or durability
*(v2: [#29](https://github.com/kevinmcaleer/snakeros/issues/29))*.

Note the distinction that catches people: XRCE stream reliability
(board↔Agent) is **independent** of DDS QoS reliability (Agent↔ROS 2).

## Security

None. No SROS2, no DDS-Security, no authentication or encryption. Anything on
the network segment can publish to your robot. Treat a SnakeROS robot as you
would any other unauthenticated device: keep it on a network you control.

## Platform

Requires MicroPython **1.20+** for `asyncio`; developed and tested against
**1.29.0**. `.mpy` builds are tied to the `.mpy` format version, not to a
particular board.

Tested on the MicroPython Unix port against a real micro-ROS Agent in CI.
Board coverage is documented in [Hardware bring-up](bringup.md), including
what has and has not been verified on physical hardware.
