---
title: Memory and performance
---

# Memory and performance

SnakeROS trades throughput for reach. That trade is only honest if the cost is
measured and published rather than hand-waved, so here are the numbers.

## The headline

**Design target: `cmd_vel`, `odom`, IMU and joint states at 10-50 Hz.** Not
camera frames, not lidar scans at rate. The encoder is comfortably fast enough
for the first and comfortably too slow for the second, and the tables below
show exactly where the line falls.

## A warning about these numbers

The results below are from the **MicroPython Unix port on an Apple Silicon
laptop**. They are a *relative profile* and a regression check -- which
messages are expensive, how much the fast path helps, whether a change made
things worse. They are **not** board numbers: a desktop has 64-bit pointers, a
far faster CPU and effectively unlimited heap.

Board figures need real hardware. Run them yourself with:

```console
$ micropython tests/bench.py 192.168.1.10
```

and see [Hardware bring-up](bringup.md).

## Full benchmark output

```
==============================================================
SnakeROS benchmarks
  platform: (name='micropython', version=(1, 29, 0, ''), _machine='darwin [Clang 21.0.0] version', _mpy=774, _build='standard', _thread='unsafe')
==============================================================

-- heap cost of imports --
  snakeros core                          61,152 bytes
  msg.builtin_interfaces                  7,168 bytes
  msg.std_msgs                           18,624 bytes
  msg.geometry_msgs                      13,920 bytes
  msg.sensor_msgs                        11,008 bytes
  msg.nav_msgs                            1,184 bytes
  msg.std_srvs                            2,784 bytes

-- heap cost per message instance --
  std_msgs/String                            64 bytes
  geometry_msgs/Twist                       256 bytes
  sensor_msgs/Imu                         1,184 bytes
  nav_msgs/Odometry                       1,984 bytes

-- CDR encode speed (per message) --
  std_msgs/String (17 ch)                 2.7 us      22 bytes    370165 msg/s
  geometry_msgs/Vector3                   2.2 us      24 bytes    454133 msg/s
  geometry_msgs/Twist                     5.1 us      48 bytes    195446 msg/s
  geometry_msgs/PoseStamped               9.5 us      72 bytes    105474 msg/s
  sensor_msgs/Imu                        40.9 us     312 bytes     24461 msg/s
  sensor_msgs/JointState (2)             15.9 us     100 bytes     62913 msg/s
  sensor_msgs/LaserScan (100)           122.7 us     452 bytes      8150 msg/s
  nav_msgs/Odometry                      96.1 us     704 bytes     10408 msg/s

-- CDR decode speed (per message) --
  std_msgs/String (17 ch)                 2.5 us    393314 msg/s
  geometry_msgs/Vector3                   2.0 us    500501 msg/s
  geometry_msgs/Twist                     4.9 us    203707 msg/s
  geometry_msgs/PoseStamped               9.7 us    103306 msg/s
  sensor_msgs/Imu                        44.5 us     22467 msg/s
  sensor_msgs/JointState (2)             18.2 us     54984 msg/s
  sensor_msgs/LaserScan (100)           127.4 us      7851 msg/s
  nav_msgs/Odometry                     102.1 us      9790 msg/s

-- struct fast path --
  Vector3 compiled format            <ddd
  Twist (nested, no fast path)       None

-- live publish rate (Agent at 127.0.0.1) --
  Node + session + participant            1,600 bytes
  one publisher                             192 bytes
  one subscription                          384 bytes
  publish Twist (no spin)                21.7 us     46062 msg/s

-- timer jitter at 50 Hz --
  target period                         20000 us
  mean period                           19996 us
  median                                20352 us
  p95                                   20926 us
  worst                                 25995 us
  fires in 5 s                            250

==============================================================
free heap at exit: 1,927,200 bytes
```

## What the numbers say

**The struct fast path earns its place.** `Vector3` compiles to the single
format `<ddd` and encodes in ~2 µs. `Twist`, which nests two `Vector3`s and so
cannot use the fast path, takes ~5 µs -- more than twice as long for only
twice the data, because the field loop costs real time.

**Covariance arrays dominate the big messages.** `Odometry` carries two
36-element `float64` covariance blocks; they are most of its 704 bytes and
most of its ~97 µs. If you do not need covariance, publishing `Pose` and
`Twist` separately is dramatically cheaper.

**Decoding costs about the same as encoding**, which matters if a board
subscribes to something chatty.

## Memory budget rules of thumb

| Thing | Cost |
|---|---|
| SnakeROS core import | see table above; import only what you use |
| One message pack | 1-25 KB depending on the pack |
| A publisher | small, hundreds of bytes |
| A subscription | roughly double a publisher |
| A **reliable** stream | `max_buffered * MTU` ≈ **4.4 KB** at the defaults |
| A declared parameter | ~120-200 bytes |

The `.mpy` build is **38% of the source size** and measurably reduces heap. On
a Pico W that is the difference between fitting and not; see
[Packaging](packaging.md).

## If you run out of memory

1. Install the `.mpy` build rather than `.py`.
2. Import only the message packs you use -- `snakeros.msg.geometry_msgs`, not
   everything.
3. Drop reliable streams you do not need; best-effort costs no window.
4. Reuse message objects instead of allocating per publish. The diff-drive
   example does this deliberately.
5. Call `snakeros.board.preallocate(node)` after setup to get the first
   allocations and their collection out of the way before your control loop
   starts.
6. Move to a Pico 2 W. 520 KB against the Pico W's ~150-190 KB of usable heap
   is not a small difference.
