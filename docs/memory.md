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

## Three measurement environments

Board numbers need a board. Short of one, these figures come from three places
and it matters which:

| Environment | Pointers | Heap | What it tells you |
|---|---|---|---|
| Unix port, laptop | 64-bit | unlimited | Relative profile, regression checks. **Timing is real.** |
| **32-bit ARM (`armv7l`), 190 KB heap** | **32-bit** | **190 KB** | **Heap accounting close to a real Pico W.** Timing is emulated and meaningless. |
| A physical Pico 2 W / Pico W | 32-bit | 190-400 KB | The real answer. Not yet run — see [bring-up](bringup.md). |

The 32-bit ARM column is the useful one for memory: it is a real 32-bit
MicroPython v1.29.0 with a real allocator running under a heap cap, against a
real micro-ROS Agent. Its *timing* is emulated (roughly 13× slower than the
host) and should be ignored entirely.

## Does it fit on a Pico W?

**Yes, comfortably.** Measured on 32-bit ARM with a 190 KB heap — the tightest
realistic target:

```
heap total 190,080 bytes
  core             43,072   (144,320 free)
  std_msgs         10,704   (133,536 free)
  geometry_msgs    11,888   (121,568 free)
  sensor_msgs       6,240   (115,248 free)
  node + session       784   (114,384 free)
  publisher            128   (114,192 free)
  subscription         160   (113,952 free)

  384 Imu published, 113,056 bytes still free
```

Every message pack imported, a publisher and a subscription created, publishing
`sensor_msgs/Imu` — and **113 KB of the 190 KB heap is still free.** A Pico W
is not the constrained stretch goal this project assumed it would be.

## ESP32: the two-heap trap, measured on real silicon

An ESP32 has **two heaps**, and `gc.mem_free()` only reports one:

* MicroPython's **GC heap** — Python objects
* The **ESP-IDF heap** — where lwIP takes its network buffers

MicroPython's GC heap *grows by claiming blocks from the IDF heap and never
returns them*. Load enough before the network is up and lwIP starves, so sends
fail with `ENOMEM` while `gc.mem_free()` still looks fine.

Measured on a Generic ESP32 (MicroPython v1.29.0), same board, same firmware,
differing only in what `boot.py` does:

| | `boot.py` imports SnakeROS first | `boot.py` uses raw `network` + `gc.threshold()` |
|---|---|---|
| python heap free | 35,664 | **113,584** |
| IDF heap free | 956 | **85,980** |
| **IDF largest contiguous** | **400 B** | **59,392 B** |
| result | every send `ENOMEM` | works |

**148× more contiguous space for lwIP**, from two changes in `boot.py`:

1. Use raw `network`, not `snakeros.board`. Even with lazy imports the latter
   costs ~10 KB you do not need before WiFi is up.
2. `gc.threshold(gc.mem_alloc() + gc.mem_free() // 4)` immediately after WiFi
   and before any heavy import, so the GC collects instead of growing.

Diagnose with:

```python
import esp32
print(esp32.idf_heap_info(esp32.HEAP_DATA)[-1])   # (total, free, largest, min)
```

`largest` is the number that matters — lwIP needs a contiguous block.

See [`examples/qwiicbot/boot.py`](https://github.com/kevinmcaleer/snakeros/blob/main/examples/qwiicbot/boot.py).

## Leaks and fragmentation: 30-minute soak

Heap exhaustion fails loudly. **Fragmentation does not** — it degrades slowly
over tens of minutes and is the failure mode that shows up after a robot has
been running all afternoon. `tests/soak.py` publishes an `Imu` plus a
*varying-length* string (fixed sizes would not stress the allocator) and
watches free heap.

Two 30-minute runs under board-sized heap caps, against a live Agent:

| | Pico W proxy (190 KB) | Pico 2 W proxy (400 KB) |
|---|---|---|
| Messages published | **59,887** | **60,001** |
| Baseline free | 70,752 | 283,296 |
| Final free | 69,696 | 282,336 |
| **Net drift** | **1,056 bytes** | **960 bytes** |
| Worst drift | 1,056 bytes | 1,024 bytes |
| Verdict | **STABLE** | **STABLE** |

Drift plateaued at about 1 KB within the first minute and never grew again
across 60,000 messages. That is a one-off settling cost, not a leak and not
fragmentation creep.

```console
$ micropython -X heapsize=190K tests/soak.py 127.0.0.1 1800 0xAABBCC01 soak_picow
```

Give each concurrent client a **distinct XRCE key** — two sharing one fight
over entities on the Agent and the second fails to create a participant.

## 32-bit vs 64-bit heap cost

Roughly halved, as you would expect for pointer-heavy structures:

| | 64-bit host | **32-bit ARM** |
|---|---|---|
| `snakeros` core | 61,152 | **41,088** |
| `std_msgs` | 18,624 | **10,368** |
| `geometry_msgs` | 13,920 | **7,344** |
| `sensor_msgs` | 11,008 | **6,144** |
| one `Twist` instance | 256 | **128** |
| one `Imu` instance | 1,184 | **624** |
| one `Odometry` instance | 1,984 | **992** |

Quote the 32-bit column when budgeting for a board.

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

The `.mpy` build is **roughly a third of the source size** and cuts the *peak* heap during
import by ~32 KB — which is what actually raises `MemoryError` on a board. It
does **not** reduce steady-state heap. See [Packaging](packaging.md) for the
measured table.

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
