---
title: Hardware bring-up
---

# Hardware bring-up

## Verification status — read this first

SnakeROS is developed and continuously tested against a **real, stock micro-ROS
Agent**, with `ros2 topic echo`, `ros2 service call` and `ros2 param` doing the
asserting. It has been run on three architectures:

| Environment | What it proves |
|---|---|
| 64-bit Unix port | Full stack end to end. Real timing. |
| **32-bit ARM (`armv7l`), 190 KB heap cap** | Full stack end to end **under a Pico W-sized heap**. Real memory accounting. |
| **Bare-metal ARM Cortex-M3**, frozen firmware | The computational core on real ARM Thumb with no OS. No transport on this port, so no end-to-end. |
| **Physical Pico 2 W / Pico W / ESP32-S3** | ⚠️ **Never run.** |

| | Status |
|---|---|
| Protocol, CDR, services, parameters | **Verified** against a real Agent |
| Serial framing | **Verified** against a real Agent over a pty bridge |
| Reconnection and fail-safe | **Verified** by killing the Agent mid-run |
| Memory under a Pico W-sized heap | **Verified** — 113 KB of 190 KB free |
| 30-minute soak, leak/fragmentation | **Verified** — ~60,000 messages, ~1 KB drift |
| Core on bare-metal Cortex-M | **Verified** |
| **Wall-clock speed on real silicon** | ❌ **Not measured** |
| **Real WiFi (`cyw43`)** | ❌ **Not tested** |
| **Encoder interrupts, GC under real timing** | ❌ **Not tested** |
| **Motors, encoders, IMU on a robot** | ❌ **Not tested** |

Treat every timing figure as a *host* number until the checklist below is done.

## The outstanding hardware checklist

This is the canonical home for the remaining verification — it is a maintainer
task requiring a device, not repo work, so it lives here rather than in an
issue nobody reading the backlog can action.

- [ ] **Pico 2 W** (RP2350, primary target) — `tools/bringup.py`, then
      `examples/publisher.py` and `examples/subscriber.py`
- [ ] **Pico W** (RP2040) — expected to fit with ~113 KB spare; confirm, and
      record the real `cyw43` cost
- [ ] **ESP32-S3** — expected to be the comfortable case
- [ ] **Wall-clock publish rate** per message type:
      `micropython tests/bench.py <agent>`
- [ ] **30-minute soak on hardware:** `micropython tests/soak.py <agent> 1800`
- [ ] **Timer jitter at 50 Hz** on real silicon
- [ ] **Serial** over a real UART against `micro_ros_agent serial`, and whether
      `machine.USBDevice` gives a workable second CDC (see
      [transports](transports.md) option 2)
- [ ] **The diff-drive example** on actual motors, encoders and an IMU

When these are done the numbers replace the host figures in
[memory](memory.md), whose three-way table already has a column waiting, and
the wiring diagram and parts list for `examples/diff_drive/` should be written
against a robot that has actually been built.

### Why emulation stops here

Every emulation route was tried. The MicroPython QEMU port sets
`MICROPY_PY_ASYNCIO=0`, has no `socket`, and exposes no `machine.UART` to
Python — so it has **no transport of any kind** and cannot reach an Agent.
Enabling one means forking MicroPython, at which point it no longer represents
stock firmware. Renode and `rp2040js` hit the same wall: no `cyw43` stack.

## Step 1: run the bring-up check

```console
$ mpremote connect /dev/tty.usbmodem1101 run tools/bringup.py
```

Or on the board:

```python
import bringup
bringup.main(ssid='my-wifi', password='secret', agent='192.168.1.10')
```

It works up the stack one layer at a time — platform, import, message packs,
serialisation, WiFi, Agent, publish, rate, memory soak — and stops at the
first thing genuinely wrong instead of leaving you to guess.

## Step 2: what to watch for

**Heap.** A Pico W leaves roughly 150-190 KB once WiFi is up; a Pico 2 W
(RP2350, 520 KB SRAM) is far more comfortable. The bring-up script prints the
cost of each message pack, which is usually where the budget goes.

**GC pauses in hot paths.** Publishing allocates a CDR buffer. Doing that for
the first time inside a 50 Hz control loop invites a collection exactly where
it hurts. Call `snakeros.board.preallocate(node)` after setup.

**Fragmentation over long runs.** Slower and nastier than plain exhaustion.
The bring-up script's memory soak checks for drift over 300 publishes; a real
soak should run for half an hour.

**WiFi dropping.** It will. Use `ResilientNode` — and give it an
`on_disconnect` that stops your motors.

## Step 3: board notes

### Raspberry Pi Pico 2 W — the primary target

RP2350, 520 KB SRAM. The recommended board: enough headroom that you can
import several message packs and use reliable streams without counting bytes.

### Raspberry Pi Pico W — the constrained target

RP2040, 264 KB SRAM, and the `cyw43` WiFi driver takes a meaningful share.
Expect to install the `.mpy` build, import only the packs you need, and think
before adding a reliable stream (~4.4 KB each).

### Pico (no WiFi)

Serial only — which is a perfectly good way to run a robot. See
[Transports](transports.md), and note the REPL conflict.

### ESP32-S3

With PSRAM this is the comfortable case. `machine.UART` pin assignment
differs; check your board's pinout.

## Step 4: the fail-safe

The most important lines you will write:

```python
def on_disconnect():
    robot.stop()          # motors off when the Agent goes away

rn = ResilientNode(factory, setup=setup, on_disconnect=on_disconnect)
```

and a command timeout in your control loop:

```python
if time.time() - self._last_cmd > 0.5:
    self.stop()
```

You want **both**. A UDP publish to a dead Agent succeeds silently, so the
ping-based liveness check in `ResilientNode` is what notices the Agent has
gone — but the command timeout is what protects you when the *host* stops
sending while the link stays up.
