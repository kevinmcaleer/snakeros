---
title: Hardware bring-up
---

# Hardware bring-up

## Verification status — read this first

SnakeROS is developed and continuously tested on the **MicroPython Unix port
against a real, stock micro-ROS Agent**, with `ros2 topic echo`,
`ros2 service call` and `ros2 param` doing the asserting. That covers the
entire protocol stack: handshake, entity creation, CDR both ways, reliable
streams, fragmentation, serial framing, services and parameters.

What that does **not** cover is a physical board.

| | Status |
|---|---|
| Protocol, CDR, services, parameters | **Verified** in CI against a real Agent |
| Serial framing | **Verified** against a real Agent over a pty bridge |
| Reconnection and fail-safe | **Verified** by killing the Agent mid-run |
| Timer jitter, encode/decode rates | **Measured** — on the host, not a board |
| Pico 2 W / Pico W / ESP32-S3 | **Not yet run on physical hardware** |
| Real motors, encoders, IMU | **Not yet run on physical hardware** |

The board-facing code — `snakeros/board.py`, the pin handling in
`examples/diff_drive/hardware.py` — is written and imports cleanly, but has
not been exercised on silicon. Treat the memory and rate tables as a
*relative* profile until someone runs them on a board. If you do, please open
an issue with the numbers.

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
