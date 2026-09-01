---
title: Transports
---

# Transports

## Which to pick

| | UDP | Serial |
|---|---|---|
| Boards | Pico W, Pico 2 W, ESP32 | **any** Pico |
| Setup | WiFi credentials, an AP, an IP | a cable |
| Framing | free — one datagram, one message | HDLC with CRC |
| Complication | network config | the USB CDC port *is* the REPL |
| Agent | `udp4 --port 8888` | `serial --dev /dev/ttyUSB0` |

**Start with UDP.** Move to serial when you want a robot with no WiFi
dependency, no credentials in your source, and one less thing to go wrong on a
crowded 2.4 GHz band at a maker faire.

## UDP

```python
from snakeros import Node
node = Node('pico', agent='192.168.1.10', port=8888)
```

`agent` must be your PC's **LAN address**, not `127.0.0.1`.

```python
from snakeros.transport import UDPTransport
node = Node('pico', transport=UDPTransport('192.168.1.10', 8888, mtu=512, timeout=0.002))
```

**Keep `timeout` small.** It bounds how long one `recv()` blocks with nothing
waiting, so it puts a floor under `spin_once()` and a ceiling on your control
rate. At 0.1 s a "50 Hz" timer fires at 10 Hz.

## Serial

```python
from machine import Pin, UART
from snakeros import Node
from snakeros.transport import SerialTransport

uart = UART(0, 115200, tx=Pin(0), rx=Pin(1), timeout=0)
node = Node('pico', transport=SerialTransport(uart=uart))
```

```console
$ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

### Framing

```
0x7E | stuffed( src, dst, len_lo, len_hi, payload..., crc_lo, crc_hi )
```

`0x7E` → `0x7D 0x5E`, `0x7D` → `0x7D 0x5D`.

The CRC is **CRC-16/ARC** — reflected polynomial `0xA001`, initial value 0 —
over the **payload only**.

> eProsima's documentation states the polynomial as `x^16 + x^12 + x^5 + 1`
> (CCITT, `0x1021`). **That is wrong.** The table in
> `stream_framing_protocol.c` is unambiguously CRC-16/ARC. An implementation
> that trusts the prose produces frames the Agent silently discards.

`transport.crc_errors` and `transport.resyncs` tell you whether frames are
arriving corrupted or the stream is out of sync.

## The REPL conflict

On a Pico, the USB CDC port the Agent would talk to is the same one
MicroPython uses for its REPL. Three ways out, none free.

### 1. A UART on GPIO pins — recommended

Costs two pins and a USB-serial adapter. **Leaves the REPL alone**, so you can
still print, debug and drop to a prompt while the board is talking to ROS 2.
For development that is worth far more than the two pins.

### 2. A second USB CDC interface

```python
from snakeros.transport.serial import usb_cdc_stream
```

Best user experience: one cable, REPL intact. Needs `machine.USBDevice` and a
board whose port enables runtime USB reconfiguration, and the interface has to
be built for the specific board. `usb_cdc_stream()` raises a useful
`ImportError` where unsupported so you can fall back to a UART.

### 3. Take over the REPL's port

```python
from snakeros.transport.serial import take_over_repl_cdc
stream = take_over_repl_cdc()          # os.dupterm(None)
```

You get the port the Agent expects and **lose the REPL until reset**. Fine for
a deployed robot, painful during development. Think before reaching for it.

## Serial over TCP

```python
from snakeros.transport import ByteStreamTransport, TCPByteStream
node = Node('pico', transport=ByteStreamTransport(TCPByteStream('10.0.0.5', 9999).connect()))
```

For serial lines tunnelled over the network — and it is how SnakeROS's own
serial framing is tested in CI, against a real Agent through a pty bridge.

## Writing your own

The contract is four methods and an attribute: `open()`, `send(bytes)`,
`recv() -> bytes | None`, `close()`, `mtu`. `recv()` returns `None` for
"nothing yet" rather than raising. That seam is deliberately narrow so a
different transport — or a different middleware entirely — slots in without
disturbing anything above it.
