---
title: Architecture
---

# Architecture

## The insight this rests on

**The micro-ROS Agent is a standard DDS-XRCE agent.** It does not care whether
the client on the other end of the wire is C, Rust or Python — only that the
bytes are right. So SnakeROS ships *no host-side software at all*: no bridge,
no relay node, nothing to install beside your ROS 2 system.

```
┌──────────────────┐         ┌───────────────────┐         ┌─────────────┐
│  MicroPython     │  XRCE   │  micro-ROS Agent  │  DDS    │  ROS 2      │
│  board           │ ──────► │  (stock, unmod'd) │ ──────► │  graph      │
│  + snakeros      │  UDP /  │                   │         │             │
└──────────────────┘  serial └───────────────────┘         └─────────────┘
```

## Why a pure-Python client is even possible

XRCE creates its entities from **XML strings sent at runtime**:

```xml
<dds><topic><name>rt/chatter</name>
<dataType>std_msgs::msg::dds_::String_</dataType></topic></dds>
```

Participant → topic → publisher → datawriter, all as strings. That is the
whole trick. Supporting a new ROS 2 message type is *string concatenation*,
not code generation — which is why adding a type never means rebuilding
firmware.

Every previous attempt at MicroPython + ROS 2 went the C-bindings route, and
every one is now archived. Bindings mean custom firmware per board, a
CMake/colcon toolchain, and "new message type = write C and reflash". That
friction is what killed them.

## The layers

```
snakeros/
├── node.py            Node, Publisher, Subscription, Timer
├── services.py        service client and server
├── parameters.py      ros2 param support
├── board.py           WiFi, resilient reconnection, heap tools
├── msg/               message type system and generated packs
│   └── _base.py       schema, struct fast path
├── cdr/               XCDR1 encode and decode
│   ├── writer.py
│   └── reader.py
├── xrce/              the DDS-XRCE protocol
│   ├── const.py       constants taken from eProsima's C sources
│   ├── entities.py    object ids, name mangling, entity XML
│   ├── session.py     handshake, entity lifecycle, data
│   └── reliable.py    HEARTBEAT/ACKNACK, retransmit window
└── transport/
    ├── udp.py
    ├── serial.py
    └── framing.py     HDLC framing and CRC-16
```

Each layer depends only on the one below, and the transport contract is four
methods (`open`/`send`/`recv`/`close`) so a different transport — or a
different middleware entirely, should Zenoh become the right answer — slots in
underneath without disturbing anything above.

## Two alignment rules, and why they differ

This is the subtlety that would silently corrupt every `float64` if you got it
wrong.

**XRCE framing is datagram-relative.** Submessage headers align to 4 bytes
measured from the start of the datagram.

**The ROS message payload restarts at zero.** micro-ROS hands the message
serialiser a *fresh* CDR buffer:

```c
ucdr_init_buffer(ub, ub->iterator, (size_t)(ub->final - ub->iterator));
```

so a message's own alignment is relative to its own first byte, wherever in
the datagram it lands. Had this been datagram-relative, every `float64` in
every message would sit at the wrong offset — and decode *cleanly* as the
wrong number, which is the worst possible failure mode.

## What the Agent does that we do not

- **Discovery and the ROS graph.** The Agent owns it. SnakeROS has no graph
  API and does not need one.
- **The CDR encapsulation header.** There is none on the wire; the Agent adds
  it when forwarding to DDS.
- **DDS QoS negotiation.** We declare QoS in the entity XML; the Agent
  negotiates.

## Protocol notes

Findings that cost real debugging time are written up in
[`NOTES-protocol.md`](https://github.com/kevinmcaleer/snakeros/blob/main/NOTES-protocol.md)
— including the handshake header masking the session id, the missing
encapsulation header, and the CRC polynomial that eProsima's own docs get
wrong.
