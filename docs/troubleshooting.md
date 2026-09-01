---
title: Troubleshooting
---

# Troubleshooting

Written from the bugs actually hit building SnakeROS, not from imagination.
Roughly in the order you are likely to meet them.

## Start here

```console
$ micropython tools/bringup.py          # or run it on the board
```

It works up the stack one layer at a time and stops at the first thing that is
genuinely wrong, instead of leaving you to guess from a silent failure.

---

## Nothing happens, and the Agent says nothing

**The single most common cause: the Agent never saw you, or saw you and
dropped it.** Run the Agent at `-v6` and look for `recv_message`.

- **No `recv_message` at all** — the datagrams are not arriving. Check the
  board can reach the host (`ping` from the board's network), that the Agent
  is on `udp4 --port 8888`, and that Docker is publishing the port
  (`-p 8888:8888/udp`) if you are not on `--net=host`.
- **`recv_message` appears but nothing follows** — the Agent parsed your
  datagram and discarded it. Historically this meant the handshake header was
  wrong (see below).

## "no STATUS_AGENT from Agent"

The session handshake got no reply.

1. Is the Agent actually running and on the port you think?
2. Is the address right? On a board, `agent=` must be your **PC's LAN
   address**, not `127.0.0.1`.
3. Firewall on the host blocking inbound UDP 8888.

There is one non-obvious cause, now fixed in SnakeROS but worth knowing if you
are writing your own client: **`CREATE_CLIENT` must carry `session_id & 0x80`
in the message header, not the full session id.** Send the full id and the
Agent parses your datagram, logs it happily at `-v6`, and silently drops it
because it is looking up a session that does not exist yet. No error, no NACK,
no log line saying why.

## The topic does not appear in `ros2 topic list`

- **Name mangling.** ROS 2 topics are `rt/` + the topic name on the DDS side.
  SnakeROS handles this; if you are constructing entities by hand, check it.
- **The type name.** It must be `pkg::msg::dds_::Name_` exactly — note the
  trailing underscore.
- **Entity creation was rejected.** SnakeROS raises `EntityError` with the
  Agent's status. If you see nothing, check the Agent log for
  `create_datawriter`.

## Messages go to the *wrong* topic

If samples arrive on a topic you published to in a **previous run**, the
Agent has reused a stale entity.

This is what the `REUSE` flag does: it binds a new entity to a matching
existing one. Because object ids restart at 0 on every run while the client
key stays the same, a fresh datawriter can inherit the previous run's topic.
The Agent logs `objects matched` instead of `datawriter created`.

SnakeROS uses `REPLACE` only, so this should not happen. If it does, restart
the Agent to clear its state, and give each board a distinct `key=`.

## Messages stop arriving after a while, with no error

**Almost certainly the `READ_DATA` renewal.** XRCE subscriptions are
request-driven: the Agent delivers up to `max_samples` and then goes quiet.
No error, no log, messages just stop. At 50 Hz a 65535-sample budget runs out
after about 22 minutes — long enough to look like a completely different bug.

SnakeROS renews at 75% of the budget with a 30-second fallback. If you are
seeing this anyway, check that `spin_once()` is actually being called; the
renewal happens there.

## "incompatible QoS ... Last incompatible policy: RELIABILITY"

A ROS 2 subscriber asked for `RELIABLE` and your publisher is `BEST_EFFORT`
(the default).

Either create the publisher with `reliable=True`, or have the subscriber use
best-effort QoS. Note that **`ros2 topic echo` adapts automatically** but an
explicit `rclpy` subscriber does not — which is why `echo` can work while your
own node receives nothing.

Be clear about which reliability you mean: XRCE stream reliability
(board↔Agent) is **independent** of DDS QoS reliability (Agent↔ROS 2).

## `ros2 param` times out but `ros2 service call` works

**You are missing `set_parameters_atomically`.** `ros2`'s
`AsyncParameterClient` waits for *all six* parameter services before it will
talk to a node, and times out with "waiting for parameter services" if any is
absent — even though every other service answers a direct call perfectly.

SnakeROS creates all six. If you are seeing this, check the node name matches:
the services live under `/<node_name>/…`.

## Large messages never arrive

Anything over the MTU has to be fragmented — and **fragments cannot go on a
best-effort stream.** The Agent logs `deserialization error processing
WRITE_DATA` and drops them.

SnakeROS promotes oversized samples to a reliable stream automatically. If you
are hand-rolling, do the same.

## A "50 Hz" timer runs much slower

Check your transport's socket timeout. It bounds how long a single `recv()`
blocks when nothing has arrived, and so puts a floor under `spin_once()`.

SnakeROS defaulted this to 0.1 s early on, which made a 50 Hz timer fire at
10 Hz. It is now 2 ms. If you set `UDPTransport(timeout=...)` yourself, keep
it small and let `poll()` own the waiting.

## The robot keeps driving after the Agent dies

**A UDP publish to a dead Agent succeeds.** The datagram goes into the void
and no exception is ever raised, so Agent loss is *not* detectable by
publishing — a robot whose Agent has died looks perfectly healthy.

Two defences, and you want both:

1. A command timeout in your control loop: no `cmd_vel` recently means stop.
2. `snakeros.board.ResilientNode`, which actively probes with pings and calls
   your `on_disconnect` so you can stop the motors.

## `MemoryError`

See [Memory and performance](memory.md). In short: install the `.mpy` build,
import only the message packs you need, reuse message objects, and drop
reliable streams you are not using.

Fragmentation is nastier than plain exhaustion because it degrades slowly.
`snakeros.board.heap_report()` and `preallocate()` help.

## Values decode as nonsense numbers

Almost always CDR alignment. If you are extending SnakeROS: XRCE framing is
datagram-relative, but the ROS payload restarts alignment at its own first
byte. Getting that backwards misplaces every `float64` and the result decodes
*cleanly* as the wrong value.

`make cdr-diff` diffs the encoder against `rclpy` in both directions and will
catch it.

## Serial: frames are ignored

- The CRC is **CRC-16/ARC** (reflected `0xA001`, init 0), computed over the
  **payload only**. eProsima's documentation says `x^16+x^12+x^5+1` (CCITT);
  that is wrong, and a client that trusts the prose produces frames the Agent
  silently discards.
- Check byte stuffing handles a payload containing `0x7E` **and** `0x7D`.
- `transport.crc_errors` and `transport.resyncs` tell you whether frames are
  arriving corrupted or the stream is out of sync.

## Serial: the REPL fights you

On a Pico the USB CDC port *is* the REPL. See [Transports](transports.md) for
the three ways out; a UART on GPIO pins is the recommended one.
