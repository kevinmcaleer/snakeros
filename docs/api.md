---
title: API reference
---

# API reference

## `snakeros.Node`

```python
Node(name, agent="127.0.0.1", port=8888, domain_id=0, transport=None,
     mtu=512, key=0xAABBCCDD, connect=True, namespace="")
```

Connects on construction unless `connect=False`.

| Argument | |
|---|---|
| `name` | Node name in the ROS graph. Parameter services live under `/<name>/`. |
| `agent` | Address of the micro-ROS Agent. On a board this is your PC's LAN address. |
| `transport` | Supply a `SerialTransport` or `ByteStreamTransport` to use something other than UDP. |
| `key` | XRCE client key. **Give each board a distinct value** — two boards sharing a key fight over entities on the Agent. |
| `mtu` | Datagram size. Larger samples are fragmented automatically. |

### Methods

| | |
|---|---|
| `create_publisher(msg_type, topic, reliable=False, history_depth=None)` | → `Publisher` |
| `create_subscription(msg_type, topic, callback, reliable=False, max_samples=8192)` | → `Subscription` |
| `create_timer(period_s, callback)` | → `Timer` |
| `create_service(srv_type, name, handler)` | → `Service`; `handler(request) -> response` |
| `create_client(srv_type, name)` | → `ServiceClient` |
| `declare_parameter(name, default, description="", minimum=None, maximum=None, read_only=False, callback=None)` | Stands up the parameter services on first call |
| `get_parameter(name, default=None)` / `set_parameter(name, value)` | |
| `spin_once(timeout_ms=10)` | Service transport, timers and subscription upkeep once |
| `spin(timeout_ms=10)` | Block forever; `Ctrl-C` exits cleanly |
| `await spin_async(timeout_ms=5, idle_ms=2)` | Cooperative spin — SnakeROS need not own `main()` |
| `ping(timeout_ms=500)` | → `bool`. The only reliable way to detect Agent loss over UDP |
| `destroy()` | Close the session. `Node` is also a context manager |

## `Publisher`

`publish(msg)` — serialises and sends. Type-checked against the publisher's
`msg_type`, because passing a `Twist` to a `String` publisher otherwise
produces a confusing CDR error rather than an obvious one.

`reliable=True` costs about 4.4 KB for the retransmit window. Use it for
commands that must arrive, not for a 50 Hz sensor feed.

## `Subscription`

`callback(msg)` is called from `spin_once()`. Attributes `received` and
`errors` are exposed for soak testing. A malformed sample increments `errors`
and is dropped rather than taking down the spin loop.

## `Timer`

`cancel()`. Timers advance from their scheduled time rather than from now, so
a late tick does not push every later tick late.

## `Service` and `ServiceClient`

```python
node.create_service(SetBool, 'arm_motors', handler)   # handler(req) -> res

cli = node.create_client(SetBool, 'arm_motors')
res = cli.call(SetBool.Request(data=True), timeout_ms=2000)  # raises ServiceTimeout
cli.call_nowait(req); ...; res = cli.take_reply()            # non-blocking
```

Services ride a reliable stream. A handler that raises is caught and counted
rather than killing the spin loop.

## `snakeros.board`

| | |
|---|---|
| `connect_wifi(ssid, password, timeout_s=20, hostname=None)` | → IP. Raises rather than returning a half-open interface |
| `wifi_connected()` | → `bool` |
| `heap_report(label="")` | Print and return free heap |
| `preallocate(node)` | Warm the allocator so the first GC does not land inside your control loop |
| `ResilientNode(factory, setup=, on_disconnect=, liveness_s=3.0, max_missed_pings=2)` | Survives Agent loss; **calls `on_disconnect` so you can stop the motors** |

## Transports

```python
UDPTransport(host, port=8888, mtu=512, timeout=0.002)
SerialTransport(uart=None, device=None, mtu=512)   # machine.UART or a device path
ByteStreamTransport(stream, mtu=512)               # any read/write/close object
TCPByteStream(host, port)                          # serial tunnelled over TCP
```

All four implement the same contract: `open`, `send`, `recv`, `close`, `mtu`.

## Exceptions

`SnakeROSError` is the base. `TransportError`, `SessionError`
(`HandshakeError`, `SessionTimeout`), `EntityError`, `CDRError`
(`CDRTruncated`), `ServiceError` (`ServiceTimeout`), `ParameterError`,
`MessageDefinitionError`.

The hierarchy is layered deliberately so a failure says *which* layer broke.
