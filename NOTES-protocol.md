# Protocol findings

Empirical results from bringing up the client against a stock
`microros/micro-ros-agent:jazzy`. Recorded because each of these cost real
debugging time and none is obvious from the documentation.

## 1. The handshake header masks the session id

`CREATE_CLIENT` must carry `session_id & 0x80` in the **message header**, not
the full session id. With session `0x81` the header byte is `0x80`.

Send the full id and the Agent parses the datagram correctly, logs it at
verbosity 6 — and then silently drops it, because it looks up a session that
does not exist yet. There is no error, no NACK, and no log line saying why.

Source: `uxr_stamp_create_session_header` in `session_info.c`:

```c
uxr_serialize_message_header(&ub, info->id & SESSION_ID_WITHOUT_CLIENT_KEY, 0, 0, info->key);
```

Subsequent messages use the full session id normally.

## 2. No CDR encapsulation header on the wire

The ROS message payload inside `WRITE_DATA` is **raw CDR with no 4-byte
encapsulation header**. The Agent adds it when forwarding to DDS.

Verified both ways: without the header `ros2 topic echo` prints correct data;
this is the variant the C client produces too.

## 3. ROS payload alignment restarts at the payload

CDR alignment inside the ROS message is relative to the **start of the
message**, not the start of the datagram. `uxr_prepare_output_stream` calls

```c
ucdr_init_buffer(ub, ub->iterator, (size_t)(ub->final - ub->iterator));
```

which gives the message serialiser a fresh origin. Had this been
datagram-relative, every `float64` in every message would have landed at the
wrong offset — decoding cleanly as the wrong number, which is the worst
possible failure mode.

XRCE's own framing (submessage headers) *is* datagram-relative and aligns to 4.

## 4. Entity XML, verbatim

Matched to `rmw_microxrcedds`' own templates so the Agent sees what it would
see from a C client:

- participant: `<dds><participant><rtps><name>NAME</name></rtps></participant></dds>`
- topic: `<dds><topic><name>rt/chatter</name><dataType>std_msgs::msg::dds_::String_</dataType></topic></dds>`
- publisher / subscriber: **empty string**, with `REUSE|REPLACE` flags
- data_writer / data_reader: `<dds><data_writer>…<qos><reliability><kind>…` etc.

Type names are `<pkg>::<msg|srv>::dds_::<Name>_`; topic names are `rt` + the
leading-slash topic. Services use `rq`/`rr` prefixes with `Request`/`Reply`
suffixes.

## 5. Object ids are 12-bit, scoped by kind

`raw[0] = id >> 4`, `raw[1] = ((id & 0x0F) << 4) | kind`. A participant and a
topic may share numeric id 0 without colliding.

## 6. Request ids are big-endian

`request_id.data[0] = request_id >> 8`. Unlike almost everything else on the
wire, which is little-endian.
