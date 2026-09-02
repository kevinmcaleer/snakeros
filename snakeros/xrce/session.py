"""DDS-XRCE session against a micro-ROS Agent.

The Agent is a stock, unmodified ``micro_ros_agent``: it does not care whether
the client is C, Rust or Python, only that the bytes on the wire are right. So
this module is the whole integration -- there is no bridge and no host-side
software of ours.

Only best-effort streams are implemented here. Reliable streams need
HEARTBEAT/ACKNACK state machines and a bounded retransmit buffer, which cost
real RAM; they live in :mod:`snakeros.xrce.reliable` and are opt-in per
publisher.
"""

import struct
import time

from ..cdr import CDRWriter, CDRReader
from ..errors import EntityError, HandshakeError, SessionError, SessionTimeout
from . import const as C
from .reliable import (
    InputReliableStream,
    OutputReliableStream,
    pack_acknack,
    pack_heartbeat,
    unpack_acknack,
    unpack_heartbeat,
)
from .entities import (
    ObjectIdAllocator,
    object_id,
    parse_object_id,
    participant_xml,
    topic_xml,
    publisher_xml,
    subscriber_xml,
    datawriter_xml,
    datareader_xml,
    requester_xml,
    replier_xml,
)

_ticks_ms = getattr(time, "ticks_ms", None)
if _ticks_ms is None:
    def _now_ms():
        return int(time.time() * 1000)
else:
    def _now_ms():
        return time.ticks_ms()


def _elapsed_ms(start):
    if _ticks_ms is not None:
        return time.ticks_diff(time.ticks_ms(), start)
    return _now_ms() - start


class Session:
    """An XRCE session: handshake, entity lifecycle, and data in both directions."""

    def __init__(self, transport, key=0xAABBCCDD, session_id=0x81, mtu=512):
        self.transport = transport
        self.session_id = session_id
        self.key = struct.pack(">I", key)
        self.mtu = mtu
        self.connected = False

        self._out_seq = {}           # stream_id -> next sequence number
        self._request_counter = 10   # ids below 10 are reserved by the spec
        self._alloc = ObjectIdAllocator()
        self._data_callbacks = {}    # datareader raw id -> callable(bytes)
        self._pending_status = {}    # request_id -> status code
        self._out_reliable = {}      # stream_id -> OutputReliableStream
        self._in_reliable = {}       # stream_id -> InputReliableStream
        self._frag_buf = {}          # stream_id -> bytearray of partial submessage
        self.fragments_sent = 0
        self.fragments_received = 0

    # -- framing -----------------------------------------------------------

    def _header_size(self):
        return (
            C.MAX_HEADER_SIZE
            if self.session_id < C.SESSION_ID_WITHOUT_CLIENT_KEY
            else C.MIN_HEADER_SIZE
        )

    def _next_seq(self, stream_id):
        n = self._out_seq.get(stream_id, 0)
        self._out_seq[stream_id] = (n + 1) & 0xFFFF
        return n

    def _next_request_id(self):
        self._request_counter = (self._request_counter + 1) & 0xFFFF
        if self._request_counter < 10:
            self._request_counter = 10
        return self._request_counter

    def _begin_message(self, stream_id):
        """Start a datagram. Alignment inside is relative to its first byte."""
        w = CDRWriter()
        w.u8(self.session_id)
        w.u8(stream_id)
        w.buf.extend(struct.pack("<H", self._next_seq(stream_id)))
        if self.session_id < C.SESSION_ID_WITHOUT_CLIENT_KEY:
            w.buf.extend(self.key)
        return w

    def _begin_submessage(self, w, sub_id, flags=0):
        """Write a submessage header, returning where to patch its length.

        Submessage headers align to 4 relative to the datagram start, which is
        why the whole message is built in a single writer.
        """
        w.align(4)
        w.u8(sub_id)
        w.u8(flags | C.FLAG_ENDIANNESS)
        length_at = len(w.buf)
        w.buf.extend(b"\x00\x00")
        return length_at

    def _end_submessage(self, w, length_at):
        payload_len = len(w.buf) - (length_at + 2)
        struct.pack_into("<H", w.buf, length_at, payload_len)

    def _send(self, w):
        # Send the writer's bytearray directly. w.bytes() would copy the whole
        # datagram, and on a memory-tight board (an ESP32 has ~35 KB free once
        # WiFi is up) that spare copy is enough to turn a send into ENOMEM.
        self.transport.send(w.buf)

    # -- reliable streams --------------------------------------------------

    def reliable_stream(self, index=0, max_buffered=8):
        """Get (or create) an output reliable stream id.

        Reliable streams are opt-in because they cost RAM: roughly
        ``max_buffered * MTU`` for the retransmit window. See
        :mod:`snakeros.xrce.reliable`.
        """
        sid = C.STREAM_RELIABLE + index
        if sid not in self._out_reliable:
            self._out_reliable[sid] = OutputReliableStream(
                sid, max_buffered=max_buffered
            )
            self._in_reliable[sid] = InputReliableStream(sid)
        return sid

    def _is_reliable(self, stream_id):
        return stream_id >= C.STREAM_RELIABLE

    def _send_reliable(self, stream_id, w, seq):
        stream = self._out_reliable.get(stream_id)
        dg = w.bytes()
        if stream is not None and not stream.add(seq, dg):
            # Window full: flush by asking the Agent where it has got to,
            # rather than silently growing the buffer.
            self._send_heartbeat(stream_id, force=True)
            self.poll(20)
            stream.add(seq, dg)
        self.transport.send(dg)

    def _send_heartbeat(self, stream_id, force=False):
        stream = self._out_reliable.get(stream_id)
        if stream is None:
            return
        rng = stream.heartbeat_range()
        if rng is None:
            return
        now = _now_ms()
        if not force and _elapsed_ms(stream.last_heartbeat) < stream.heartbeat_ms:
            return
        stream.last_heartbeat = now
        w = CDRWriter()
        w.u8(self.session_id)
        w.u8(0)                       # heartbeats ride stream 0
        w.buf.extend(b"\x00\x00")
        if self.session_id < C.SESSION_ID_WITHOUT_CLIENT_KEY:
            w.buf.extend(self.key)
        at = self._begin_submessage(w, C.SUBMESSAGE_HEARTBEAT)
        w.raw(pack_heartbeat(rng[0], rng[1], stream_id))
        self._end_submessage(w, at)
        self.transport.send(w.bytes())

    def _send_acknack(self, stream_id):
        stream = self._in_reliable.get(stream_id)
        if stream is None:
            return
        first, bitmap = stream.compute_acknack()
        w = CDRWriter()
        w.u8(self.session_id)
        w.u8(0)
        w.buf.extend(b"\x00\x00")
        if self.session_id < C.SESSION_ID_WITHOUT_CLIENT_KEY:
            w.buf.extend(self.key)
        at = self._begin_submessage(w, C.SUBMESSAGE_ACKNACK)
        w.raw(pack_acknack(first, bitmap, stream_id))
        self._end_submessage(w, at)
        self.transport.send(w.bytes())

    # -- fragmentation -----------------------------------------------------

    def _max_payload(self):
        return self.mtu - self._header_size() - C.SUBHEADER_SIZE

    def _send_fragmented(self, stream_id, submessage):
        """Split an oversized submessage across FRAGMENT submessages.

        Each fragment is its own datagram; the last carries LAST_FRAGMENT so
        the Agent knows to reassemble and process.
        """
        chunk = self._max_payload()
        total = len(submessage)
        off = 0
        while off < total:
            piece = submessage[off:off + chunk]
            off += len(piece)
            last = off >= total
            w = self._begin_message(stream_id)
            at = self._begin_submessage(
                w, C.SUBMESSAGE_FRAGMENT,
                C.FLAG_LAST_FRAGMENT if last else 0,
            )
            w.raw(piece)
            self._end_submessage(w, at)
            self.fragments_sent += 1
            if self._is_reliable(stream_id):
                self._send_reliable(stream_id, w, self._out_reliable[stream_id].take_seq())
            else:
                self._send(w)

    # -- receive -----------------------------------------------------------

    def _parse(self, data):
        """Split a datagram into ``(sub_id, flags, payload_bytes)`` tuples."""
        if data is None or len(data) < C.MIN_HEADER_SIZE:
            return []
        session_id = data[0]
        pos = (
            C.MAX_HEADER_SIZE
            if session_id < C.SESSION_ID_WITHOUT_CLIENT_KEY
            else C.MIN_HEADER_SIZE
        )
        out = []
        n = len(data)
        while pos + C.SUBHEADER_SIZE <= n:
            pos += (-pos) % 4                     # subheaders are 4-aligned
            if pos + C.SUBHEADER_SIZE > n:
                break
            sub_id = data[pos]
            flags = data[pos + 1]
            length = struct.unpack_from("<H", data, pos + 2)[0]
            pos += C.SUBHEADER_SIZE
            if pos + length > n:
                length = n - pos                  # tolerate a truncated tail
            out.append((sub_id, flags, data[pos:pos + length]))
            pos += length
        return out

    def poll(self, timeout_ms=0):
        """Read and dispatch whatever has arrived. Returns submessages seen."""
        for sid in self._out_reliable:
            self._send_heartbeat(sid)
        deadline = _now_ms()
        seen = []
        while True:
            data = self.transport.recv()
            if data is None:
                if _elapsed_ms(deadline) >= timeout_ms:
                    return seen
                continue
            for sub_id, flags, payload in self._parse(data):
                seen.append((sub_id, flags, payload))
                self._dispatch(sub_id, flags, payload)
            if _elapsed_ms(deadline) >= timeout_ms:
                return seen

    def _dispatch(self, sub_id, flags, payload):
        if sub_id == C.SUBMESSAGE_STATUS:
            # BaseObjectReply: related_request(request_id 2 + object_id 2) + result(2)
            if len(payload) >= 6:
                rid = (payload[0] << 8) | payload[1]
                self._pending_status[rid] = payload[4]
        elif sub_id == C.SUBMESSAGE_ACKNACK:
            parsed = unpack_acknack(payload)
            if parsed is not None:
                first, bitmap, sid = parsed
                stream = self._out_reliable.get(sid)
                if stream is not None:
                    for dg in stream.on_acknack(first, bitmap):
                        self.transport.send(dg)
        elif sub_id == C.SUBMESSAGE_HEARTBEAT:
            parsed = unpack_heartbeat(payload)
            if parsed is not None:
                _first, _last, sid = parsed
                if sid in self._in_reliable:
                    self._send_acknack(sid)
        elif sub_id == C.SUBMESSAGE_DATA:
            # BaseObjectRequest(request_id 2 + object_id 2) then the sample.
            if len(payload) >= 4:
                reader_raw = bytes(payload[2:4])
                cb = self._data_callbacks.get(reader_raw)
                if cb is not None:
                    cb(payload[4:])

    def _wait_status(self, request_id, timeout_ms=1000):
        start = _now_ms()
        while _elapsed_ms(start) < timeout_ms:
            if request_id in self._pending_status:
                return self._pending_status.pop(request_id)
            self.poll(10)
        return None

    # -- lifecycle ---------------------------------------------------------

    def connect(self, timeout_ms=3000, retries=5):
        """Perform the CREATE_CLIENT handshake.

        Closes the transport on **any** failure. Leaking the socket here is
        not a tidiness issue: lwIP on an ESP32 allows only a handful of
        concurrent sockets, so a caller that retries a failed connect
        exhausts them within a few attempts and every subsequent send fails
        with ENOMEM -- which looks like a memory problem and is really a
        descriptor leak.
        """
        try:
            return self._connect(timeout_ms, retries)
        except Exception:
            try:
                self.transport.close()
            except Exception:
                pass
            raise

    def _connect(self, timeout_ms, retries):
        self.transport.open()
        for _ in range(retries):
            w = CDRWriter()
            # The handshake header carries the session id *masked* with
            # SESSION_ID_WITHOUT_CLIENT_KEY, not the full id -- see
            # uxr_stamp_create_session_header. Send the full id here and the
            # Agent looks up a session that does not exist yet and drops the
            # datagram without a word of complaint.
            w.u8(self.session_id & C.SESSION_ID_WITHOUT_CLIENT_KEY)
            w.u8(0)                                # session establishment: stream 0
            w.buf.extend(b"\x00\x00")              # sequence 0
            if self.session_id < C.SESSION_ID_WITHOUT_CLIENT_KEY:
                w.buf.extend(self.key)
            at = self._begin_submessage(w, C.SUBMESSAGE_CREATE_CLIENT)
            w.raw(C.XRCE_COOKIE)                   # 4 bytes
            w.raw(C.XRCE_VERSION)                  # 2 bytes
            w.raw(C.VENDOR_ID_EPROSIMA)            # 2 bytes
            w.raw(self.key)                        # 4 bytes
            w.u8(self.session_id)
            w.boolean(False)                       # optional_properties
            w.u16(self.mtu)
            self._end_submessage(w, at)
            self._send(w)

            start = _now_ms()
            while _elapsed_ms(start) < timeout_ms // retries + 200:
                for sub_id, _flags, payload in self.poll(50):
                    if sub_id == C.SUBMESSAGE_STATUS_AGENT:
                        status = payload[0] if payload else 0xFF
                        if status in (C.STATUS_OK, C.STATUS_OK_MATCHED):
                            self.connected = True
                            return self
                        raise HandshakeError(
                            "Agent refused session: " + C.status_name(status)
                        )
        raise SessionTimeout(
            "no STATUS_AGENT from Agent at {}:{}".format(
                getattr(self.transport, "host", "?"),
                getattr(self.transport, "port", "?"),
            )
        )

    def close(self):
        """Delete the session and close the transport."""
        if self.connected:
            try:
                w = self._begin_message(0)
                at = self._begin_submessage(w, C.SUBMESSAGE_DELETE)
                w.raw(b"\x00\x00")                 # request id 0 (logout)
                w.raw(C.OBJECTID_CLIENT)
                self._end_submessage(w, at)
                self._send(w)
            except Exception:
                pass
            self.connected = False
        self.transport.close()

    def ping(self, timeout_ms=500):
        """GET_INFO liveness probe. True if the Agent answered."""
        w = self._begin_message(C.STREAM_BEST_EFFORT)
        rid = self._next_request_id()
        at = self._begin_submessage(w, C.SUBMESSAGE_GET_INFO)
        w.raw(struct.pack(">H", rid))
        w.raw(C.OBJECTID_CLIENT)
        w.u32(0x0A)                                # info mask: activity + config
        self._end_submessage(w, at)
        self._send(w)
        start = _now_ms()
        while _elapsed_ms(start) < timeout_ms:
            for sub_id, _f, _p in self.poll(50):
                if sub_id in (C.SUBMESSAGE_INFO, C.SUBMESSAGE_STATUS):
                    return True
        return False

    # -- entity creation ---------------------------------------------------

    def _create(self, obj_raw, kind, build_representation, flags=None, timeout_ms=2000):
        """Send CREATE and wait for a STATUS that isn't an error."""
        if flags is None:
            # REPLACE only -- deliberately *not* REUSE. With REUSE the Agent
            # binds a new entity to a matching existing one, and because
            # object ids restart at 0 on every run while the client key stays
            # the same, a fresh datawriter silently inherits the previous
            # run's topic. The symptom is samples arriving on the topic you
            # published to *last* time, with no error anywhere.
            flags = C.FLAG_REPLACE
        w = self._begin_message(C.STREAM_BEST_EFFORT)
        rid = self._next_request_id()
        at = self._begin_submessage(w, C.SUBMESSAGE_CREATE, flags)
        w.raw(struct.pack(">H", rid))
        w.raw(obj_raw)
        w.u8(kind)
        build_representation(w)
        self._end_submessage(w, at)
        self._send(w)

        status = self._wait_status(rid, timeout_ms)
        if status is None:
            raise EntityError(
                "no STATUS for CREATE of kind 0x{:02x} (Agent silent)".format(kind)
            )
        if status not in (C.STATUS_OK, C.STATUS_OK_MATCHED):
            raise EntityError(
                "Agent rejected entity kind 0x{:02x}: {}".format(
                    kind, C.status_name(status)
                ),
                status,
            )
        return obj_raw

    def create_participant(self, name="snakeros_node", domain_id=0):
        raw = self._alloc.alloc(C.OBJK_PARTICIPANT)
        xml = participant_xml(name)

        def rep(w):
            w.u8(C.REPRESENTATION_AS_XML_STRING)
            w.string(xml)
            w.i16(domain_id)

        return self._create(raw, C.OBJK_PARTICIPANT, rep)

    def create_topic(self, participant, dds_name, type_name):
        raw = self._alloc.alloc(C.OBJK_TOPIC)
        xml = topic_xml(dds_name, type_name)

        def rep(w):
            w.u8(C.REPRESENTATION_AS_XML_STRING)
            w.string(xml)
            w.raw(participant)

        return self._create(raw, C.OBJK_TOPIC, rep)

    def create_publisher(self, participant):
        raw = self._alloc.alloc(C.OBJK_PUBLISHER)
        xml = publisher_xml()

        def rep(w):
            w.u8(C.REPRESENTATION_AS_XML_STRING)
            w.string(xml)
            w.raw(participant)

        return self._create(raw, C.OBJK_PUBLISHER, rep)

    def create_subscriber(self, participant):
        raw = self._alloc.alloc(C.OBJK_SUBSCRIBER)
        xml = subscriber_xml()

        def rep(w):
            w.u8(C.REPRESENTATION_AS_XML_STRING)
            w.string(xml)
            w.raw(participant)

        return self._create(raw, C.OBJK_SUBSCRIBER, rep)

    def create_datawriter(self, publisher, dds_name, type_name,
                          reliable=False, history_depth=None):
        raw = self._alloc.alloc(C.OBJK_DATAWRITER)
        xml = datawriter_xml(dds_name, type_name, reliable, history_depth)

        def rep(w):
            w.u8(C.REPRESENTATION_AS_XML_STRING)
            w.string(xml)
            w.raw(publisher)

        return self._create(raw, C.OBJK_DATAWRITER, rep)

    def create_datareader(self, subscriber, dds_name, type_name,
                          reliable=False, history_depth=None):
        raw = self._alloc.alloc(C.OBJK_DATAREADER)
        xml = datareader_xml(dds_name, type_name, reliable, history_depth)

        def rep(w):
            w.u8(C.REPRESENTATION_AS_XML_STRING)
            w.string(xml)
            w.raw(subscriber)

        return self._create(raw, C.OBJK_DATAREADER, rep)

    def create_requester(self, participant, service_name, req_type, res_type):
        raw = self._alloc.alloc(C.OBJK_REQUESTER)
        xml = requester_xml(service_name, req_type, res_type)

        def rep(w):
            w.u8(C.REPRESENTATION_AS_XML_STRING)
            w.string(xml)
            w.raw(participant)

        return self._create(raw, C.OBJK_REQUESTER, rep)

    def create_replier(self, participant, service_name, req_type, res_type):
        raw = self._alloc.alloc(C.OBJK_REPLIER)
        xml = replier_xml(service_name, req_type, res_type)

        def rep(w):
            w.u8(C.REPRESENTATION_AS_XML_STRING)
            w.string(xml)
            w.raw(participant)

        return self._create(raw, C.OBJK_REPLIER, rep)

    def write_reply(self, replier, sample_identity, payload,
                    stream_id=C.STREAM_RELIABLE):
        """Reply to a service request.

        The 24-byte SampleIdentity from the incoming request is echoed back
        verbatim; it is how the Agent routes the reply to the right caller.
        """
        if stream_id >= C.STREAM_RELIABLE and stream_id not in self._out_reliable:
            self.reliable_stream(stream_id - C.STREAM_RELIABLE)
        w = self._begin_message(stream_id)
        rid = self._next_request_id()
        at = self._begin_submessage(w, C.SUBMESSAGE_WRITE_DATA, C.FORMAT_DATA)
        w.raw(struct.pack(">H", rid))
        w.raw(replier)
        w.raw(sample_identity)
        w.raw(payload)
        self._end_submessage(w, at)
        if stream_id >= C.STREAM_RELIABLE:
            self._send_reliable(stream_id, w, self._out_seq.get(stream_id, 1) - 1)
        else:
            self._send(w)
        return rid

    def delete(self, obj_raw, timeout_ms=1000):
        w = self._begin_message(C.STREAM_BEST_EFFORT)
        rid = self._next_request_id()
        at = self._begin_submessage(w, C.SUBMESSAGE_DELETE)
        w.raw(struct.pack(">H", rid))
        w.raw(obj_raw)
        self._end_submessage(w, at)
        self._send(w)
        self._wait_status(rid, timeout_ms)

    # -- data --------------------------------------------------------------

    def write_data(self, datawriter, payload, stream_id=C.STREAM_BEST_EFFORT):
        """Publish one pre-serialised sample through a datawriter.

        ``payload`` is an opaque CDR blob. Its internal alignment restarts at
        its own first byte -- micro-ROS gives the message serialiser a fresh
        CDR buffer here, so we append it as raw octets and never let the
        datagram's alignment leak into it.
        """
        rid = self._next_request_id()

        # Oversized samples go out as FRAGMENT submessages. Build the whole
        # WRITE_DATA submessage first so it can be split without having to
        # know the fragment boundaries in advance.
        if C.SUBHEADER_SIZE + 4 + len(payload) > self._max_payload():
            # Fragmentation requires a *reliable* stream: reassembly needs
            # guaranteed in-order delivery, and the C client rejects
            # best-effort outright (see uxr_prepare_output_stream_fragmented).
            # Sending FRAGMENTs on a best-effort stream makes the Agent log
            # "deserialization error processing WRITE_DATA" and drop them, so
            # promote transparently rather than fail.
            if not self._is_reliable(stream_id):
                stream_id = self.reliable_stream()
            elif stream_id not in self._out_reliable:
                self.reliable_stream(stream_id - C.STREAM_RELIABLE)
            sub = CDRWriter()
            at = self._begin_submessage(sub, C.SUBMESSAGE_WRITE_DATA, C.FORMAT_DATA)
            sub.raw(struct.pack(">H", rid))
            sub.raw(datawriter)
            sub.raw(payload)
            self._end_submessage(sub, at)
            self._send_fragmented(stream_id, sub.bytes())
            return rid

        reliable = self._is_reliable(stream_id)
        if reliable and stream_id not in self._out_reliable:
            self.reliable_stream(stream_id - C.STREAM_RELIABLE)
        w = self._begin_message(stream_id)
        at = self._begin_submessage(w, C.SUBMESSAGE_WRITE_DATA, C.FORMAT_DATA)
        w.raw(struct.pack(">H", rid))
        w.raw(datawriter)
        w.raw(payload)
        self._end_submessage(w, at)
        if reliable:
            self._send_reliable(stream_id, w, self._out_seq.get(stream_id, 1) - 1)
        else:
            self._send(w)
        return rid

    def request_read(self, datareader, max_samples=0xFFFF,
                     stream_id=C.STREAM_BEST_EFFORT):
        """Issue a standing READ_DATA request.

        XRCE subscriptions are request-driven: the Agent delivers up to
        ``max_samples`` and then stops. Leave this un-renewed and messages
        silently cease after N samples, which is a miserable bug to chase in
        the field -- :class:`snakeros.node.Subscription` renews it.
        """
        w = self._begin_message(stream_id)
        rid = self._next_request_id()
        at = self._begin_submessage(w, C.SUBMESSAGE_READ_DATA)
        w.raw(struct.pack(">H", rid))
        w.raw(datareader)
        # ReadSpecification
        w.u8(stream_id)                 # preferred_stream_id
        w.u8(C.FORMAT_DATA)             # data_format
        w.boolean(False)                # optional_content_filter_expression
        w.boolean(True)                 # optional_delivery_control
        w.u16(max_samples)              # max_samples
        w.u16(0)                        # max_elapsed_time
        w.u16(0)                        # max_bytes_per_second
        w.u16(0)                        # min_pace_period
        self._end_submessage(w, at)
        self._send(w)
        return rid

    def on_data(self, datareader, callback):
        self._data_callbacks[bytes(datareader)] = callback
