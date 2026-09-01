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
        self.transport.send(w.bytes())

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
        """Perform the CREATE_CLIENT handshake."""
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
        w = self._begin_message(stream_id)
        rid = self._next_request_id()
        at = self._begin_submessage(w, C.SUBMESSAGE_WRITE_DATA, C.FORMAT_DATA)
        w.raw(struct.pack(">H", rid))
        w.raw(datawriter)
        w.raw(payload)
        self._end_submessage(w, at)
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
