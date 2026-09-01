"""The public SnakeROS API.

Familiar names, MicroPython idiom. Anyone who has written ``rclpy`` should be
able to read a SnakeROS script without looking anything up::

    from snakeros import Node
    from snakeros.msg.geometry_msgs import Twist

    node = Node('pico_node', agent='192.168.1.10')
    pub = node.create_publisher(Twist, 'cmd_vel')
    node.create_subscription(Twist, 'cmd_vel', on_cmd)
    node.create_timer(0.1, tick)
    node.spin()

What it deliberately does *not* reimplement: executors, callback groups and
the rclpy lifecycle. That machinery buys nothing on a microcontroller and
costs RAM that a Pico W does not have.
"""

import time

from .errors import SnakeROSError
from .transport import UDPTransport
from .xrce import const as C
from .xrce.entities import dds_type_name, mangle_topic
from .xrce.session import Session

try:
    import asyncio
except ImportError:  # pragma: no cover - asyncio is present on all modern ports
    asyncio = None

_ticks_ms = getattr(time, "ticks_ms", None)
if _ticks_ms is None:
    def _now_ms():
        return int(time.time() * 1000)

    def _diff_ms(a, b):
        return a - b
else:
    def _now_ms():
        return time.ticks_ms()

    def _diff_ms(a, b):
        return time.ticks_diff(a, b)


def _sleep_ms(ms):
    time.sleep(ms / 1000.0)


class Publisher:
    """A ROS 2 publisher backed by an XRCE datawriter."""

    def __init__(self, node, msg_type, topic, datawriter, reliable):
        self.node = node
        self.msg_type = msg_type
        self.topic = topic
        self._dw = datawriter
        self.reliable = reliable
        # Reliability is per-publisher so you pay the retransmit-window RAM
        # only where you need it: an e-stop, yes; a 50 Hz IMU feed, no.
        self._stream = (
            node._session.reliable_stream() if reliable else C.STREAM_BEST_EFFORT
        )

    def publish(self, msg):
        """Serialise and publish one message."""
        if not isinstance(msg, self.msg_type):
            raise TypeError(
                "publisher for {} was given a {}".format(
                    self.msg_type.ros_name(), type(msg).__name__
                )
            )
        self.node._session.write_data(self._dw, msg.serialize(), self._stream)


class Subscription:
    """A ROS 2 subscription backed by an XRCE datareader.

    XRCE subscriptions are *request-driven*: the Agent delivers up to
    ``max_samples`` and then goes quiet. That is the single nastiest failure
    mode in this protocol -- everything works for twenty minutes and then
    messages stop with no error anywhere -- so the request is renewed well
    before the budget runs out.
    """

    def __init__(self, node, msg_type, topic, callback, datareader,
                 max_samples=8192, renew_at=0.75, renew_ms=30000):
        self.node = node
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self._dr = datareader
        self._max_samples = max_samples
        self._renew_after = int(max_samples * renew_at)
        self._renew_ms = renew_ms
        self._count = 0
        self._last_request = 0
        self.received = 0
        self.errors = 0
        node._session.on_data(datareader, self._on_data)
        self._request()

    def _request(self):
        self.node._session.request_read(self._dr, self._max_samples)
        self._count = 0
        self._last_request = _now_ms()

    def _on_data(self, payload):
        self._count += 1
        self.received += 1
        try:
            msg = self.msg_type.deserialize(payload)
        except Exception:
            # A malformed sample must not take down the spin loop.
            self.errors += 1
            return
        if self.callback is not None:
            self.callback(msg)

    def maintain(self):
        """Renew the standing READ_DATA request before the Agent stops."""
        if self._count >= self._renew_after:
            self._request()
        elif _diff_ms(_now_ms(), self._last_request) >= self._renew_ms:
            self._request()


class Timer:
    """A periodic callback that does not accumulate drift."""

    def __init__(self, period_s, callback):
        self.period_ms = int(period_s * 1000)
        self.callback = callback
        self._next = _now_ms() + self.period_ms
        self.cancelled = False

    def cancel(self):
        self.cancelled = True

    def check(self, now=None):
        if self.cancelled:
            return False
        if now is None:
            now = _now_ms()
        if _diff_ms(now, self._next) >= 0:
            # Advance from the scheduled time, not from now, so a late tick
            # does not push every later tick late as well.
            self._next += self.period_ms
            if _diff_ms(now, self._next) > 0:
                # We fell far enough behind that catching up is pointless.
                self._next = now + self.period_ms
            self.callback()
            return True
        return False


class Node:
    """A ROS 2 node running on MicroPython."""

    def __init__(self, name, agent="127.0.0.1", port=8888, domain_id=0,
                 transport=None, mtu=512, key=0xAABBCCDD, connect=True,
                 namespace=""):
        self.name = name
        self.namespace = namespace
        self.domain_id = domain_id
        self._transport = transport or UDPTransport(agent, port, mtu=mtu)
        self._session = Session(self._transport, key=key, mtu=mtu)
        self._participant = None
        self._publisher_entity = None
        self._subscriber_entity = None
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self._services = []
        self._clients = []
        self._params = None
        self.connected = False
        if connect:
            self.connect()

    # -- lifecycle ---------------------------------------------------------

    def connect(self, timeout_ms=3000, retries=5):
        self._session.connect(timeout_ms=timeout_ms, retries=retries)
        self._participant = self._session.create_participant(
            self.name, self.domain_id
        )
        self.connected = True
        return self

    def _ensure_publisher_entity(self):
        if self._publisher_entity is None:
            self._publisher_entity = self._session.create_publisher(self._participant)
        return self._publisher_entity

    def _ensure_subscriber_entity(self):
        if self._subscriber_entity is None:
            self._subscriber_entity = self._session.create_subscriber(self._participant)
        return self._subscriber_entity

    def destroy(self):
        """Tear the node down and close the session."""
        try:
            self._session.close()
        finally:
            self.connected = False

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.destroy()

    # -- entities ----------------------------------------------------------

    def _resolve(self, topic):
        if self.namespace and not topic.startswith("/"):
            return "/" + self.namespace.strip("/") + "/" + topic
        return topic

    def create_publisher(self, msg_type, topic, reliable=False, history_depth=None):
        topic = self._resolve(topic)
        dds_name = mangle_topic(topic)
        type_name = msg_type.type_name()
        self._session.create_topic(self._participant, dds_name, type_name)
        pubent = self._ensure_publisher_entity()
        dw = self._session.create_datawriter(
            pubent, dds_name, type_name, reliable, history_depth
        )
        p = Publisher(self, msg_type, topic, dw, reliable)
        self.publishers.append(p)
        return p

    def create_subscription(self, msg_type, topic, callback, reliable=False,
                            history_depth=None, max_samples=8192):
        topic = self._resolve(topic)
        dds_name = mangle_topic(topic)
        type_name = msg_type.type_name()
        self._session.create_topic(self._participant, dds_name, type_name)
        subent = self._ensure_subscriber_entity()
        dr = self._session.create_datareader(
            subent, dds_name, type_name, reliable, history_depth
        )
        s = Subscription(self, msg_type, topic, callback, dr,
                         max_samples=max_samples)
        self.subscriptions.append(s)
        return s

    def create_service(self, srv_type, service_name, handler):
        """Host a service on the board."""
        from .services import Service

        svc = Service(self, srv_type, service_name, handler)
        self._services.append(svc)
        return svc

    def create_client(self, srv_type, service_name):
        """Call a service hosted elsewhere in the graph."""
        from .services import ServiceClient

        cli = ServiceClient(self, srv_type, service_name)
        self._clients.append(cli)
        return cli

    def create_timer(self, period_s, callback):
        t = Timer(period_s, callback)
        self.timers.append(t)
        return t

    # -- spinning ----------------------------------------------------------

    def spin_once(self, timeout_ms=10):
        """Service the transport, timers and subscription upkeep once."""
        self._session.poll(timeout_ms)
        now = _now_ms()
        for t in self.timers:
            t.check(now)
        for s in self.subscriptions:
            s.maintain()
        for svc in self._services:
            svc.maintain()

    def spin(self, timeout_ms=10):
        """Block forever, servicing callbacks. Ctrl-C exits cleanly."""
        try:
            while True:
                self.spin_once(timeout_ms)
        except KeyboardInterrupt:
            pass

    async def spin_async(self, timeout_ms=5, idle_ms=2):
        """Cooperative spin, so SnakeROS can share the loop with your own tasks.

        SnakeROS does not need to own ``main()``: run this as one task
        alongside your motor control, sensor polling or web server.
        """
        if asyncio is None:
            raise SnakeROSError("asyncio unavailable on this port")
        while True:
            self.spin_once(timeout_ms)
            await asyncio.sleep_ms(idle_ms) if hasattr(asyncio, "sleep_ms") \
                else await asyncio.sleep(idle_ms / 1000.0)

    # -- health ------------------------------------------------------------

    def ping(self, timeout_ms=500):
        return self._session.ping(timeout_ms)
