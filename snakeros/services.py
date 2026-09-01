"""ROS 2 services: request/response in both directions.

Lets a board be *asked* to do something and report whether it worked -- arm
the motors, calibrate the IMU, reset odometry -- rather than only shouting
sensor data into the graph.

Services ride a **reliable** stream by default. A service call that silently
vanishes is worse than one that fails loudly.
"""

import time

from .errors import ServiceError, ServiceTimeout
from .xrce import const as C

SAMPLE_IDENTITY_SIZE = 24
RELATED_REQUEST_SIZE = 4

_ticks_ms = getattr(time, "ticks_ms", None)
if _ticks_ms is None:
    def _now_ms():
        return int(time.time() * 1000)

    def _elapsed_ms(start):
        return _now_ms() - start
else:
    def _now_ms():
        return time.ticks_ms()

    def _elapsed_ms(start):
        return time.ticks_diff(time.ticks_ms(), start)


class Service:
    """A service *server*: handles incoming requests on the board.

    ``handler(request) -> response``. Raising inside the handler is caught and
    counted rather than being allowed to kill the spin loop -- a robot that
    stops driving because a diagnostic service threw is a bad trade.
    """

    def __init__(self, node, srv_type, service_name, handler):
        self.node = node
        self.srv_type = srv_type
        self.service_name = (
            service_name if service_name.startswith("/") else "/" + service_name
        )
        self.handler = handler
        self.handled = 0
        self.errors = 0
        session = node._session
        self._replier = session.create_replier(
            node._participant,
            self.service_name,
            srv_type.Request.type_name(),
            srv_type.Response.type_name(),
        )
        session.on_data(self._replier, self._on_request)
        session.request_read(self._replier, 0xFFFF)
        self._last_request = _now_ms()

    def _on_request(self, payload):
        # An incoming request is a 24-byte SampleIdentity followed by the
        # serialised request. The identity must be echoed back verbatim so
        # the Agent can route the reply to the caller.
        if len(payload) < SAMPLE_IDENTITY_SIZE:
            self.errors += 1
            return
        identity = payload[:SAMPLE_IDENTITY_SIZE]
        body = payload[SAMPLE_IDENTITY_SIZE:]
        try:
            request = self.srv_type.Request.deserialize(body)
            response = self.handler(request)
        except Exception:
            self.errors += 1
            return
        if response is None:
            response = self.srv_type.Response()
        self.node._session.write_reply(
            self._replier, identity, response.serialize()
        )
        self.handled += 1

    def maintain(self, renew_ms=30000):
        if _elapsed_ms(self._last_request) >= renew_ms:
            self.node._session.request_read(self._replier, 0xFFFF)
            self._last_request = _now_ms()


class ServiceClient:
    """A service *client*: call a service hosted elsewhere in the graph."""

    def __init__(self, node, srv_type, service_name):
        self.node = node
        self.srv_type = srv_type
        self.service_name = (
            service_name if service_name.startswith("/") else "/" + service_name
        )
        self._replies = []
        session = node._session
        self._requester = session.create_requester(
            node._participant,
            self.service_name,
            srv_type.Request.type_name(),
            srv_type.Response.type_name(),
        )
        session.on_data(self._requester, self._on_reply)
        session.request_read(self._requester, 0xFFFF)

    def _on_reply(self, payload):
        self._replies.append(payload)

    def _decode(self, payload):
        """Strip the related-request header and decode the response.

        The two directions are asymmetric, which is not documented anywhere
        and cost a debugging cycle to establish:

        * a **replier** receives a 24-byte ``SampleIdentity`` before the
          request body (echoed back verbatim so the Agent can route the reply)
        * a **requester** receives a 4-byte ``BaseObjectRequest`` -- the
          request id and the requester's own object id -- before the response
          body, so concurrent calls can be correlated

        Decode at the expected offset, but fall back rather than raising if an
        Agent version differs.
        """
        for offset in (RELATED_REQUEST_SIZE, 0, SAMPLE_IDENTITY_SIZE):
            if len(payload) < offset:
                continue
            try:
                return self.srv_type.Response.deserialize(payload[offset:])
            except Exception:
                continue
        raise ServiceError(
            "could not decode a {} reply from {} bytes".format(
                self.srv_type.Response.__name__, len(payload)
            )
        )

    def call(self, request, timeout_ms=2000):
        """Send a request and block for the reply.

        Raises :class:`~snakeros.errors.ServiceTimeout` rather than hanging
        the spin loop for ever if the far side never answers.
        """
        self._replies = []
        self.node._session.write_data(
            self._requester, request.serialize(), C.STREAM_RELIABLE
        )
        start = _now_ms()
        while _elapsed_ms(start) < timeout_ms:
            self.node._session.poll(20)
            if self._replies:
                return self._decode(self._replies.pop(0))
        raise ServiceTimeout(
            "no reply from {} within {} ms".format(self.service_name, timeout_ms)
        )

    def call_nowait(self, request):
        """Fire a request without waiting; poll :meth:`take_reply` later."""
        self.node._session.write_data(
            self._requester, request.serialize(), C.STREAM_RELIABLE
        )

    def take_reply(self):
        if self._replies:
            return self._decode(self._replies.pop(0))
        return None
