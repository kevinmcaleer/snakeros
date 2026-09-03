"""UDP transport.

One datagram in, one datagram out -- message boundaries come free, which is
why this is the transport the project starts with. The serial transport has to
do HDLC framing itself and lands later.
"""

import socket

from ..errors import NotConnectedError, TransportError


class UDPTransport:
    """Datagram transport to a micro-ROS Agent listening on ``udp4``.

    The interface here -- ``open``/``send``/``recv``/``close`` plus ``mtu`` --
    is deliberately the whole contract the session layer depends on, so a
    different middleware or transport can be slotted in underneath without
    disturbing anything above.
    """

    # The socket timeout bounds how long a single recv() blocks when nothing
    # has arrived, so it puts a floor under spin_once() and therefore a
    # ceiling on the control-loop rate. At the old 0.1 s default a
    # spin_once(2) really took 100 ms and a "50 Hz" timer fired at 10 Hz.
    # Keep it small and let Session.poll() own the waiting.
    def __init__(self, host, port=8888, mtu=512, timeout=0.002):
        self.host = host
        self.port = port
        self.mtu = mtu
        self.timeout = timeout
        self._sock = None
        self._addr = None
        self._family = None
        self._rxbuf = None

    def open(self):
        # Resolve ONCE and cache. lwIP's getaddrinfo allocates an addrinfo
        # struct that MicroPython does not always free, so calling it on every
        # reconnect leaks until it fails with EAI_MEMORY -- surfaced as
        # "OSError: -203", which reads like a send problem and is really the
        # resolver running out of memory.
        #
        # A board that has retried a connection a few dozen times will hit
        # this, and a *soft* reboot does not clear it: lwIP keeps its state.
        # A hard reset does.
        if self._addr is None:
            ai = socket.getaddrinfo(self.host, self.port, 0, socket.SOCK_DGRAM)[0]
            self._addr = ai[-1]
            self._family = ai[0]
        self._sock = socket.socket(self._family, socket.SOCK_DGRAM)
        self._rxbuf = bytearray(self.mtu + 64)
        try:
            self._sock.settimeout(self.timeout)
        except (AttributeError, OSError):
            # Some MicroPython ports only offer non-blocking mode.
            self._sock.setblocking(False)
        return self

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def send(self, data):
        if self._sock is None:
            raise NotConnectedError("transport not open")
        try:
            return self._sock.sendto(data, self._addr)
        except OSError as e:
            # Name the destination. On a board, errno alone ("ENOMEM", "-203")
            # is routinely a *reachability* problem rather than a local one:
            # lwIP surfaces a queued-ARP failure as ENOMEM, and an ICMP
            # port-unreachable from a host with nothing listening as an errno
            # on the next send.
            raise TransportError(
                "send to {}:{} failed: {} (is the micro-ROS Agent running "
                "and reachable there?)".format(self.host, self.port, e))

    def recv(self):
        """Return one datagram, or ``None`` if nothing arrived in time.

        Reads into a buffer allocated once at open() rather than letting
        ``recv`` allocate per call. On a board with tens of KB free, a fresh
        allocation on every poll is a reliable way to fragment the heap.
        """
        if self._sock is None:
            raise NotConnectedError("transport not open")
        try:
            if self._rxbuf is not None and hasattr(self._sock, "recv_into"):
                n = self._sock.recv_into(self._rxbuf)
                if not n:
                    return None
                return bytes(memoryview(self._rxbuf)[:n])
            data = self._sock.recv(self.mtu + 64)
        except OSError:
            # Timeout or EAGAIN -- both mean "nothing yet", not "broken".
            return None
        return data or None

    def __enter__(self):
        return self.open()

    def __exit__(self, *a):
        self.close()
