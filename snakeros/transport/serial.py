"""Serial transport.

Serial matters because it works on **any** Pico, not just the wireless ones:
no WiFi, no access point, no network config, and no credentials in your
source. For a lot of small robots it is the better transport.

It is harder than UDP for two separate reasons.

Framing
-------
A byte stream has no message boundaries, so every message is HDLC-framed and
CRC-checked. See :mod:`snakeros.transport.framing`.

The USB CDC port *is* the REPL
------------------------------
On a Pico, the USB serial the Agent would talk to is the same one MicroPython
uses for its REPL. There are three ways out and none is free:

1. **Use a UART** on GPIO pins -- simplest and the default here. Costs two
   pins and a USB-serial adapter, but leaves the REPL alone, so you can still
   debug the board while it is talking to ROS 2. Recommended.
2. **A second USB CDC interface** via ``machine.USBDevice`` -- best user
   experience, most work, and needs a recent MicroPython. Sketched in
   :func:`usb_cdc_stream`.
3. **Take over the USB CDC** with ``os.dupterm(None)`` -- you lose the REPL,
   which makes development painful. Provided, but think first.

Run the Agent with, for example::

    micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
"""

from ..errors import NotConnectedError, TransportError
from .framing import FrameParser, encode_frame


class ByteStreamTransport:
    """XRCE over any byte stream, with HDLC framing.

    ``stream`` needs ``read(n)``, ``write(b)`` and ``close()``. ``read`` must
    be non-blocking and may return ``None`` or ``b""`` when nothing is
    waiting.
    """

    def __init__(self, stream=None, mtu=512, src=0, dst=0, read_chunk=256):
        self.stream = stream
        self.mtu = mtu
        self.src = src
        self.dst = dst
        self.read_chunk = read_chunk
        self._parser = FrameParser(max_payload=mtu * 4)
        self._pending = []

    # -- lifecycle ---------------------------------------------------------

    def open(self):
        if self.stream is None:
            raise NotConnectedError("no byte stream supplied")
        return self

    def close(self):
        if self.stream is not None:
            try:
                self.stream.close()
            except Exception:
                pass

    # -- the transport contract -------------------------------------------

    def send(self, data):
        if self.stream is None:
            raise NotConnectedError("transport not open")
        try:
            return self.stream.write(encode_frame(data, self.src, self.dst))
        except OSError as e:
            raise TransportError("serial write failed: {}".format(e))

    def recv(self):
        """Return one complete payload, or ``None`` if none is ready."""
        if self._pending:
            return self._pending.pop(0)
        if self.stream is None:
            raise NotConnectedError("transport not open")
        try:
            chunk = self.stream.read(self.read_chunk)
        except OSError:
            chunk = None
        if chunk:
            frames = self._parser.feed(chunk)
            if frames:
                self._pending.extend(frames)
                return self._pending.pop(0)
        return None

    # -- diagnostics -------------------------------------------------------

    @property
    def crc_errors(self):
        return self._parser.crc_errors

    @property
    def resyncs(self):
        return self._parser.resyncs

    def __enter__(self):
        return self.open()

    def __exit__(self, *a):
        self.close()


class SerialTransport(ByteStreamTransport):
    """XRCE over a UART or a serial device file.

    On a board, pass a configured ``machine.UART``::

        from machine import UART
        uart = UART(0, 115200, tx=Pin(0), rx=Pin(1), timeout=0)
        node = Node('pico', transport=SerialTransport(uart=uart))

    On a host (or the MicroPython Unix port), pass a device path::

        SerialTransport(device='/dev/ttyUSB0')
    """

    def __init__(self, uart=None, device=None, mtu=512, src=0, dst=0):
        stream = uart
        self._device = device
        if stream is None and device is not None:
            stream = _open_device(device)
        ByteStreamTransport.__init__(self, stream, mtu=mtu, src=src, dst=dst)

    def open(self):
        if self.stream is None and self._device is not None:
            self.stream = _open_device(self._device)
        return ByteStreamTransport.open(self)


def _open_device(path):
    import os

    fd = os.open(path, os.O_RDWR)
    try:
        os.set_blocking(fd, False)
    except AttributeError:
        pass
    return _FdStream(fd)


class _FdStream:
    """Minimal non-blocking file-descriptor stream."""

    def __init__(self, fd):
        import os

        self._os = os
        self.fd = fd

    def read(self, n):
        try:
            return self._os.read(self.fd, n)
        except OSError:
            return None

    def write(self, b):
        return self._os.write(self.fd, b)

    def close(self):
        self._os.close(self.fd)


def usb_cdc_stream(itf=1):
    """Second USB CDC interface, leaving the REPL on the first.

    Requires MicroPython with ``machine.USBDevice`` support and a board whose
    port enables runtime USB device configuration. This is option 2 from the
    module docstring: the nicest result, the most setup.

    Raises :class:`ImportError` where unsupported, so callers can fall back to
    a UART.
    """
    import machine  # noqa: F401  (import errors are the signal here)

    if not hasattr(machine, "USBDevice"):
        raise ImportError(
            "machine.USBDevice not available; use a UART "
            "(SerialTransport(uart=...)) instead"
        )
    raise NotImplementedError(
        "a second CDC interface must be built for the specific board; "
        "see docs/transports.md"
    )


def take_over_repl_cdc():
    """Detach the REPL from USB CDC and return it as a stream.

    Option 3: you get the USB port the Agent expects, and you lose the REPL
    until reset. Useful for a deployed robot, painful during development.
    """
    import os
    import sys

    try:
        os.dupterm(None)
    except (AttributeError, OSError) as e:
        raise TransportError("could not detach the REPL: {}".format(e))
    return sys.stdin.buffer if hasattr(sys.stdin, "buffer") else sys.stdin


class TCPByteStream:
    """A byte stream over TCP.

    Two uses: testing the framing against a real Agent through a
    pty-to-TCP bridge, and the genuinely common deployment where a serial
    line is tunnelled over the network.
    """

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self._sock = None

    def connect(self):
        import socket

        ai = socket.getaddrinfo(self.host, self.port, 0, socket.SOCK_STREAM)[0]
        self._sock = socket.socket(ai[0], socket.SOCK_STREAM)
        self._sock.connect(ai[-1])
        try:
            self._sock.settimeout(0.02)
        except (AttributeError, OSError):
            self._sock.setblocking(False)
        return self

    def read(self, n):
        if self._sock is None:
            return None
        try:
            return self._sock.recv(n)
        except OSError:
            return None

    def write(self, b):
        return self._sock.send(b)

    def close(self):
        if self._sock:
            self._sock.close()
            self._sock = None
