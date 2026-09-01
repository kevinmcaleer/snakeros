"""Board bring-up helpers: WiFi, resilient connection, and safe defaults.

Everything else in SnakeROS is portable. This module is the part that knows
about real hardware -- and it exists because the MicroPython Unix port hides
things that bite on a board: WiFi that takes seconds to associate and drops
without warning, GC pauses landing mid-publish, and heap fragmentation that
degrades a robot slowly rather than failing cleanly.
"""

import gc
import time

from .errors import SnakeROSError

try:
    import network
except ImportError:  # the Unix port has no network module
    network = None


def connect_wifi(ssid, password, timeout_s=20, hostname=None, verbose=True):
    """Bring up station-mode WiFi and return the IP address.

    Raises rather than returning a half-open interface, because a node that
    thinks it has a network is harder to debug than one that failed to start.
    """
    if network is None:
        raise SnakeROSError("no network module: not running on a WiFi board")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if hostname is not None:
        try:
            network.hostname(hostname)
        except (AttributeError, OSError):
            pass
    if not wlan.isconnected():
        wlan.connect(ssid, password)
        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > timeout_s:
                wlan.active(False)
                raise SnakeROSError(
                    "WiFi did not associate with {!r} in {} s".format(ssid, timeout_s)
                )
            time.sleep(0.25)
    ip = wlan.ifconfig()[0]
    if verbose:
        print("WiFi up:", ip)
    return ip


def wifi_connected():
    if network is None:
        return True
    try:
        return network.WLAN(network.STA_IF).isconnected()
    except Exception:
        return False


class ResilientNode:
    """Wraps a :class:`~snakeros.node.Node` so a robot survives a dropped link.

    A microcontroller on a robot cannot simply exit when the network blips.
    This retries the session, rebuilds entities through a user-supplied
    ``setup`` callback, and -- crucially -- calls ``on_disconnect`` so the
    application can **stop the motors** rather than continue on the last
    received command.
    """

    def __init__(self, factory, setup=None, on_disconnect=None,
                 retry_s=2.0, max_backoff_s=30.0, verbose=True,
                 liveness_s=3.0, max_missed_pings=2):
        self.factory = factory
        self.setup = setup
        self.on_disconnect = on_disconnect
        self.retry_s = retry_s
        self.max_backoff_s = max_backoff_s
        self.verbose = verbose
        self.liveness_s = liveness_s
        self.max_missed_pings = max_missed_pings
        self.node = None
        self.reconnects = 0
        self.missed_pings = 0
        self._backoff = retry_s
        self._last_ping = 0.0

    def _log(self, *a):
        if self.verbose:
            print("[snakeros]", *a)

    def connect(self):
        while True:
            try:
                self.node = self.factory()
                if self.setup is not None:
                    self.setup(self.node)
                self._backoff = self.retry_s
                self._log("connected")
                return self.node
            except Exception as e:
                self._log("connect failed:", e, "- retrying in", self._backoff, "s")
                time.sleep(self._backoff)
                self._backoff = min(self._backoff * 2, self.max_backoff_s)

    def _check_liveness(self):
        """Actively probe the Agent.

        A publish over UDP to a dead Agent **succeeds** -- the datagram goes
        into the void and no exception is ever raised. So silence is not
        detectable by publishing; it has to be probed for. Without this, a
        robot whose Agent has died looks perfectly healthy, keeps "publishing"
        to nobody, and never reconnects.
        """
        now = time.time()
        if now - self._last_ping < self.liveness_s:
            return True
        self._last_ping = now
        if self.node.ping(timeout_ms=400):
            self.missed_pings = 0
            return True
        self.missed_pings += 1
        self._log("missed ping", self.missed_pings, "of", self.max_missed_pings)
        return self.missed_pings < self.max_missed_pings

    def spin_once(self, timeout_ms=10):
        try:
            self.node.spin_once(timeout_ms)
            if not self._check_liveness():
                raise SnakeROSError("Agent stopped answering pings")
            return True
        except Exception as e:
            self._log("lost the Agent:", e)
            if self.on_disconnect is not None:
                try:
                    self.on_disconnect()
                except Exception:
                    pass
            self.reconnects += 1
            self.missed_pings = 0
            self._last_ping = 0.0
            try:
                self.node.destroy()
            except Exception:
                pass
            gc.collect()
            self.connect()
            return False

    def spin(self, timeout_ms=10):
        try:
            while True:
                self.spin_once(timeout_ms)
        except KeyboardInterrupt:
            pass


def heap_report(label=""):
    """Print free/allocated heap. The first thing to reach for on a board."""
    gc.collect()
    free = gc.mem_free()
    alloc = gc.mem_alloc()
    print("[heap] {:<16} free {:>7,}  alloc {:>7,}  total {:>7,}".format(
        label, free, alloc, free + alloc))
    return free


def preallocate(node, samples=4):
    """Warm the allocator by exercising the publish path.

    Publishing allocates a CDR buffer each time. Doing that for the first time
    inside a 50 Hz control loop invites a GC pause exactly where it hurts;
    doing it during setup gets the collection out of the way and reduces
    fragmentation later.
    """
    for pub in node.publishers:
        msg = pub.msg_type()
        for _ in range(samples):
            try:
                msg.serialize()
            except Exception:
                break
    gc.collect()
