"""SnakeROS -- a pure-Python ROS 2 client for MicroPython.

Speaks DDS-XRCE straight to a stock micro-ROS Agent: no C toolchain, no custom
firmware, and no reflashing to add a message type.

Names are resolved **lazily**. Importing a submodule -- ``snakeros.board`` for
WiFi setup, say -- would otherwise drag in the node, transport, XRCE and CDR
layers as a side effect, because a package's ``__init__`` runs first. That
costs around 41 KB on a 32-bit board, and on an ESP32 it is the difference
between lwIP having buffers and not: the GC heap grows by claiming ESP-IDF
heap and never gives it back.

So ``from snakeros.board import connect_wifi`` in ``boot.py`` now costs a
couple of KB rather than the whole library, and ``from snakeros import Node``
pulls the core in only when you actually ask for it.
"""

__version__ = "0.1.0"

# name -> submodule it lives in
_LAZY = {
    "Node": "node",
    "Publisher": "node",
    "Subscription": "node",
    "Timer": "node",
    "SnakeROSError": "errors",
    "TransportError": "errors",
    "SessionError": "errors",
    "EntityError": "errors",
    "CDRError": "errors",
    "ServiceError": "errors",
    "ParameterError": "errors",
}

__all__ = sorted(_LAZY) + ["__version__"]


def __getattr__(name):
    where = _LAZY.get(name)
    if where is None:
        raise AttributeError("module 'snakeros' has no attribute '%s'" % name)
    module = __import__("snakeros." + where, None, None, (where,))
    value = getattr(module, name)
    globals()[name] = value  # cache, so this costs once
    return value
