"""SnakeROS -- a pure-Python ROS 2 client for MicroPython.

Speaks DDS-XRCE straight to a stock micro-ROS Agent: no C toolchain, no custom
firmware, and no reflashing to add a message type.
"""

__version__ = "0.1.0"

from .errors import (  # noqa: F401
    SnakeROSError,
    TransportError,
    SessionError,
    EntityError,
    CDRError,
    ServiceError,
    ParameterError,
)
from .node import Node, Publisher, Subscription, Timer  # noqa: F401

__all__ = [
    "Node",
    "Publisher",
    "Subscription",
    "Timer",
    "SnakeROSError",
    "TransportError",
    "SessionError",
    "EntityError",
    "CDRError",
    "ServiceError",
    "ParameterError",
    "__version__",
]
