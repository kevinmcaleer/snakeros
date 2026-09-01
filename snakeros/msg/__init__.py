"""ROS 2 message packs.

Packages are imported lazily -- ``from snakeros.msg import geometry_msgs``
costs heap only for what you actually use, which matters on a Pico W.
"""

from ._base import Msg

__all__ = ["Msg"]
