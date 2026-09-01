"""Minimal subscriber: prints incoming Twist messages.

    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
"""

import sys

from snakeros import Node
from snakeros.msg.geometry_msgs import Twist

AGENT = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"


def on_cmd(msg):
    print("cmd_vel: linear.x=%.2f angular.z=%.2f" % (msg.linear.x, msg.angular.z))


node = Node("snakeros_subscriber", agent=AGENT)
node.create_subscription(Twist, "cmd_vel", on_cmd)
print("listening on /cmd_vel")
node.spin()
