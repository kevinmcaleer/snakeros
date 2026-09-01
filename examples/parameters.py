"""Live-tunable parameters.

    ros2 param list /snakeros_parameters
    ros2 param set /snakeros_parameters kp 4.5
"""

import sys

from snakeros import Node

AGENT = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"

node = Node("snakeros_parameters", agent=AGENT)


def on_kp(value):
    print("kp is now", value)


node.declare_parameter("wheel_radius", 0.033, "wheel radius (m)",
                       minimum=0.001, maximum=1.0)
node.declare_parameter("kp", 1.0, "proportional gain",
                       minimum=0.0, maximum=100.0, callback=on_kp)
node.declare_parameter("robot_name", "snakebot", "display name")
node.declare_parameter("firmware", "0.1.0", "build id", read_only=True)

print("try:  ros2 param set /snakeros_parameters kp 4.5")
node.spin()
