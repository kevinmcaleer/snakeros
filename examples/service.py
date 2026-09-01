"""A service the rest of the graph can call.

    ros2 service call /arm_motors std_srvs/srv/SetBool "{data: true}"
"""

import sys

from snakeros import Node
from snakeros.msg.std_srvs import SetBool

AGENT = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"

armed = [False]


def on_arm(req):
    armed[0] = req.data
    res = SetBool.Response()
    res.success = True
    res.message = "motors " + ("armed" if req.data else "disarmed")
    print(res.message)
    return res


node = Node("snakeros_service", agent=AGENT)
node.create_service(SetBool, "arm_motors", on_arm)
print("serving /arm_motors")
node.spin()
