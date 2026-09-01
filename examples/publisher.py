"""Minimal publisher: a String at 2 Hz.

    micropython examples/publisher.py 192.168.1.10
"""

import sys
import time

from snakeros import Node
from snakeros.msg.std_msgs import String

AGENT = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"

node = Node("snakeros_publisher", agent=AGENT)
pub = node.create_publisher(String, "chatter")

i = 0
while True:
    pub.publish(String(data="hello from MicroPython %d" % i))
    print("published", i)
    node.spin_once(10)
    i += 1
    time.sleep(0.5)
