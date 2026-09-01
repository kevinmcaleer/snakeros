"""A timer publishing an IMU reading at 50 Hz."""

import sys
import time

from snakeros import Node
from snakeros.msg.sensor_msgs import Imu

AGENT = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"

node = Node("snakeros_timer", agent=AGENT)
pub = node.create_publisher(Imu, "imu/data")

msg = Imu()
msg.header.frame_id = "imu_link"
count = [0]


def tick():
    msg.header.stamp.sec = int(time.time())
    msg.linear_acceleration.z = 9.81
    pub.publish(msg)
    count[0] += 1
    if count[0] % 50 == 0:
        print("published", count[0])


node.create_timer(0.02, tick)   # 50 Hz
node.spin()
