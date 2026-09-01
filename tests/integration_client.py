"""The MicroPython half of the integration test.

Publishes a String and an Imu, subscribes to a Twist, and reports counts as
machine-readable lines the harness asserts on. Runs on the Unix port in CI and
unmodified on a real board.
"""

import sys
import time

sys.path.insert(0, ".")

from snakeros import Node
from snakeros.msg.std_msgs import String
from snakeros.msg.sensor_msgs import Imu
from snakeros.msg.geometry_msgs import Twist

AGENT = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

got = []


def on_cmd(msg):
    got.append((msg.linear.x, msg.angular.z))


node = Node("snakeros_itest", agent=AGENT, port=8888)
print("CONNECTED")

pub_s = node.create_publisher(String, "snakeros_chatter")
pub_i = node.create_publisher(Imu, "snakeros_imu")
sub = node.create_subscription(Twist, "snakeros_cmd", on_cmd)
print("ENTITIES_OK")

imu = Imu()
imu.header.frame_id = "imu_link"
imu.linear_acceleration.z = 9.81
imu.orientation_covariance = [0.5] * 9

n = 0
t0 = time.time()
while time.time() - t0 < SECONDS:
    pub_s.publish(String(data="snakeros integration %d" % n))
    imu.angular_velocity.z = 0.25
    imu.header.stamp.sec = int(time.time())
    pub_i.publish(imu)
    node.spin_once(15)
    n += 1
    time.sleep(0.05)

print("PUBLISHED %d" % n)
print("RECEIVED %d" % sub.received)
print("DECODE_ERRORS %d" % sub.errors)
if got:
    print("SAMPLE %.3f %.3f" % got[-1])
node.destroy()
print("DONE")
