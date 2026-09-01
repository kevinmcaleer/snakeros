"""A differential-drive robot whose entire low-level control runs on a Pico.

This is the end-to-end proof: a MicroPython board participating in a ROS 2
graph as a first-class node.

    subscribes  /cmd_vel        geometry_msgs/Twist
    publishes   /odom           nav_msgs/Odometry
                /imu/data       sensor_msgs/Imu
                /joint_states   sensor_msgs/JointState
    parameters  wheel_radius, wheel_separation, max_speed, kp, publish_rate
    service     /reset_odometry (std_srvs/Trigger)

Drive it with::

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 topic echo /odom

On a board::

    from examples.diff_drive.robot import main
    main(agent="192.168.1.10", ssid="my-wifi", password="secret")

On a laptop (simulated hardware, real Agent, real ROS 2 graph)::

    micropython examples/diff_drive/robot.py 127.0.0.1
"""

import math
import sys
import time

sys.path.insert(0, ".")

from snakeros import Node                                    # noqa: E402
from snakeros.board import ResilientNode, connect_wifi, heap_report, preallocate  # noqa: E402
from snakeros.msg.geometry_msgs import Twist                 # noqa: E402
from snakeros.msg.nav_msgs import Odometry                   # noqa: E402
from snakeros.msg.sensor_msgs import Imu, JointState         # noqa: E402
from snakeros.msg.std_srvs import Trigger                    # noqa: E402

from .hardware import IMU, Encoder, Motor, HAVE_HARDWARE     # noqa: E402

# -- pin map (adjust for your robot) --------------------------------------
LEFT_IN1, LEFT_IN2, LEFT_EN = 2, 3, 4
RIGHT_IN1, RIGHT_IN2, RIGHT_EN = 6, 7, 8
LEFT_ENC_A, LEFT_ENC_B = 10, 11
RIGHT_ENC_A, RIGHT_ENC_B = 12, 13
IMU_SCL, IMU_SDA = 17, 16

# If no command arrives within this long, stop. A robot that keeps driving on
# a stale command after the link drops is the failure mode that breaks things.
CMD_TIMEOUT_S = 0.5


class DiffDriveRobot:
    def __init__(self, node):
        self.node = node

        self.left = Motor(LEFT_IN1, LEFT_IN2, LEFT_EN)
        self.right = Motor(RIGHT_IN1, RIGHT_IN2, RIGHT_EN)
        self.left_enc = Encoder(LEFT_ENC_A, LEFT_ENC_B, motor=self.left)
        self.right_enc = Encoder(RIGHT_ENC_A, RIGHT_ENC_B, motor=self.right)
        self.imu = IMU(IMU_SCL, IMU_SDA)

        # pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._last_left = self.left_enc.revolutions()
        self._last_right = self.right_enc.revolutions()
        self._last_odom = time.time()
        self._last_cmd = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.stopped_by_timeout = False

        # -- parameters, tunable live with ros2 param ----------------------
        node.declare_parameter("wheel_radius", 0.033, "wheel radius (m)",
                               minimum=0.001, maximum=1.0)
        node.declare_parameter("wheel_separation", 0.16, "track width (m)",
                               minimum=0.01, maximum=2.0)
        node.declare_parameter("max_speed", 0.5, "max linear speed (m/s)",
                               minimum=0.01, maximum=5.0)
        node.declare_parameter("kp", 1.0, "proportional gain",
                               minimum=0.0, maximum=100.0)
        node.declare_parameter("publish_rate", 20.0, "odom/joint rate (Hz)",
                               minimum=1.0, maximum=100.0)
        node.declare_parameter("frame_id", "odom", "odometry frame")
        node.declare_parameter("child_frame_id", "base_link", "robot frame")

        # -- ROS interface -------------------------------------------------
        self.pub_odom = node.create_publisher(Odometry, "odom")
        self.pub_imu = node.create_publisher(Imu, "imu/data")
        self.pub_joints = node.create_publisher(JointState, "joint_states")
        node.create_subscription(Twist, "cmd_vel", self.on_cmd_vel)
        node.create_service(Trigger, "reset_odometry", self.on_reset)

        # reusable message objects: allocating these per publish would invite
        # a GC pause inside the control loop
        self._odom = Odometry()
        self._imu = Imu()
        self._joints = JointState()
        self._joints.name = ["left_wheel_joint", "right_wheel_joint"]
        self._joints.position = [0.0, 0.0]
        self._joints.velocity = [0.0, 0.0]

        rate = node.get_parameter("publish_rate", 20.0)
        node.create_timer(1.0 / rate, self.publish_state)
        node.create_timer(0.05, self.control_step)

    # -- ROS callbacks -----------------------------------------------------

    def on_cmd_vel(self, msg):
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        self._last_cmd = time.time()
        self.stopped_by_timeout = False

    def on_reset(self, _req):
        self.x = self.y = self.theta = 0.0
        res = Trigger.Response()
        res.success = True
        res.message = "odometry reset"
        print("[robot] odometry reset")
        return res

    # -- control -----------------------------------------------------------

    def control_step(self):
        # Fail safe: no command recently means stop, not "keep going".
        if time.time() - self._last_cmd > CMD_TIMEOUT_S:
            if not self.stopped_by_timeout:
                self.stop()
                self.stopped_by_timeout = True
            return

        sep = self.node.get_parameter("wheel_separation", 0.16)
        max_speed = self.node.get_parameter("max_speed", 0.5)
        kp = self.node.get_parameter("kp", 1.0)

        # differential drive kinematics
        left = self.target_linear - self.target_angular * sep / 2.0
        right = self.target_linear + self.target_angular * sep / 2.0

        self.left.set(kp * left / max_speed)
        self.right.set(kp * right / max_speed)

    def stop(self):
        self.left.stop()
        self.right.stop()

    # -- odometry ----------------------------------------------------------

    def update_odometry(self):
        radius = self.node.get_parameter("wheel_radius", 0.033)
        sep = self.node.get_parameter("wheel_separation", 0.16)

        now = time.time()
        dt = now - self._last_odom
        if dt <= 0:
            return 0.0, 0.0
        self._last_odom = now

        lrev = self.left_enc.revolutions()
        rrev = self.right_enc.revolutions()
        dl = (lrev - self._last_left) * 2.0 * math.pi * radius
        dr = (rrev - self._last_right) * 2.0 * math.pi * radius
        self._last_left = lrev
        self._last_right = rrev

        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / sep

        # integrate at the midpoint heading: noticeably better than using the
        # start heading when turning
        mid = self.theta + dtheta / 2.0
        self.x += dc * math.cos(mid)
        self.y += dc * math.sin(mid)
        self.theta += dtheta
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi

        return dc / dt, dtheta / dt

    # -- publishing --------------------------------------------------------

    def publish_state(self):
        vx, wz = self.update_odometry()
        secs = int(time.time())

        o = self._odom
        o.header.stamp.sec = secs
        o.header.frame_id = self.node.get_parameter("frame_id", "odom")
        o.child_frame_id = self.node.get_parameter("child_frame_id", "base_link")
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        # yaw -> quaternion (flat robot, so only z and w are non-zero)
        o.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        o.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        o.twist.twist.linear.x = vx
        o.twist.twist.angular.z = wz
        self.pub_odom.publish(o)

        ax, ay, az, gx, gy, gz = self.imu.read()
        m = self._imu
        m.header.stamp.sec = secs
        m.header.frame_id = "imu_link"
        m.linear_acceleration.x = ax
        m.linear_acceleration.y = ay
        m.linear_acceleration.z = az
        m.angular_velocity.x = gx
        m.angular_velocity.y = gy
        m.angular_velocity.z = gz
        self.pub_imu.publish(m)

        j = self._joints
        j.header.stamp.sec = secs
        j.position[0] = self._last_left * 2.0 * math.pi
        j.position[1] = self._last_right * 2.0 * math.pi
        j.velocity[0] = self.left.sim_speed
        j.velocity[1] = self.right.sim_speed
        self.pub_joints.publish(j)


def main(agent="127.0.0.1", port=8888, ssid=None, password=None, resilient=True):
    if ssid:
        connect_wifi(ssid, password, hostname="snakebot")

    print("[robot] hardware present:", HAVE_HARDWARE)
    heap_report("before node")

    robot = {}

    def factory():
        return Node("snakebot", agent=agent, port=port)

    def setup(node):
        robot["r"] = DiffDriveRobot(node)
        preallocate(node)
        heap_report("after setup")
        print("[robot] ready -- try: ros2 topic echo /odom")

    def on_disconnect():
        # The single most important line in this file.
        if "r" in robot:
            robot["r"].stop()
            print("[robot] Agent lost -- motors stopped")

    if resilient:
        rn = ResilientNode(factory, setup=setup, on_disconnect=on_disconnect)
        rn.connect()
        rn.spin(10)
    else:
        node = factory()
        setup(node)
        try:
            node.spin(10)
        finally:
            robot["r"].stop()
            node.destroy()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1")
