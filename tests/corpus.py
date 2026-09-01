"""A corpus of messages with awkward values, shared by both sides of the
rclpy differential test.

Values are chosen to catch alignment bugs: odd-length strings before floats,
sequences before nested messages, negative and fractional numbers, and the
9-element covariance arrays that make Imu and Odometry such good canaries.
"""

CORPUS = [
    ("std_msgs/String", {"data": "hello from a Pico"}),
    ("std_msgs/String", {"data": ""}),
    ("std_msgs/String", {"data": "x" * 17}),
    ("std_msgs/Bool", {"data": True}),
    ("std_msgs/Int8", {"data": -8}),
    ("std_msgs/UInt8", {"data": 200}),
    ("std_msgs/Int16", {"data": -300}),
    ("std_msgs/UInt16", {"data": 65000}),
    ("std_msgs/Int32", {"data": -70000}),
    ("std_msgs/UInt32", {"data": 4000000000}),
    ("std_msgs/Int64", {"data": -5000000000}),
    ("std_msgs/UInt64", {"data": 18000000000000000000}),
    ("std_msgs/Float32", {"data": 1.5}),
    ("std_msgs/Float64", {"data": -2.718281828459045}),
    ("std_msgs/Header", {"stamp": {"sec": 12345, "nanosec": 678901234},
                          "frame_id": "base_link"}),
    ("geometry_msgs/Vector3", {"x": 1.5, "y": -2.25, "z": 0.125}),
    ("geometry_msgs/Quaternion", {"x": 0.0, "y": 0.0, "z": 0.7071, "w": 0.7071}),
    ("geometry_msgs/Twist", {"linear": {"x": 0.5, "y": 0.0, "z": 0.0},
                             "angular": {"x": 0.0, "y": 0.0, "z": -1.25}}),
    ("geometry_msgs/Pose", {"position": {"x": 1.0, "y": 2.0, "z": 3.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}),
    ("geometry_msgs/PoseStamped", {
        "header": {"stamp": {"sec": 7, "nanosec": 8}, "frame_id": "odd"},
        "pose": {"position": {"x": -1.5, "y": 0.0, "z": 9.75},
                 "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.4}}}),
    ("geometry_msgs/TransformStamped", {
        "header": {"stamp": {"sec": 1, "nanosec": 2}, "frame_id": "map"},
        "child_frame_id": "base_link",
        "transform": {"translation": {"x": 1.0, "y": 2.0, "z": 3.0},
                      "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}}),
    ("sensor_msgs/Imu", {
        "header": {"stamp": {"sec": 100, "nanosec": 200}, "frame_id": "imu_link"},
        "orientation": {"x": 0.1, "y": 0.2, "z": 0.3, "w": 0.9},
        "orientation_covariance": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9],
        "angular_velocity": {"x": 0.01, "y": 0.02, "z": 0.03},
        "angular_velocity_covariance": [1.0] * 9,
        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.81},
        "linear_acceleration_covariance": [0.0] * 9}),
    ("sensor_msgs/JointState", {
        "header": {"stamp": {"sec": 5, "nanosec": 6}, "frame_id": ""},
        "name": ["left_wheel", "right_wheel"],
        "position": [1.25, -1.25],
        "velocity": [0.5, 0.5],
        "effort": []}),
    ("sensor_msgs/Range", {
        "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "sonar"},
        "radiation_type": 0, "field_of_view": 0.26, "min_range": 0.02,
        "max_range": 4.0, "range": 1.234}),
    ("sensor_msgs/LaserScan", {
        "header": {"stamp": {"sec": 3, "nanosec": 4}, "frame_id": "laser"},
        "angle_min": -1.57, "angle_max": 1.57, "angle_increment": 0.01,
        "time_increment": 0.0001, "scan_time": 0.1,
        "range_min": 0.1, "range_max": 10.0,
        "ranges": [1.0, 2.0, 3.0, 4.0, 5.0],
        "intensities": [10.0, 20.0]}),
    ("sensor_msgs/BatteryState", {
        "header": {"stamp": {"sec": 1, "nanosec": 1}, "frame_id": "bat"},
        "voltage": 12.6, "temperature": 25.0, "current": -1.5,
        "charge": 2.0, "capacity": 2.2, "design_capacity": 2.2,
        "percentage": 0.85, "power_supply_status": 2,
        "power_supply_health": 1, "power_supply_technology": 2,
        "present": True, "cell_voltage": [4.2, 4.2, 4.2],
        "cell_temperature": [25.0, 25.0, 25.0],
        "location": "rear", "serial_number": "SN-0001"}),
    ("nav_msgs/Odometry", {
        "header": {"stamp": {"sec": 11, "nanosec": 22}, "frame_id": "odom"},
        "child_frame_id": "base_link",
        "pose": {"pose": {"position": {"x": 1.1, "y": 2.2, "z": 0.0},
                          "orientation": {"x": 0.0, "y": 0.0, "z": 0.38, "w": 0.92}},
                 "covariance": [0.01 * i for i in range(36)]},
        "twist": {"twist": {"linear": {"x": 0.3, "y": 0.0, "z": 0.0},
                            "angular": {"x": 0.0, "y": 0.0, "z": 0.1}},
                  "covariance": [0.02 * i for i in range(36)]}}),
    ("geometry_msgs/Polygon", {
        "points": [{"x": 0.0, "y": 0.0, "z": 0.0},
                   {"x": 1.0, "y": 0.0, "z": 0.0},
                   {"x": 1.0, "y": 1.0, "z": 0.0}]}),
    ("std_msgs/ColorRGBA", {"r": 0.1, "g": 0.2, "b": 0.3, "a": 1.0}),
]
