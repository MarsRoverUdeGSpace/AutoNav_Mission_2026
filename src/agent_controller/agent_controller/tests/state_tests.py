from agent_controller.schemas.mcu import MCUSchema


# 1) Nominal: driving forward at ~0.5 m/s, everything consistent
STATE_NOMINAL: MCUSchema = {
    "imu": (
        "sensor_msgs.msg.Imu(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1760012345, nanosec=412000000), frame_id='imu_link'), "
        "orientation=geometry_msgs.msg.Quaternion(x=0.012, y=-0.008, z=0.5646, w=0.8253), "
        "angular_velocity=geometry_msgs.msg.Vector3(x=0.003, y=-0.002, z=0.011), "
        "linear_acceleration=geometry_msgs.msg.Vector3(x=0.21, y=0.04, z=9.79))"
    ),
    "gnss_fix": (
        "sensor_msgs.msg.NavSatFix(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1760012345, nanosec=300000000), frame_id='gnss_link'), "
        "status=sensor_msgs.msg.NavSatStatus(status=2, service=1), "
        "latitude=38.406412, longitude=-110.791873, altitude=1371.8, "
        "position_covariance=[0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.09], position_covariance_type=2)"
    ),
    "altimeter": "std_msgs.msg.Float32(data=1372.4)",
    "yaw": "std_msgs.msg.Float32(data=1.19)",
    "odom": (
        "nav_msgs.msg.Odometry(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1760012345, nanosec=400000000), frame_id='odom'), child_frame_id='base_link', "
        "pose=PoseWithCovariance(pose=Pose(position=Point(x=14.82, y=36.51, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.5646, w=0.8253))), "
        "twist=TwistWithCovariance(twist=Twist(linear=Vector3(x=0.52, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.01))))"
    ),
    "rob_status": (
        "roboclaw_msgs.msg.Status(battery_voltage=24.6, m1_current=3.2, m2_current=3.4, "
        "temp1=41.5, temp2=39.8, error_flags=0x0000, error_text='OK')"
    ),
}


# 2) Faulty: odom says moving, IMU says still, M1 overcurrent, weak GNSS, yaw mismatch
STATE_FAULTY: MCUSchema = {
    "imu": (
        "sensor_msgs.msg.Imu(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1760012611, nanosec=118000000), frame_id='imu_link'), "
        "orientation=geometry_msgs.msg.Quaternion(x=0.031, y=0.044, z=0.5646, w=0.8253), "
        "angular_velocity=geometry_msgs.msg.Vector3(x=0.001, y=0.000, z=0.002), "
        "linear_acceleration=geometry_msgs.msg.Vector3(x=0.02, y=0.01, z=9.81))"
    ),
    "gnss_fix": (
        "sensor_msgs.msg.NavSatFix(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1760012611, nanosec=0), frame_id='gnss_link'), "
        "status=sensor_msgs.msg.NavSatStatus(status=0, service=1), "
        "latitude=38.406430, longitude=-110.791860, altitude=1371.2, "
        "position_covariance=[18.5, 0.0, 0.0, 0.0, 22.1, 0.0, 0.0, 0.0, 41.7], position_covariance_type=2)"
    ),
    "altimeter": "std_msgs.msg.Float32(data=1412.9)",
    "yaw": "std_msgs.msg.Float32(data=2.90)",
    "odom": (
        "nav_msgs.msg.Odometry(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1760012611, nanosec=100000000), frame_id='odom'), child_frame_id='base_link', "
        "pose=PoseWithCovariance(pose=Pose(position=Point(x=14.83, y=36.52, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.5646, w=0.8253))), "
        "twist=TwistWithCovariance(twist=Twist(linear=Vector3(x=0.60, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))))"
    ),
    "rob_status": (
        "roboclaw_msgs.msg.Status(battery_voltage=22.1, m1_current=14.8, m2_current=3.1, "
        "temp1=78.4, temp2=44.0, error_flags=0x0001, error_text='M1 OVERCURRENT WARNING')"
    ),
}


# 3) Dropout: several sensors silent or malformed
STATE_DROPOUT: MCUSchema = {
    "imu": "",
    "gnss_fix": "None",
    "altimeter": "std_msgs.msg.Float32(data=nan)",
    "yaw": "std_msgs.msg.Float32(data=1.19)",
    "odom": "nav_msgs.msg.Odometry(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1760012700, nanosec=0), frame_id='odom'), child_frame_id='base_link', pose=PoseWith",
    "rob_status": (
        "roboclaw_msgs.msg.Status(battery_voltage=24.4, m1_current=0.1, m2_current=0.1, "
        "temp1=35.0, temp2=34.8, error_flags=0x0000, error_text='OK')"
    ),
}