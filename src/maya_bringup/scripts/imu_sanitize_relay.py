#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def make_covariance(diagonal):
    covariance = [0.0] * 9
    covariance[0] = float(diagonal[0])
    covariance[4] = float(diagonal[1])
    covariance[8] = float(diagonal[2])
    return covariance


class ImuSanitizeRelay(Node):
    def __init__(self) -> None:
        super().__init__("imu_sanitize_relay")

        self.declare_parameter("input_topic", "/sensors/bno055/imu/data")
        self.declare_parameter("output_topic", "/imu")
        self.declare_parameter("frame_id_override", "")
        self.declare_parameter("replace_zero_stamp_only", True)
        self.declare_parameter("orientation_covariance_diagonal", [0.05, 0.05, 0.1])
        self.declare_parameter("angular_velocity_covariance_diagonal", [0.02, 0.02, 0.02])
        self.declare_parameter("linear_acceleration_covariance_diagonal", [0.1, 0.1, 0.1])

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.frame_id_override = self.get_parameter("frame_id_override").get_parameter_value().string_value
        self.replace_zero_stamp_only = bool(self.get_parameter("replace_zero_stamp_only").value)

        self.orientation_covariance = make_covariance(
            self.get_parameter("orientation_covariance_diagonal").value
        )
        self.angular_velocity_covariance = make_covariance(
            self.get_parameter("angular_velocity_covariance_diagonal").value
        )
        self.linear_acceleration_covariance = make_covariance(
            self.get_parameter("linear_acceleration_covariance_diagonal").value
        )

        self.publisher = self.create_publisher(Imu, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"imu_sanitize_relay forwarding {input_topic} -> {output_topic}, "
            f"frame_id_override={self.frame_id_override or '<keep>'}, "
            f"replace_zero_stamp_only={self.replace_zero_stamp_only}"
        )

    @staticmethod
    def covariance_unavailable(covariance) -> bool:
        return len(covariance) == 9 and all(float(value) == -1.0 for value in covariance)

    def imu_callback(self, msg: Imu) -> None:
        sanitized = deepcopy(msg)

        stamp_is_zero = sanitized.header.stamp.sec == 0 and sanitized.header.stamp.nanosec == 0
        if stamp_is_zero or not self.replace_zero_stamp_only:
            sanitized.header.stamp = self.get_clock().now().to_msg()

        if self.frame_id_override:
            sanitized.header.frame_id = self.frame_id_override

        if self.covariance_unavailable(sanitized.orientation_covariance):
            sanitized.orientation_covariance = self.orientation_covariance
        if self.covariance_unavailable(sanitized.angular_velocity_covariance):
            sanitized.angular_velocity_covariance = self.angular_velocity_covariance
        if self.covariance_unavailable(sanitized.linear_acceleration_covariance):
            sanitized.linear_acceleration_covariance = self.linear_acceleration_covariance

        self.publisher.publish(sanitized)


def main() -> None:
    rclpy.init()
    node = ImuSanitizeRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
