#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class VslamOdomRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("vslam_odom_relay")

        self.declare_parameter("input_topic", "/visual_slam/tracking/odometry")
        self.declare_parameter("output_topic", "/visual_slam/tracking/odometry_base")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("camera_frame_id", "zed2i_camera_link")
        self.declare_parameter("tf_lookup_timeout_sec", 0.2)
        self.declare_parameter("pose_covariance_diagonal", [0.2, 0.2, 1e6, 1e6, 1e6, 0.1])
        self.declare_parameter("twist_covariance_diagonal", [1e3, 1e3, 1e6, 1e6, 1e6, 1e3])

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.tf_lookup_timeout = Duration(
            seconds=float(self.get_parameter("tf_lookup_timeout_sec").value)
        )

        self.pose_covariance = self.make_covariance(
            self.get_parameter("pose_covariance_diagonal").value
        )
        self.twist_covariance = self.make_covariance(
            self.get_parameter("twist_covariance_diagonal").value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self.odom_callback, 10
        )
        self.warned_tf_failure = False

        self.get_logger().info(
            f"Relaying VSLAM odom {self.input_topic} -> {self.output_topic} "
            f"using camera_frame={self.camera_frame_id} and base_frame={self.base_frame_id}"
        )

    @staticmethod
    def make_covariance(diagonal) -> list[float]:
        covariance = [0.0] * 36
        diagonal_indices = [0, 7, 14, 21, 28, 35]
        for index, value in zip(diagonal_indices, diagonal):
            covariance[index] = float(value)
        return covariance

    def odom_callback(self, msg: Odometry) -> None:
        if msg.child_frame_id != self.camera_frame_id:
            self.get_logger().warn(
                f"Ignoring VSLAM odom child_frame_id={msg.child_frame_id}; "
                f"expected {self.camera_frame_id}",
                throttle_duration_sec=5.0,
            )
            return

        try:
            base_to_camera = self.tf_buffer.lookup_transform(
                self.base_frame_id,
                self.camera_frame_id,
                rclpy.time.Time(),
                timeout=self.tf_lookup_timeout,
            )
        except TransformException as exc:
            if not self.warned_tf_failure:
                self.get_logger().warn(
                    f"Waiting for TF {self.base_frame_id} -> {self.camera_frame_id}: {exc}"
                )
                self.warned_tf_failure = True
            return

        self.warned_tf_failure = False

        camera_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        base_to_camera_yaw = yaw_from_quaternion(base_to_camera.transform.rotation)
        base_to_camera_x = base_to_camera.transform.translation.x
        base_to_camera_y = base_to_camera.transform.translation.y

        cos_yaw = math.cos(camera_yaw)
        sin_yaw = math.sin(camera_yaw)

        rotated_offset_x = cos_yaw * base_to_camera_x - sin_yaw * base_to_camera_y
        rotated_offset_y = sin_yaw * base_to_camera_x + cos_yaw * base_to_camera_y

        base_yaw = camera_yaw - base_to_camera_yaw
        base_yaw = math.atan2(math.sin(base_yaw), math.cos(base_yaw))

        relay_msg = Odometry()
        relay_msg.header = msg.header
        relay_msg.header.frame_id = self.odom_frame_id
        relay_msg.child_frame_id = self.base_frame_id
        relay_msg.pose.pose.position.x = msg.pose.pose.position.x - rotated_offset_x
        relay_msg.pose.pose.position.y = msg.pose.pose.position.y - rotated_offset_y
        relay_msg.pose.pose.position.z = 0.0
        relay_msg.pose.pose.orientation = quaternion_from_yaw(base_yaw)
        relay_msg.pose.covariance = self.pose_covariance
        relay_msg.twist.twist = msg.twist.twist
        relay_msg.twist.covariance = self.twist_covariance
        self.publisher.publish(relay_msg)


def main() -> None:
    rclpy.init()
    node = VslamOdomRelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
