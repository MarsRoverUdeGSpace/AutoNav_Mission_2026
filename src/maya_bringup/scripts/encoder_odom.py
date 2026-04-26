#!/usr/bin/env python3

import math
from typing import Optional, Type

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, Int64
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class EncoderOdomNode(Node):
    def __init__(self) -> None:
        super().__init__("encoder_odom")

        self.declare_parameter("left_ticks_topic", "/sensors/roboclaw/encoders/left_m1/ticks")
        self.declare_parameter("right_ticks_topic", "/sensors/roboclaw/encoders/right_m1/ticks")
        self.declare_parameter("encoder_msg_type", "std_msgs/msg/Int32")
        self.declare_parameter("ticks_per_rev", 0)
        self.declare_parameter("wheel_radius", 0.1636)
        self.declare_parameter("wheel_separation", 1.0)
        self.declare_parameter("left_ticks_sign", 1.0)
        self.declare_parameter("right_ticks_sign", 1.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("pose_covariance_diagonal", [0.02, 0.02, 1e6, 1e6, 1e6, 0.05])
        self.declare_parameter("twist_covariance_diagonal", [0.05, 0.05, 1e6, 1e6, 1e6, 0.08])

        left_topic = self.get_parameter("left_ticks_topic").get_parameter_value().string_value
        right_topic = self.get_parameter("right_ticks_topic").get_parameter_value().string_value
        self.ticks_per_rev = float(self.get_parameter("ticks_per_rev").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.left_ticks_sign = float(self.get_parameter("left_ticks_sign").value)
        self.right_ticks_sign = float(self.get_parameter("right_ticks_sign").value)
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self.base_frame_id = self.get_parameter("base_frame_id").get_parameter_value().string_value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        if self.ticks_per_rev <= 0.0:
            raise ValueError("ticks_per_rev must be > 0")
        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be > 0")
        if self.wheel_separation <= 0.0:
            raise ValueError("wheel_separation must be > 0")

        encoder_msg_type = self.get_parameter("encoder_msg_type").get_parameter_value().string_value
        msg_type = self.resolve_msg_type(encoder_msg_type)

        self.left_ticks: Optional[int] = None
        self.right_ticks: Optional[int] = None
        self.last_left_ticks: Optional[int] = None
        self.last_right_ticks: Optional[int] = None
        self.last_stamp = None
        self.left_ticks_fresh = False
        self.right_ticks_fresh = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.pose_cov = self.make_covariance(self.get_parameter("pose_covariance_diagonal").value)
        self.twist_cov = self.make_covariance(self.get_parameter("twist_covariance_diagonal").value)

        # Hardware encoder publishers are commonly best-effort. Match that QoS so
        # the node can receive raw ticks from the PCB without forcing a bridge.
        encoder_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(msg_type, left_topic, self.left_ticks_cb, encoder_qos)
        self.create_subscription(msg_type, right_topic, self.right_ticks_cb, encoder_qos)

        self.get_logger().info(
            f"encoder_odom listening on left={left_topic}, right={right_topic}, "
            f"ticks_per_rev={self.ticks_per_rev}, wheel_radius={self.wheel_radius}, "
            f"wheel_separation={self.wheel_separation}, "
            f"left_ticks_sign={self.left_ticks_sign}, right_ticks_sign={self.right_ticks_sign}, "
            f"publish_tf={self.publish_tf}"
        )

    @staticmethod
    def resolve_msg_type(name: str) -> Type[Int32] | Type[Int64]:
        if name == "std_msgs/msg/Int32":
            return Int32
        if name == "std_msgs/msg/Int64":
            return Int64
        raise ValueError(f"Unsupported encoder_msg_type: {name}")

    @staticmethod
    def make_covariance(diagonal):
        cov = [0.0] * 36
        idx = [0, 7, 14, 21, 28, 35]
        for i, v in zip(idx, diagonal):
            cov[i] = float(v)
        return cov

    def left_ticks_cb(self, msg) -> None:
        self.left_ticks = int(self.left_ticks_sign * int(msg.data))
        self.left_ticks_fresh = True
        self.maybe_update()

    def right_ticks_cb(self, msg) -> None:
        self.right_ticks = int(self.right_ticks_sign * int(msg.data))
        self.right_ticks_fresh = True
        self.maybe_update()

    def maybe_update(self) -> None:
        if self.left_ticks is None or self.right_ticks is None:
            return
        if not (self.left_ticks_fresh and self.right_ticks_fresh):
            return

        now = self.get_clock().now()
        if self.last_stamp is None:
            self.last_left_ticks = self.left_ticks
            self.last_right_ticks = self.right_ticks
            self.last_stamp = now
            self.left_ticks_fresh = False
            self.right_ticks_fresh = False
            return

        dt = (now - self.last_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        dl_ticks = self.left_ticks - self.last_left_ticks
        dr_ticks = self.right_ticks - self.last_right_ticks

        if dl_ticks == 0 and dr_ticks == 0:
            self.publish_odom(now, 0.0, 0.0)
            self.last_stamp = now
            self.left_ticks_fresh = False
            self.right_ticks_fresh = False
            return

        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        dl = dl_ticks * meters_per_tick
        dr = dr_ticks * meters_per_tick

        dc = 0.5 * (dl + dr)
        dtheta = (dr - dl) / self.wheel_separation

        self.x += dc * math.cos(self.yaw + 0.5 * dtheta)
        self.y += dc * math.sin(self.yaw + 0.5 * dtheta)
        self.yaw += dtheta
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        vx = dc / dt
        wz = dtheta / dt

        self.publish_odom(now, vx, wz)

        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        self.last_stamp = now
        self.left_ticks_fresh = False
        self.right_ticks_fresh = False

    def publish_odom(self, stamp, vx: float, wz: float) -> None:
        quat = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.pose.covariance = self.pose_cov
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = self.twist_cov
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = quat
            self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
