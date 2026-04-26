#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ZedBgraToRgb(Node):
    def __init__(self) -> None:
        super().__init__('zed_bgra_to_rgb')

        self._bridge = CvBridge()
        self.declare_parameter('left_input_topic', '/zed2i/zed_node/left/color/rect/image')
        self.declare_parameter('right_input_topic', '/zed2i/zed_node/right/color/rect/image')
        self.declare_parameter('left_output_topic', '/zed2i/zed_node/left/color/rect/image_rgb8')
        self.declare_parameter('right_output_topic', '/zed2i/zed_node/right/color/rect/image_rgb8')

        left_input_topic = self.get_parameter('left_input_topic').value
        right_input_topic = self.get_parameter('right_input_topic').value
        left_output_topic = self.get_parameter('left_output_topic').value
        right_output_topic = self.get_parameter('right_output_topic').value

        self._sub_left = self.create_subscription(
            Image,
            left_input_topic,
            self._left_cb,
            10
        )
        self._sub_right = self.create_subscription(
            Image,
            right_input_topic,
            self._right_cb,
            10
        )

        self._pub_left = self.create_publisher(
            Image,
            left_output_topic,
            10
        )
        self._pub_right = self.create_publisher(
            Image,
            right_output_topic,
            10
        )

        self.get_logger().info(
            'Republishing ZED stereo images as rgb8 '
            f'left={left_input_topic} -> {left_output_topic}, '
            f'right={right_input_topic} -> {right_output_topic}'
        )

    def _convert(self, msg: Image):
        if msg.encoding == 'bgra8':
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
            cv_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2RGB)
            out = self._bridge.cv2_to_imgmsg(cv_rgb, encoding='rgb8')
            out.header = msg.header
            return out

        if msg.encoding == 'bgr8':
            cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            out = self._bridge.cv2_to_imgmsg(cv_rgb, encoding='rgb8')
            out.header = msg.header
            return out

        if msg.encoding == 'rgb8':
            return msg

        self.get_logger().error(f'Unsupported image encoding: {msg.encoding}')
        return None

    def _left_cb(self, msg: Image) -> None:
        out = self._convert(msg)
        if out is not None:
            self._pub_left.publish(out)

    def _right_cb(self, msg: Image) -> None:
        out = self._convert(msg)
        if out is not None:
            self._pub_right.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedBgraToRgb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
