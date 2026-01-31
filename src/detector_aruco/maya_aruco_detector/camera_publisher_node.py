#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        self.declare_parameter('video_device', 0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        
        video_device = self.get_parameter('video_device').get_parameter_value().integer_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Image, image_topic, 10)
        self.timer = self.create_timer(0.033, self.timer_callback) # ~30 FPS
        self.cap = cv2.VideoCapture(video_device)
        self.cv_bridge = CvBridge()
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video device {video_device}")
        else:
            self.get_logger().info(f"Camera Publisher started on device {video_device}, publishing to {image_topic}")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.publisher_.publish(msg)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
