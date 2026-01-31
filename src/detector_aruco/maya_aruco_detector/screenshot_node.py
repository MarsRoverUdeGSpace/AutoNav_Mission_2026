#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import os
import time
from datetime import datetime

class ScreenshotNode(Node):
    def __init__(self):
        super().__init__('screenshot_node')

        self.cv_bridge = CvBridge()
        self.last_image = None
        self.last_save_time = 0
        self.cooldown_duration = 10.0  # seconds

        # Create output directory
        self.output_dir = os.path.expanduser('~/aruco_detected_screenshot')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            self.get_logger().info(f"Created directory: {self.output_dir}")

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.id_sub = self.create_subscription(
            Int32,
            '/detected_aruco_id',
            self.id_callback,
            10
        )

        self.get_logger().info("Screenshot Node started. Saving to ~/aruco_detected_screenshot/")

    def image_callback(self, msg):
        try:
            self.last_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def id_callback(self, msg):
        current_time = time.time()
        
        if (current_time - self.last_save_time) < self.cooldown_duration:
            # Cooldown active
            return

        if self.last_image is None:
            self.get_logger().warn("ArUco detected but no image available yet.")
            return

        # Save screenshot
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"aruco_{msg.data}_{timestamp}.png"
            filepath = os.path.join(self.output_dir, filename)
            
            cv2.imwrite(filepath, self.last_image)
            self.get_logger().info(f"Screenshot saved: {filepath}")
            
            self.last_save_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Failed to save screenshot: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ScreenshotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
