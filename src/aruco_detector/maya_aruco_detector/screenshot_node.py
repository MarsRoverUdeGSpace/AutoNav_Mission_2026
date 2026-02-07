#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class ScreenshotNode(Node):
    """
    A ROS 2 node that saves a screenshot of the camera feed when an ArUco marker is detected.
    Includes a cooldown mechanism to prevent spamming screenshots.
    Publishes status to /aruco_captured.
    """
    def __init__(self):
        super().__init__('screenshot_node')

        self.cv_bridge = CvBridge()
        self.last_image = None
        self.last_save_time = self.get_clock().now()
        # Initialize with a time in the past so first detection works immediately
        self.last_save_time -= rclpy.duration.Duration(seconds=20) 
        
        self.cooldown_duration = 10.0  # seconds
        self.status_duration = 3.0 # seconds to keep status True
        self.capture_active_until = self.get_clock().now() - rclpy.duration.Duration(seconds=10)

        # Create output directory
        self.output_dir = os.path.expanduser('~/aruco_detected_screenshot')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            self.get_logger().info(f"Created directory: {self.output_dir}")

        # Publishers
        self.status_pub = self.create_publisher(Bool, '/aruco_captured', 10)

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

        # Timer for status publishing (10Hz)
        self.create_timer(0.1, self.status_callback)

        self.get_logger().info("Screenshot Node started. Saving to ~/aruco_detected_screenshot/")

    def image_callback(self, msg):
        """
        Callback for image topic. Updates the latest available image.
        """
        try:
            self.last_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def status_callback(self):
        """
        Timer callback to publish the capture status.
        """
        msg = Bool()
        if self.get_clock().now() < self.capture_active_until:
            msg.data = True
        else:
            msg.data = False
        self.status_pub.publish(msg)

    def id_callback(self, msg):
        """
        Callback for detected ArUco ID. Triggers screenshot save if cooldown has passed.
        """
        current_time = self.get_clock().now()
        
        time_diff = (current_time - self.last_save_time).nanoseconds / 1e9
        
        if time_diff < self.cooldown_duration:
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
            # Set status to True for 3 seconds
            self.capture_active_until = current_time + rclpy.duration.Duration(seconds=self.status_duration)
            
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
