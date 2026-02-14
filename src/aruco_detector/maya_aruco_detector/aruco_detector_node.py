#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class ArucoDetectorNode(Node):
    """
    A ROS 2 node that subscribes to an image topic, detects ArUco markers,
    and publishes the ID of the first detected marker.
    """

    def __init__(self):
        super().__init__('aruco_detector_node')

        # Parameters
        self.declare_parameter('aruco_dictionary_id', 'DICT_4X4_250')
        self.declare_parameter('image_topic', '/camera/image_raw')
        
        dictionary_id_name = self.get_parameter('aruco_dictionary_id').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # Resolve dictionary ID from string
        try:
            self.aruco_dictionary_id = getattr(cv2.aruco, dictionary_id_name)
        except AttributeError:
            self.get_logger().error(f"Invalid ArUco dictionary name: {dictionary_id_name}. Defaulting to DICT_4X4_250")
            self.aruco_dictionary_id = cv2.aruco.DICT_4X4_250

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(self.aruco_dictionary_id)
        
        # Detector Parameters setup (Handling version differences)
        try:
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_parameters = cv2.aruco.DetectorParameters()

        # Publishers and Subscribers
        self.publisher_ = self.create_publisher(Int32, '/detected_aruco_id', 10)
        self.subscription_ = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.cv_bridge = CvBridge()
        self.cv_image = None

        # Timer for processing images at ~1Hz
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(f"ArUco Detector Node has been started. Subscribed to {image_topic}")

    def image_callback(self, msg):
        """
        Callback function for the image subscriber.
        Converts ROS Image message to OpenCV image and stores it.
        """
        try:
            # Convert ROS Image message to OpenCV image
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unknown conversion error: {e}")

    def timer_callback(self):
        """
        Timer callback function to process the latest image and detect markers.
        """
        if self.cv_image is None:
            return

        # Detect markers
        # Ensure image is contiguous and grayscale
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dictionary, parameters=self.aruco_parameters
        )

        if ids is not None and len(ids) > 0:
            # We found at least one marker
            first_id = int(ids[0][0])
            self.get_logger().info(f"Detected ArUco ID: {first_id}")
            
            # Publish the first detected ID
            msg_out = Int32()
            msg_out.data = first_id
            self.publisher_.publish(msg_out)
        else:
             # No markers detected
             pass

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
