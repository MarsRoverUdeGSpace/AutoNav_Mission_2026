from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maya_aruco_detector',
            executable='camera_publisher_node',
            name='camera_publisher',
            output='screen',
            parameters=[{'video_device': 0}]
        ),
        Node(
            package='maya_aruco_detector',
            executable='aruco_detector_node',
            name='aruco_detector',
            output='screen'
        ),
    ])
