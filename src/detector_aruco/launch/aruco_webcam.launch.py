from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv',
                'camera_name': 'test_camera',
                'frame_id': 'camera_frame'
            }],
            remappings=[('/image_raw', '/camera/image_raw')]
        ),
        Node(
            package='maya_aruco_detector',
            executable='aruco_detector_node',
            name='aruco_detector',
            output='screen'
        ),
    ])
