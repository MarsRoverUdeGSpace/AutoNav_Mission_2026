#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=[
            ('visual_slam/image_0', '/zed2i/zed_node/left/color/rect/image_rgb8'),
            ('visual_slam/camera_info_0', '/zed2i/zed_node/left/color/rect/camera_info'),
            ('visual_slam/image_1', '/zed2i/zed_node/right/color/rect/image_rgb8'),
            ('visual_slam/camera_info_1', '/zed2i/zed_node/right/color/rect/camera_info'),
            ('visual_slam/imu', '/zed2i/zed_node/imu/data'),
        ],
        parameters=[{
            'rectified_images': True,
            'tracking_mode': 1,
            'base_frame': 'zed2i_camera_link',
            'imu_frame': 'zed2i_imu_link',
            'camera_optical_frames': [
                'zed2i_left_camera_frame_optical',
                'zed2i_right_camera_frame_optical'
            ],
            'enable_image_denoising': False,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
        }],
    )

    container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return LaunchDescription([container])
