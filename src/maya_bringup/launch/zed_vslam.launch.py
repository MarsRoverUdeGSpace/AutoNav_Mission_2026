#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name')
    node_name = LaunchConfiguration('node_name')
    image_0_topic = LaunchConfiguration('image_0_topic')
    camera_info_0_topic = LaunchConfiguration('camera_info_0_topic')
    image_1_topic = LaunchConfiguration('image_1_topic')
    camera_info_1_topic = LaunchConfiguration('camera_info_1_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    base_frame = LaunchConfiguration('base_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    optical_frame_0 = LaunchConfiguration('optical_frame_0')
    optical_frame_1 = LaunchConfiguration('optical_frame_1')
    enable_slam_visualization = LaunchConfiguration('enable_slam_visualization')
    enable_landmarks_view = LaunchConfiguration('enable_landmarks_view')
    enable_observations_view = LaunchConfiguration('enable_observations_view')

    image_0_topic_value = image_0_topic.perform(context)
    camera_info_0_topic_value = camera_info_0_topic.perform(context)
    image_1_topic_value = image_1_topic.perform(context)
    camera_info_1_topic_value = camera_info_1_topic.perform(context)
    imu_topic_value = imu_topic.perform(context)
    base_frame_value = base_frame.perform(context)
    imu_frame_value = imu_frame.perform(context)
    optical_frame_0_value = optical_frame_0.perform(context)
    optical_frame_1_value = optical_frame_1.perform(context)
    enable_slam_visualization_value = enable_slam_visualization.perform(context).lower() == 'true'
    enable_landmarks_view_value = enable_landmarks_view.perform(context).lower() == 'true'
    enable_observations_view_value = enable_observations_view.perform(context).lower() == 'true'

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=[
            ('visual_slam/image_0', image_0_topic_value),
            ('visual_slam/camera_info_0', camera_info_0_topic_value),
            ('visual_slam/image_1', image_1_topic_value),
            ('visual_slam/camera_info_1', camera_info_1_topic_value),
            ('visual_slam/imu', imu_topic_value),
        ],
        parameters=[{
            'rectified_images': True,
            'tracking_mode': 1,
            'base_frame': base_frame_value,
            'imu_frame': imu_frame_value,
            'camera_optical_frames': [optical_frame_0_value, optical_frame_1_value],
            'enable_image_denoising': False,
            'enable_slam_visualization': enable_slam_visualization_value,
            'enable_landmarks_view': enable_landmarks_view_value,
            'enable_observations_view': enable_observations_view_value,
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

    return [container]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='zed2i'),
        DeclareLaunchArgument('node_name', default_value='zed_node'),
        DeclareLaunchArgument(
            'image_0_topic',
            default_value='/zed2i/zed_node/left/color/rect/image_rgb8',
        ),
        DeclareLaunchArgument(
            'camera_info_0_topic',
            default_value='/zed2i/zed_node/left/color/rect/camera_info',
        ),
        DeclareLaunchArgument(
            'image_1_topic',
            default_value='/zed2i/zed_node/right/color/rect/image_rgb8',
        ),
        DeclareLaunchArgument(
            'camera_info_1_topic',
            default_value='/zed2i/zed_node/right/color/rect/camera_info',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/sensors/bno055/imu/data',
        ),
        DeclareLaunchArgument('base_frame', default_value='zed2i_camera_link'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        DeclareLaunchArgument(
            'optical_frame_0',
            default_value='zed2i_left_camera_frame_optical',
        ),
        DeclareLaunchArgument(
            'optical_frame_1',
            default_value='zed2i_right_camera_frame_optical',
        ),
        DeclareLaunchArgument('enable_slam_visualization', default_value='true'),
        DeclareLaunchArgument('enable_landmarks_view', default_value='true'),
        DeclareLaunchArgument('enable_observations_view', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
