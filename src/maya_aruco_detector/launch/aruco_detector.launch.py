from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maya_aruco_detector',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            parameters=[{
                'use_compressed': True,
                'compressed_topic': '/zed/zed_node/rgb/color/rect/image/compressed',
                'image_topic': '/zed/zed_node/rgb/color/rect/image',

                'annotated_topic': '/aruco/annotated_image',
                'ids_topic': '/aruco/ids',

                'save_dir': '/home/ro/aruco_captures',
                'save_mode': 'crop',
                'crop_padding_px': 20,
                'overwrite_latest_per_id': True,
                'also_save_timestamped': True,
                'min_save_interval_sec': 0.5,

                'aruco_dictionary': 'DICT_4X4_50',
            }],
        ),
    ])
