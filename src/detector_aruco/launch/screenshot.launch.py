from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('maya_aruco_detector')
    
    # Include the existing aruco_webcam launch file
    aruco_webcam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'aruco_webcam.launch.py')
        )
    )

    screenshot_node = Node(
        package='maya_aruco_detector',
        executable='screenshot_node',
        name='screenshot_recorder',
        output='screen'
    )

    return LaunchDescription([
        aruco_webcam_launch,
        screenshot_node
    ])
