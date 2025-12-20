# Simulation Environment

This project uses **Gazebo Sim** (formerly Ignition Gazebo) for simulating the Maya Rover. Interaction with ROS 2 is handled by `ros_gz_bridge`.

## Gazebo Configuration
*   **Gazebo Version**: Garden / Harmonic (recommended)
*   **World**: Empty world (default `empty.sdf`) with physics enabled.
*   **Physics Engine**: Ogre2 (Rendering), DART (Physics)

## ROS 2 <-> Gazebo Bridge
The `ros_gz_bridge` package handles message conversion between ROS 2 topics and Gazebo Transport topics. The configuration is defined in `maya_bringup/config/gazebo_bridge.yaml`.

| ROS Topic | Gazebo Topic | Message Type | Direction | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `/model/maya/cmd_vel` | `geometry_msgs/Twist` | ROS -> GZ | Velocity commands for the robot controller. |
| `/joint_states` | `/world/empty/model/maya/joint_state` | `sensor_msgs/JointState` | GZ -> ROS | Position and velocity of wheel joints. |
| `/tf` | `/model/maya/tf` | `tf2_msgs/TFMessage` | GZ -> ROS | Transform tree published by Gazebo plugins. |
| `/imu` | `/imu` | `sensor_msgs/Imu` | GZ -> ROS | IMU raw data (orientation, angular vel, linear accel). |
| `/scan` | `/scan` | `sensor_msgs/LaserScan` | GZ -> ROS | 2D LiDAR scan data. |
| `/odom` | `/model/maya/odometry` | `nav_msgs/Odometry` | GZ -> ROS | Ground truth odometry from the differential drive plugin. |
| `/clock` | `/clock` | `rosgraph_msgs/Clock` | GZ -> ROS | Simulation time for synchronization (`use_sim_time`). |

## Plugins
The simulation uses the following Gazebo System Plugins:
1.  **DiffDrive**: Controls the skid-steer mechanism via target velocity.
2.  **JointStatePublisher**: Publishes the state of all non-fixed joints.
3.  **Imu**: Generates IMU data.
4.  **Sensors**: Manages rendering-based sensors like LiDAR and Cameras.
