# Hardware Specifications - Maya Rover

This document details the physical and sensor specifications of the Maya Rover as defined in the simulation description.

## Physical Description

The rover assumes a 4-wheel skid-steer configuration.

### Kinematics
*   **Drive Type**: Skid Steer (modeled as Differential Drive in Gazebo)
*   **Wheel Separation (Track Width)**: 0.785 m
*   **Wheel Radius**: 0.10 m (10 cm)
*   **Wheelbase**: ~0.80 m (Approximated from base length)

### Inertial Properties
*   **Base Mass**: ~24.9 kg
*   **Wheel Mass**: ~1.93 kg (each)
*   **Total Mass**: ~32.6 kg

### Coordinate Frames (TF)
The URL tree roughly follows standard ROS conventions (REP-105):
*   `odom` (World fixed)
    *   `base_footprint` (Projected on ground)
        *   `base_link` (Chassis center, +0.424m Z offset)
            *   `lidar_link` (+0.3m Z from base)
            *   `imu_link`
            *   `front_left_wheel`
            *   `front_right_wheel`
            *   `back_left_wheel`
            *   `back_right_wheel`

## Sensors

### Light Detection and Ranging (LiDAR)
*   **Model**: Generic 2D GPU Lidar
*   **Frame**: `lidar_link`
*   **Topic**: `/scan`
*   **Update Rate**: 10 Hz
*   **Range**: 0.08m - 10.0m
*   **Field of View**: 360° (Horizontal)
*   **Resolution**: 1° (approx. 360 samples)

### Inertial Measurement Unit (IMU)
*   **Model**: Generic 6-DOF IMU (Accelerometer + Gyroscope)
*   **Frame**: `imu_link`
*   **Topic**: `/imu`
*   **Update Rate**: 100 Hz
*   **Gravity Vector**: Included in linear acceleration
