# Maya ArUco Detector

## Overview
The `maya_aruco_detector` package is a ROS 2 node designed for the Maya Rover (AutoNav Mission 2026). It detects ArUco markers from a live camera feed and publishes the ID of the detected markers.

## Features
- Subscribe to camera image topics.
- Detect ArUco markers using OpenCV (`cv2.aruco`).
- Publish the ID of the first detected marker.
- Configurable parameters for dictionary and topics.

## Requirements
- ROS 2 (Humble/Iron/Jazzy)
- Python 3
- OpenCV (`python3-opencv`)
- `cv_bridge`
- `zed-ros2-wrapper` (for ZED camera usage)

## Installation
1. Clone the repository into your workspace `src` directory:
   ```bash
   cd ~/<yourworkspace>/
   # (Clone command here if applicable, or copy files)
   ```

2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   sudo apt install ros-jazzy-usb-cam # Or your ROS distro
   ```

3. Build the package:
   ```bash
   cd ~/<yourworkspace>/
   colcon build --packages-select maya_aruco_detector
   ```

4. Source the setup script:
   ```bash
   source install/setup.bash
   ```

## Usage

This package is designed to work exclusively with the **ZED 2i Camera** and requires the **ZED SDK** and NVIDIA GPU.

### Prerequisites
- **ZED SDK** installed from [Stereolabs](https://www.stereolabs.com/developers/release/).
- `zed-ros2-wrapper` package installed in your workspace.

### Installation of ZED Wrapper (if missing)
```bash
cd ~/projects/AutoNav_Mission_2026/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to zed_wrapper maya_aruco_detector
source install/setup.bash
```

### Launching
To launch the ZED wrapper and the detector together (Standard Mode):
```bash
ros2 launch maya_aruco_detector aruco_detector.launch.xml
```
*   **Default Topic:** `/zed/zed_node/rgb/image_rect_color`
*   **Default Camera Model:** `zed2i`

### Running Node Only
If your ZED camera is already running (e.g., launched separately):
```bash
ros2 run maya_aruco_detector aruco_detector_node
```

## Topics

*   **Subscribed:**
    *   `/zed/zed_node/rgb/image_rect_color` (`sensor_msgs/msg/Image`)
*   **Published:**
    *   `/detected_aruco_id` (`std_msgs/msg/Int32`): ID of the first detected marker.


## Troubleshooting
### "No executable found"
If you encounter `No executable found` when running the node:
1. Ensure you have sourced the setup file: `source install/setup.bash`
2. If you renamed the package, verify that `setup.cfg` points to the correct library directory (e.g., `lib/maya_aruco_detector`).
3. Try a clean build: `rm -rf build/ maya_aruco_detector install/maya_aruco_detector` followed by `colcon build`.
