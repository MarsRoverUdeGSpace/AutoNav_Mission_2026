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
- `usb_cam` (optional, for laptop webcam)

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

This package supports two modes of operation: **Webcam Mode** (standard USB cameras, CPU only) and **ZED Camera Mode** (ZED 2i, requires NVIDIA GPU).

### 1. Webcam Mode (No GPU required)
Use this mode for standard USB webcams or if you do not have the ZED SDK installed.

Run the standard launch file:
```bash
ros2 launch maya_aruco_detector aruco_detector.launch.xml
```
This launches:
- `usb_cam`: Captures video from `/dev/video0`.
- `aruco_detector_node`: Detects markers on `/camera/image_raw`.
- `screenshot_node`: Takes screenshots on demand.

### 2. ZED Camera Mode (Requires GPU + ZED SDK)
Use this mode if you have a ZED 2i connected and the ZED SDK installed.

**Prerequisites:**
- **ZED SDK** installed from [Stereolabs](https://www.stereolabs.com/developers/release/).
- `zed-ros2-wrapper` package installed in your workspace.

**Installation of ZED Wrapper (if missing):**
```bash
cd ~/projects/AutoNav_Mission_2026/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to zed_wrapper maya_aruco_detector
source install/setup.bash
```

**Launch:**
To launch the ZED wrapper and the detector together:
```bash
ros2 launch maya_aruco_detector aruco_detector_zed.launch.xml
```
*   **Default Topic:** `/zed/zed_node/rgb/image_rect_color`
*   **Default Camera Model:** `zed2i`

**Running Node Only (if ZED is already running):**
```bash
ros2 run maya_aruco_detector aruco_detector_node --ros-args -p image_topic:=/zed/zed_node/rgb/image_rect_color
```

## Topics

*   **Subscribed:**
    *   `/camera/image_raw` (Webcam Mode) OR `/zed/zed_node/rgb/image_rect_color` (ZED Mode)
*   **Published:**
    *   `/detected_aruco_id` (`std_msgs/msg/Int32`): ID of the first detected marker.

## Customization
You can remap the image topic via command line for any custom camera:
```bash
ros2 run maya_aruco_detector aruco_detector_node --ros-args -p image_topic:=/my_camera/image
```

## Troubleshooting
### "No executable found"
If you encounter `No executable found` when running the node:
1. Ensure you have sourced the setup file: `source install/setup.bash`
2. If you renamed the package, verify that `setup.cfg` points to the correct library directory (e.g., `lib/maya_aruco_detector`).
3. Try a clean build: `rm -rf build/ maya_aruco_detector install/maya_aruco_detector` followed by `colcon build`.
