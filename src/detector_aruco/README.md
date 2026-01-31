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

## Installation
1. Clone the repository into your workspace `src` directory:
   ```bash
   cd ~/<yourworkspace>/
   # (Clone command here if applicable, or copy files)
   ```

2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
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
Run the detector node:
```bash
ros2 run maya_aruco_detector aruco_detector_node
```

### Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `image_topic` | string | `/camera/image_raw` | The ROS topic to subscribe to for image data. |
| `aruco_dictionary_id` | string | `DICT_4X4_50` | The ArUco dictionary to use for detection. |

### Topics
* **Subscribed:**
  * `/camera/image_raw` (`sensor_msgs/msg/Image`)

* **Published:**
  * `/detected_aruco_id` (`std_msgs/msg/Int32`): ID of the first detected marker.

## Customization
You can remap the image topic via command line if your camera publishes to a different topic:
```bash
ros2 run maya_aruco_detector aruco_detector_node --ros-args -p image_topic:=/my_camera/image
```

## Laptop Camera Usage
To run the detector using the laptop's built-in webcam (without needing an external camera driver), use the provided launch file:
```bash
ros2 launch maya_aruco_detector aruco_webcam.launch.py
```
This launches both:
- `camera_publisher_node`: Captures video from device 0 and publishes to `/camera/image_raw`.
- `aruco_detector_node`: Subscribes to `/camera/image_raw` and detects markers.

## Troubleshooting
### "No executable found"
If you encounter `No executable found` when running the node:
1. Ensure you have sourced the setup file: `source install/setup.bash`
2. If you renamed the package, verify that `setup.cfg` points to the correct library directory (e.g., `lib/maya_aruco_detector`).
3. Try a clean build: `rm -rf build/ maya_aruco_detector install/maya_aruco_detector` followed by `colcon build`.
