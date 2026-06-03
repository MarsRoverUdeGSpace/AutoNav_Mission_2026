# Test Log

Short, consistent entries make it easy to correlate field tests to commits/tags.

## Template

```text
Date: 2026-02-10
Branch: humble/scan-tune-2026-02-10
Commit: <git rev-parse --short HEAD>
Tag: test-humble-2026-02-10a
Robot: Maya (Jetson Orin Nano, JP 6.2, Ubuntu 22.04, ROS 2 Humble)
Mode: HW | SIM
What changed:
- <one line summary>
Test procedure:
- <how you ran it>
Result:
- PASS | FAIL
Notes:
- <anything important>
```

## Log

Date: 2026-06-02
Branch: humble-jetson
Commit: 757f8f6
Tag: v1.0.0
Robot: Maya (Jetson Orin Nano, JP 6.x, Ubuntu 22.04, ROS 2 Humble)
Mode: HW
What changed:
- Promoted validated GNSS waypoint navigation to stable release.
- Integrated direct GNSS waypoint controller, ordered waypoint queues, 1.5 m fallback waypoint acceptance, motion-based yaw auto-calibration, and the validated minimal odom baseline.
- Kept ZED VSLAM, ArUco, YOLO, and LD19 hardware bringup paths optional around the wheel odom + BNO055 local odometry baseline.
Test procedure:
- Field validation reported by maintainer: GNSS waypoint navigation completed successfully using the hardware/Jetson path.
- Release branch history reviewed from `humble-jetson` through `develop` and `main`.
Result:
- PASS
Notes:
- Core System stable release provides the firmware-owned `/odom`, BNO055 IMU/magnetometer, GNSS, and `cmd_vel` contract consumed by this AutoNav release.
- `humble-jetson` remains the source-of-truth historical branch for the field changes; `develop` and `main` carry the stable release promotion.
