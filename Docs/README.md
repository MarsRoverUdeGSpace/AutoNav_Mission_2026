<p align="center">
  <img src="Images/MarsRover.svg" alt="Mars Rover logo" height="200">
</p>


# Autonomous Navigation Mission

This repository contains the **Autonomous Navigation Mission** stack for rover *Maya* in the University Rover Challenge 2026.  
It implements a ROS 2–based perception → planning → control pipeline that uses LiDAR, ZED camera, GNSS, IMU and MCU telemetry to navigate up to 2 km, reach GNSS/visual/object targets within competition tolerances, and report status to operators.  
Development is done simulation-first (Gazebo/Ignition) and then transferred to the real rover for field testing.

---

```mermaid
%% ROS 2 topic Map con relaciones UML
classDiagram

%% Main Blocks
class Jetson {
  <<component>>
  ROS 2 (Jetson)
}
class MCU {
  <<component>>
  micro-ROS (ESP32-S3)
}

%% Jetson Modules and sensors 
class Lidar {
  /LiDAR/
}
class Cam {
  /ZED/
}
class Slam {
  SLAM / Localización
}
class Nav {
  Nav2 Planner/Controller
}
class Ops {
  Consola/GUI
}

%% Topics de sensores del MCU
class Pub1 { /imu }
class Pub2 { /gnss/fix }
class Pub3 { /altimeter }
class Pub4 { /solar/yaw }
class Pub5 { /odom }
class Pub6 { /roboclaw/status }
class Pub7 { /diagnostics }
class Sub1 { /cmd_vel }
class Sub2 { /mission/abort }

%% Relaciones de agregación (COMPONENTES->SUBSISTEMAS)
Jetson o-- Lidar
Jetson o-- Cam
Jetson o-- Slam
Jetson o-- Nav
Jetson o-- Ops

MCU o-- Pub1
MCU o-- Pub2
MCU o-- Pub3
MCU o-- Pub4
MCU o-- Pub5
MCU o-- Pub6
MCU o-- Pub7
MCU o-- Sub1
MCU o-- Sub2

%% Relaciones (ASOCIACIONES Y DEPENDENCIAS)
Lidar --> Nav : Datos sensores
Cam --> Slam   : Imagenes
Slam --> Nav   : Localización
Ops <--> Pub7  : /diagnostics/
Nav --> Sub1   : /cmd_vel
Ops --> Sub2   : /mission/abort
Pub1 --> Jetson: /imu
Pub2 --> Jetson: /gnss/fix
Pub3 --> Jetson: /altimeter
Pub4 --> Jetson: /solar/yaw
Pub5 --> Jetson: /odom
Pub6 --> Jetson: /roboclaw/status
````

---

## Current autonomy direction: direct GNSS waypoint first

As of 2026-05-13, the immediate Maya GNSS milestone is **not** Nav2 GPS/global localization. The repo contains a checkpointed `feature/global_navigation` scaffold for Gazebo GNSS, georeferenced worlds, `robot_localization` GPS/global EKF, and Nav2 GPS experiments, but that path is intentionally parked until a simpler controller works.

Next implementation target: `src/maya_bringup/scripts/gnss_waypoint_test.py`. The script should subscribe to `/sensors/gnss/fix` and `/odometry/filtered`, compute ENU bearing/distance to a target lat/lon, use encoder+IMU odom yaw for heading control, and publish `/cmd_vel` directly. Nav2 should be off for the first tests.

Key safety/architecture notes:

- GNSS bearing is ENU (`east = 0`, `north = +90 deg`); odom yaw may be boot-relative, so yaw alignment/calibration is mandatory.
- Treat GNSS as noisy global position only; do not use low-speed GNSS course as heading.
- First tests should use a flat `3-5 m` waypoint, speed `<= 0.15 m/s`, and a generous goal radius around `5 m`.
- Direct failsafes must dominate `/cmd_vel`: stale data, bad GNSS/covariance/jumps, excessive pitch/roll, no-progress, yaw-not-improving, and distance-increasing timeout.
- ZED/vSLAM is optional later for obstacle/slope lookahead and odom cross-checking, not a first dependency.

The stable simulation default should remain `random_world.sdf` with `use_gps_localization:=false`; GPS/Nav2 global localization is an explicit experiment, not the default bringup path.

---

## Repo layout

> This is the intended structure for the Autonomous Navigation stack; folders will appear as they are implemented.

| Folder            | What’s inside                                                                           |
| ----------------- | --------------------------------------------------------------------------------------- |
| `src/`            | ROS 2 packages (e.g. `maya_nav2`, `maya_bringup`, `maya_perception`, `maya_mcu_bridge`) |
| `launch/`         | Top-level launch files for simulation and hardware (bringup, Nav2, perception, bridges) |
| `config/`         | YAML configs for Nav2, costmaps, sensors, robot description and MCU/Jetson interfaces   |
| `maps/`           | Saved 2D/3D maps used by localization and planning                                      |
| `sim/`            | Gazebo/Ignition worlds, models, and simulation launch files for the URC environment     |
| `Docs/`           | Mission docs, diagrams (including this README’s Mermaid), URC rule extracts and notes   |
| `LICENSE`         | Project license                                                                         |
| `CONTRIBUTING.md` | Branch workflow, coding standards and contribution guidelines                           |

---
