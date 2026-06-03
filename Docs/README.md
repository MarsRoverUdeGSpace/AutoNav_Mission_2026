<p align="center">
  <img src="Images/MarsRover.svg" alt="Mars Rover logo" height="200">
</p>


# Autonomous Navigation Mission

This repository contains the **Autonomous Navigation Mission** stack for rover *Maya* in the University Rover Challenge 2026.  
It implements a ROS 2–based perception → planning → control pipeline that uses LiDAR, ZED camera, GNSS, IMU and MCU telemetry to navigate up to 2 km, reach GNSS/visual/object targets within competition tolerances, and report status to operators.  
Development is done simulation-first (Gazebo/Ignition) and then transferred to the real rover for field testing.

---

## Stable release state

As of June 2, 2026, this repository has a stable AutoNav release based on the validated `humble-jetson` hardware branch and promoted through `develop`/`main`.

Validated release highlights:

- GNSS waypoint navigation works successfully on the rover hardware path.
- Direct GNSS waypoint control supports ordered waypoint queues.
- Waypoint acceptance is hardened with a `1.5 m` fallback radius for the validated field workflow.
- Motion-based yaw auto-calibration is integrated for runs where the initial heading must be derived from rover movement.
- The validated minimal odometry baseline is integrated with the Core System firmware contract.
- The Jetson Humble launch path keeps wheel odometry and BNO055 IMU fusion as the local odom baseline, with ZED VSLAM and perception features kept optional.
- ZED/ArUco/YOLO hardware perception hooks are available for later mission-layer use without replacing the GNSS waypoint baseline.

Branch provenance:

- `humble-jetson` carried the hardware validation work and remains the historical source branch for this release.
- `develop` is the stable integration branch after the release squash.
- `main` is the tagged stable release branch.

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
class GnssWp {
  GNSS Waypoint Controller
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
Jetson o-- GnssWp
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
Pub2 --> GnssWp: GNSS fix
Pub5 --> GnssWp: Odom baseline
GnssWp --> Sub1: /cmd_vel
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
