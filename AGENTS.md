---
# AGENTS – Autonomous Navigation Mission (Maya Rover)

These instructions apply to the entire `AutoNav_Mission_2026` repository.

You are working on the Autonomous Navigation stack for the rover **Maya** for the University Rover Challenge 2026. The project is built around **ROS 2 (Jazzy)**, **Gazebo Sim (Harmonic / gz-sim 8)**, and a simulation-first workflow; code and launch files here are meant to transfer to Jetson hardware later.

Your job is to:
- Make small, correct changes.
- Respect existing structure and intent.
- Prefer clarity and robustness over “clever” one-liners.

If these instructions ever conflict with explicit task instructions, the task instructions win.

---

## 0. Current status (2026-02-03)

- Default sim launch now targets `random_world.sdf` and keeps Gazebo GUI enabled by default (no `-s` in `gz_args`).
- A new random map was generated via `tools/map_gen/build/map_generator`; the updated heightmap PNG is in `src/maya_bringup/world/random_world/media/materials/texture/random_world.png`.
- `island.sdf` remains available and working as a fallback world.
- Open issues:
  - Random heightmap world alignment and robot spawn placement are still not correct.
  - `random_world/model.sdf` likely needs to be adapted to the newly generated map dimensions and scale.
  - The generated `random_map.pgm` has not been moved into a runtime maps folder yet.

## 0.1 Next steps / plan

- Update `src/maya_bringup/world/random_world/model.sdf` to match the generated heightmap’s pixel dimensions and intended real‑world scale.
- Confirm / adjust heightmap `size` (X, Y, Z) and `pos` so the terrain sits at Z=0 and the robot spawns above ground.
- Decide and document a stable spawn pose for `maya` that avoids obstacles.
- Move / register generated map artifacts (`random_map.pgm` and any YAML) into `src/maya_bringup/maps/` and ensure Nav2 can load them.

## 0.2 Autonomy status (2026-02-15)

Current focus is hardware bringup on Jetson (ROS 2 Humble) with ZED + IMU + Nav2.

Latest validation:
- `zed-ros2-wrapper` builds successfully in this workspace with ZED SDK `5.1.0`.
- ZED standalone launch works (`zed_wrapper zed_camera.launch.py camera_model:=zed2i enable_ipc:=false`).
- NITROS transport is available during `zed_components` build.

Current validated HW bringup command (Jetson, Humble):

```bash
ros2 launch maya_bringup maya.launch.xml \
  sim:=false rviz:=false \
  use_zed:=true \
  zed_camera_model:=zed2i \
  zed_enable_ipc:=false \
  odom_topic:=/zed/zed_node/odom \
  imu_topic:=/zed/zed_node/imu/data \
  pointcloud_topic:=/zed/zed_node/point_cloud/cloud_registered
```

**Roadmap to autonomy completeness (rough %)**  
Percentages reflect readiness for field use, not just compile/run.

- **TF + Robot Description**: 70%
  - URDF publishes base + sensors.
  - Wheel meshes and joint states still need validation on HW.
- **Odometry (wheel encoders)**: 20%
  - Plan: PCB computes `/odom` and publishes TF `odom -> base_footprint`.
  - Raw encoder ticks also published for cross-check on Jetson.
- **IMU integration (primary + backup)**: 40%
  - Primary: PCB IMU.
  - Backup: ZED IMU (optional secondary input).
- **LiDAR + scan pipeline**: 50%
  - `/scan` and `/scan_merged` must be verified on HW.
- **2D SLAM (slam_toolbox)**: 60%
  - Mapping runs on HW bringup command above.
  - Drift still present and requires IMU/odom refinement.
- **Nav2 navigation**: 50%
  - Costmaps, planners, and controllers run in HW mode.
  - Needs more stable odom + IMU alignment before long autonomous runs.
- **3D mapping (RTAB-Map / equivalent)**: 10%
  - Planned after LiDAR + encoder odom are stable.
- **GNSS navigation**: 0%
  - Planned after wheel encoders + joint states + robot_state_publisher + IMU2 + 3D mapping are validated.

**Gate to start GNSS work**

1. Wheel encoder odom published from PCB (`/odom` + `odom -> base_footprint` TF).
2. Wheel joint states published (`/joint_states`) for RViz + debugging.
3. robot_state_publisher confirmed on Jetson (Humble) and remote RViz.
4. Secondary IMU (IMU2) optionally fused or at least logged.
5. LiDAR + 3D mapping pipeline producing a consistent local map.

**Immediate next logical steps (execution order)**

1. Implement encoder + odom publishing in Core System (PCB repo) with stable timestamps and diagnostics.
2. Publish wheel raw telemetry (`/wheel/left_ticks`, `/wheel/right_ticks`) and `/odom` from PCB on Jetson network.
3. Configure EKF to fuse PCB odom + primary IMU first; add ZED odom as secondary source after PCB odom is stable.
4. Add `/joint_states` publishing path (PCB or Jetson converter) to restore wheel visualization/debug fidelity.
5. Run short HW validation loops and tune EKF/SLAM covariances based on logged drift and TF consistency.

---

## 1. Tech stack and key tools

- **ROS 2**:
  - Jazzy on development PC (`develop` branch).
  - Humble on Jetson (`humble-jetson` branch).
- **Simulation**: Gazebo Sim (Ignition / gz-sim 8).
- **Languages**:
  - ROS 2 packages: C++, Python, XML launch, Xacro, YAML.
  - Some C / embedded work elsewhere (follow MISRA-C style when touching C).
- **Build system**: `colcon` with `--symlink-install`.
- **Repository layout (current)**:
  - `Docs/` – documentation, diagrams, README.
  - `src/maya_description/` – robot description (URDF/Xacro), meshes, RViz config.
  - `src/maya_bringup/` – simulation bringup (launch, worlds, bridges, joystick, etc.).
  - More Nav2 / perception / MCU bridge packages will appear later; don’t invent them unless explicitly requested.

---

## 2. General behavior

When editing code or configs:

1. **Read before you write.**  
   Before changing any file, scan it completely to understand style, naming, and patterns already in use.

2. **Minimize diffs.**  
   Prefer the smallest change that solves the problem. Do not refactor large areas unless explicitly asked.

3. **Keep things ROS-idiomatic.**
   - Use standard ROS 2 naming where possible (`*_bringup`, `*_description`, etc.).
   - Prefer parameters and launch arguments over hard-coded constants when it clearly improves flexibility.

4. **No “mystery behavior”.**  
   If you add logic, add at least a short comment when the intent would not be obvious to someone new to the project.

5. **Don’t break existing workflows.**  
   Anything that currently builds and launches (e.g., `maya.launch.xml` using `island.sdf`) must keep working unless the instructions explicitly say to replace it.

---

## 3. Build and run commands

When asked to verify changes, prefer these commands:

### Build

From repo root:

```bash
colcon build --symlink-install
source install/setup.zsh   # or setup.bash as appropriate
````

If you only touched one package (e.g. `maya_bringup`):

```bash
colcon build --symlink-install --packages-select maya_bringup
source install/setup.zsh
```

### Run main simulation (current baseline)

From repo root, after sourcing:

```bash
ros2 launch maya_bringup maya.launch.xml
```

This is expected to:

* Start Gazebo Sim with a world (`world/island.sdf` or a random world SDF).
* Spawn the `maya` robot from `maya_description` via `robot_state_publisher` + `ros_gz_sim create`.
* Start `ros_gz_bridge` using `config/gazebo_bridge.yaml`.
* Start RViz with `maya_description.rviz`.

When you modify launch files or worlds that affect this flow, ensure this command still works or is clearly updated in documentation/comments.

The explicit SIM entrypoint is:

```bash
ros2 launch maya_bringup maya_sim.launch.xml
```

---

## 4. Directory-specific guidance

### 4.1 `Docs/`

* Markdown and diagrams only; do not introduce build logic here.
* If you update diagrams or high-level docs, keep terminology consistent with the existing README (e.g. “Maya”, “Autonomous Navigation Mission”, “Jetson”, “MCU bridge”).
* Avoid adding very large assets into `Docs/Images`; prefer vector or small PNGs.

### 4.2 `src/maya_description/`

Purpose: robot model, meshes, and visualization.

* Keep Xacro modular:

  * `common_properties.xacro` – shared materials/constants.
  * `mobile_base.xacro`, `mobile_base_gazebo.xacro` – base structure & simulation tags.
  * `zed_mount.xacro` – camera mount / sensor additions.
  * `maya_description.urdf.xacro` – top-level robot description that composes the pieces.
* When editing, **do not** change link/joint names arbitrarily; they are used by Gazebo and Nav2.
* Inertia, mass and collision geometry may be simplified, but keep them physically reasonable and consistent.
* If you introduce new visual meshes or sensors, keep filenames and case consistent and document them briefly in comments.

### 4.3 `src/maya_bringup/`

Purpose: simulation bringup and (later) hardware bringup.

Current contents:

* `launch/maya_core.launch.xml`
  Core autonomy stack for SIM and HW:

  * Launch `robot_state_publisher`.
  * Run scan pipeline (`scan_frame_relay.py`, `pointcloud_to_laserscan`, `scan_merge.py`).
  * Launch `robot_localization` EKF.
  * Launch Nav2 bringup.
  * Start RViz with `maya_description.rviz`.

* `launch/maya_sensors_sim.launch.xml`
  SIM-only sensor layer:

  * Launch `ros_gz_sim` (`gz_sim.launch.py`) with `gz_args`.
  * Call `ros_gz_sim create` using `/robot_description`.
  * Start `ros_gz_bridge` with `config/gazebo_bridge.yaml`.

* `launch/maya_sim.launch.xml`
  SIM entrypoint: includes `maya_sensors_sim.launch.xml` + `maya_core.launch.xml` with `use_sim_time=true`.

* `launch/maya_hw.launch.xml`
  HW entrypoint: includes `maya_core.launch.xml` with `use_sim_time=false` and optional Jetson sensors.

* `launch/maya.launch.xml`
  Legacy alias for `maya_sim.launch.xml` (must keep working).

* `config/gazebo_bridge.yaml`
  Defines ROS↔GZ bridge topics (e.g., `/cmd_vel`, `/joint_states`; `/tf` only when not using `robot_localization`).

* `config/joy.yaml`
  Used with `teleop_twist_joy` / `joy_linux` for joystick teleop.

* `world/`

  * `island.sdf` – existing world that is known to work.
  * `random_world.sdf` – top-level world using a heightmap model.
  * `random_world/` – a Gazebo model:

    * `model.config`
    * `model.sdf` (heightmap model using `media/materials/texture/random_world.png`)
    * `media/materials/texture/random_world.png` – generated by the map generator.

When editing `maya_bringup`:

* Keep launch files small and readable; prefer `<let>` for paths and a minimal number of arguments.
* Avoid hard-coding machine-specific paths; always derive from `$(find-pkg-share ...)` or `ros2 pkg prefix`.
* If adding new bridge topics, make sure they match actual topics on both ROS and Gazebo sides.

---

---

## 4.4 HITL / Hardware-in-the-loop bringup (SIM vs HW modes)

Purpose: keep a single operator experience (RViz2 + Nav2 + SLAM + EKF + costmaps) while swapping sensor/odom sources.

### Design goals
- The autonomy “core stack” must run without Gazebo present.
- Only the sensor-provider layer changes between simulation and real hardware.
- Minimal duplication: prefer launch arguments + small YAML overlays over duplicating entire configs.

### Layered launch structure (maya_bringup)
- `launch/maya_core.launch.xml`
  - Runs in both SIM and HW.
  - Contains: `robot_state_publisher`, scan pipeline (`scan_frame_relay.py`, `pointcloud_to_laserscan`, `scan_merge.py`), `robot_localization` EKF, Nav2 bringup, RViz.
  - Accepts `use_sim_time` argument and applies it consistently.
- `launch/maya_sensors_sim.launch.xml`
  - SIM-only: Gazebo Sim Harmonic + robot spawn + `ros_gz_bridge` using `config/gazebo_bridge.yaml`.
- `launch/maya_sim.launch.xml`
  - SIM entrypoint: includes sensors_sim + core with `use_sim_time=true`.
- `launch/maya_hw.launch.xml`
  - HW entrypoint: includes core with `use_sim_time=false` and optionally includes a guarded Jetson sensor bringup include.
  - Must not launch Gazebo and must not crash if Jetson-specific packages are absent.

`launch/maya.launch.xml` must continue to work as the primary developer entrypoint (it may wrap/alias `maya_sim.launch.xml`).

### Stable Topic / TF contract
Keep these stable across SIM and HW modes so Nav2/RViz config remains unchanged:

Raw inputs (provided by Gazebo in SIM; by Jetson in HW):
- `/scan` (LaserScan)
- `/depth_camera/points` (PointCloud2)
- `/imu` (sensor_msgs/Imu)
- `/odom` (nav_msgs/Odometry)

Derived/core outputs:
- `/scan_fixed` (relay output, frame_id=lidar_link)
- `/scan_depth` (from depth pointcloud)
- `/scan_merged` (Nav2/SLAM scan source)
- `/odometry/filtered` (EKF output)
- `/tf`, `/tf_static`

TF ownership rules:
- Odometry side publishes `odom -> base_footprint` (typically EKF or wheel odom)
- Localization (slam_toolbox / AMCL) publishes `map -> odom`
- URDF publishes static transforms from `base_* -> sensor frames`

### Time rules (critical)
- SIM mode: `/clock` exists; core nodes use `use_sim_time=true`.
- HW mode: no `/clock` required; core nodes use wall time (`use_sim_time=false`).
Do not mix sim-time and wall-time within the same autonomy graph.

### Debug commands (TF + time)
- `ros2 run tf2_ros tf2_echo map odom`
- `ros2 run tf2_ros tf2_echo odom base_footprint`
- `ros2 topic echo --once /clock` (SIM only)
- `ros2 topic hz /scan_merged`
- `ros2 topic hz /depth_camera/points`

## 5. Style and safety notes

### 5.1 C / embedded (if touched here later)

* Follow **MISRA-C-like** principles:

  * No hidden side effects in macros.
  * Avoid dynamic allocation where possible.
  * Prefer explicit types (`uint32_t`, `int32_t`, etc.).
  * Avoid implicit casts; use explicit, checked casts.
* Always check error returns and handle them; don’t leave TODOs for critical paths.

### 5.2 C++ / ROS 2

* Use RAII and smart pointers (`std::shared_ptr` / `std::unique_ptr`).
* No bare `new`/`delete` unless absolutely required.
* Follow ROS 2 naming conventions: node names lower_snake_case, parameters lower_snake_case.
* Prefer `rclcpp::Logger` for logging.

### 5.3 Launch / YAML / SDF

* Keep XML/YAML formatted and indented consistently (2 spaces).
* Comments should explain *why* something is there (e.g., workarounds, known limitations), not just *what* it does.
* For SDF/world files:

  * Use relative URIs that resolve via Gazebo resource paths:

    * Model URIs: `model://random_world`
    * Textures: `file://media/materials/texture/random_world.png` or similar **only if** they resolve correctly under `GZ_SIM_RESOURCE_PATH`.
  * Avoid overly complex shaders or plugins unless necessary.

---

## 6. Checks to run (when possible)

When a task involves code or configuration changes, prefer to run:

1. Build:

   ```bash
   colcon build --symlink-install
   ```
2. Source:

   ```bash
   source install/setup.zsh
   ```
3. Basic sim sanity (if relevant to the change):

   ```bash
   ros2 launch maya_bringup maya.launch.xml
   ```

   or, for world-only debugging:

   ```bash
   gz sim "$(ros2 pkg prefix --share maya_bringup)/world/random_world.sdf"
   ```

If a task only touches documentation (Markdown), you do not need to run code, but you should still ensure that file paths and command examples are correct and consistent.

---

## 7. Current task brief – Nav2 bringup, SLAM, mapping/localization

**Context**

* Focus is now on Nav2 bringup with SLAM to ensure mapping and localization are reliable.
* The IMU config has been edited and performance improved, but the map still drifts.

**What we are trying to achieve right now**

* A stable Nav2 bringup path with SLAM that reduces or eliminates map drift.
* Clear, minimal configuration changes that improve localization consistency.
* A repeatable refinement workflow that balances SIM-first tuning with short HW validation cycles.

**Refinement strategy (SIM vs HW)**

1. Tune algorithmic and config behavior in simulation first (Nav2, SLAM, EKF params).
2. Validate only high-value changes on Jetson hardware in short, controlled runs.
3. Treat HW runs as acceptance tests for TF integrity, topic rates, and drift, not first-pass tuning.
4. Keep branch diffs small and isolate changes by subsystem (IMU, odom, SLAM, costmaps).

**How to treat this in changes**

When a task mentions Nav2, SLAM, or IMU tuning:

1. Prefer configuration-level fixes first (SLAM params, Nav2 params, IMU integration).
2. Keep diffs minimal and document why changes are needed if not obvious.
3. Preserve existing launch entry points unless explicitly asked to restructure.

---

## 8. SIM Baseline Sync from `develop` (2026-02-24)

This section mirrors the current known-good SIM findings from the `develop` branch so Jetson/HW work can track a stable reference.

### 8.1 Confirmed working SIM baseline (2D SLAM + forward/back autonomous return)

- 2D lidar SLAM (`slam_toolbox`, `scan_topic: /scan`) is operational in SIM after the Gazebo render backend fix (`ogre2`).
- Nav2 can perform repeatable warmup-forward + return-to-reference runs in SIM (headless reliability script).
- A validated baseline on `develop` achieved repeatable forward/back autonomous runs (5/5 in the initial checkpoint; later 20-trial follow-up remained broadly functional at 18/20).
- Interpretation:
  - End-to-end Nav2 in SIM works.
  - Current tuning focus is localization consistency / turning behavior, not basic command-chain viability.

### 8.2 IMU frame-id integration bug (critical lesson, fixed on `develop`)

- Root cause found in SIM:
  - `/imu.header.frame_id` was a Gazebo-scoped sensor name that did not exist in the ROS TF tree.
- Effective fix on `develop`:
  - explicitly set Gazebo IMU `gz_frame_id` to `imu_link` in `src/maya_description/urdf/sensors_gazebo.xacro`.
- After the fix (observed on `develop`):
  - `/imu.header.frame_id` became `imu_link`
  - pose/orientation consistency during turning improved noticeably in RViz.
- Carry-forward rule:
  - validate message `frame_id` values (`/imu`, `/scan`, `/odom`, future `/visual_odom`) against TF before tuning filters/SLAM.

### 8.3 EKF / IMU tuning result to preserve

- `imu0_relative: true -> false` was tested in SIM after the IMU frame fix and made heading behavior worse.
- Keep `imu0_relative: true` in the current baseline unless a new controlled A/B test shows otherwise.
- Remaining known issue:
  - SIM `/imu.orientation_covariance` and Gazebo `/odom` covariances may be unrealistically zero, which can distort EKF source weighting.

### 8.4 Clean localization tuning baseline defaults (no depth/VIO)

- `develop` was cleaned to a simpler baseline for localization tuning:
  - depth scan pipeline disabled by default
  - VIO/RTAB-Map odometry disabled by default
- Runtime intent of that baseline:
  - EKF local odom = `/odom` + `/imu`
  - `slam_toolbox` = lidar scan (`/scan`) + EKF odom prior
  - Nav2 unchanged
- Terminology note:
  - this is a loose-coupled EKF odom + lidar SLAM baseline, not tight LIO.

### 8.5 Optional RTAB-Map / VIO experiments (not baseline)

- `develop` has optional launch scaffolding for RTAB-Map RGB-D odometry and `/visual_odom` EKF fusion testing.
- Current observed RTAB-Map odom state in SIM was not valid for fusion:
  - odometry lost (`/odom_info.lost: true`)
  - `inliers: 0`
  - invalid quaternion / `9999` covariance in `/visual_odom`
- Rule:
  - do not enable VIO odom fusion in regression runs until `/visual_odom` is demonstrably valid and stable.

### 8.6 Reliability diagnostics upgrade (paired turning stress test)

- `tools/nav2_reliability_trials.sh` on `develop` now supports an optional dual-phase per-trial mode:
  - `phase_a`: easy warmup + return
  - `phase_b`: warmup + forced in-place turn (default 90 deg) + return
- Purpose:
  - quantify turning-induced degradation within the same startup/map conditions.
- This is useful for future backports or equivalent diagnostics on `humble-jetson` after HW odom/IMU are stable.

### 8.7 Future-proof integration rules (carry forward)

- For any new odometry source (encoders, VIO, GNSS fusion):
  1. verify topic exists
  2. verify `header.frame_id` and `child_frame_id`
  3. verify covariance sanity (non-zero, realistic)
  4. only then fuse into EKF/Nav2
- Keep optional integrations disabled by default until they produce valid data.
- Prefer within-run paired diagnostics (easy vs stress) when analyzing turning regressions to reduce startup/transient confounds.

### 8.8 Humble-Jetson HW Bringup Milestone (LD19 + ZED + SLAM + ArUco) (2026-02-26)

- LD19 hardware LiDAR was installed and validated on Jetson (`/dev/ttyUSB1`) using `ldlidar_stl_ros2`.
- Verified from PC over ROS 2 network:
  - `/scan` publishes valid `LaserScan` data at ~`10 Hz`
  - `frame_id` is currently `base_laser` (kept intentionally because vendor driver was stable in this mode)
- `maya.launch.xml` on `humble-jetson` now supports optional LD19 HW bringup in `sim:=false` mode:
  - `use_ld19:=true`
  - `lidar_port:=/dev/ttyUSB1`
  - guarded vendor-style static TF for `base_link -> base_laser`
  - HW scan relay keeps `base_laser` by default to avoid mislabeling a working stream
- Selective backport from `develop` applied to `humble-jetson`:
  - lidar-only Nav2/SLAM baseline (`/scan` active source)
  - relaxed progress checker
  - higher forward speed caps
  - collision monitor observing lidar scan topic
  - `slam_toolbox` `max_laser_range` aligned to `12.0`
- Integrated HW bringup confirmed working with:
  - `LD19` (`/scan`)
  - `ZED` odom + IMU (`/zed/zed_node/odom`, `/zed/zed_node/imu/data`)
  - `slam_toolbox` mapping in HW mode
  - ArUco detector integrated through `maya.launch.xml`
- ArUco integration update:
  - `maya.launch.xml` now exposes annotated output mode/topic parameters, including raw annotated image publishing for RViz debugging.
  - Use `aruco_publish_annotated_raw:=true` to publish `/aruco/annotated_image/raw` (`sensor_msgs/Image`) when RViz rendering of compressed annotated stream is problematic.
- RViz troubleshooting note (important):
  - LaserScan displays for `/scan` and `/scan_fixed` may require `Best Effort` QoS in RViz.
  - With current HW baseline, set RViz fixed frame to `base_laser` (or another TF-connected frame) while validating the raw lidar stream.

### 8.9 Next Integration Priority – YOLO Object Detection (High-Level Plan) (2026-02-26)

- Next logical step after the current HW milestone is integrating YOLO object detection as an **optional perception node** in `maya.launch.xml` on `humble-jetson`.
- Current status:
  - `.pt` model is already available on the Jetson side.
  - Core sensing + localization stack is working (LD19 + ZED + SLAM + ArUco), so YOLO can be added without debugging basic bringup at the same time.
- Integration intent (initial phase):
  - Subscribe to **ZED compressed RGB image** topic (to match current working bandwidth/profile setup).
  - Publish detections / annotated outputs for visualization and validation only.
  - Keep YOLO decoupled from Nav2/SLAM/EKF (no autonomy behavior coupling yet).
- Guardrails:
  - Add YOLO as a launch-toggle subsystem (`use_yolo:=true/false`, default off).
  - Do not modify costmaps / planners / BTs until detections are validated and topic contracts are stable.
  - Treat message type/QoS compatibility (especially compressed image transport) as first-class validation checks before optimization.

### 8.10 YOLO Integration Status (CPU-Only) and ArUco Isolation Rule (2026-02-27)

- YOLO integration in `maya_bringup` is functional:
  - subscribes to ZED compressed RGB input
  - publishes detections and annotated outputs
  - raw annotated image output is available for RViz (`/yolo/annotated_image/raw`)
- Current runtime limitation on Jetson:
  - CUDA-enabled PyTorch wheel source was not reachable from network/DNS, so current YOLO runtime is CPU-only.
  - Use `yolo_device:=cpu` until Jetson CUDA wheel installation path is fixed.
- Operational isolation rule:
  - When validating ArUco behavior/regression, launch with `use_yolo:=false` to remove perception-resource contention and topic overlap confounds.
  - Re-enable YOLO only after ArUco topic/output is confirmed healthy.

### 8.11 Humble-Jetson Minimal Encoder Odom Validation Milestone (2026-04-16)

- A new minimal hardware validation path was established on branch `humble-jetson-minimal-odom` to bring up only:
  - `robot_state_publisher`
  - wheel encoder odom (`encoder_odom.py`)
  - `robot_localization` EKF
- Purpose:
  - validate the hardware odom + IMU + TF chain before re-enabling Nav2, LiDAR, ZED, or other autonomy subsystems.
- Hardware topic contract used in this milestone:
  - IMU: `/sensors/bno055/imu/data`
  - left encoder ticks: `/sensors/roboclaw/encoders/left_m1/ticks`
  - right encoder ticks: `/sensors/roboclaw/encoders/right_m1/ticks`
  - optional diagnostic rates:
    - `/sensors/roboclaw/encoders/left_m1/qpps`
    - `/sensors/roboclaw/encoders/right_m1/qpps`
- Current wheel geometry assumptions for this path:
  - wheel separation: `1.0 m`
  - wheel radius: `0.1636 m`
  - encoder ticks per revolution: `7400` (validated hardware setting; do not revert to the older `2048` assumption).

Validated launch command (Jetson, Humble, minimal odom test):

```bash
ros2 launch maya_bringup maya.launch.xml \
  sim:=false rviz:=false \
  use_nav2:=false \
  use_zed:=false \
  use_ld19:=false \
  use_depth_scan:=false \
  use_aruco:=false \
  use_yolo:=false \
  use_ekf:=true \
  use_encoder_odom:=true \
  odom_topic:=/odom \
  imu_topic:=/sensors/bno055/imu/data \
  left_encoder_ticks_topic:=/sensors/roboclaw/encoders/left_m1/ticks \
  right_encoder_ticks_topic:=/sensors/roboclaw/encoders/right_m1/ticks \
  encoder_msg_type:=std_msgs/msg/Int32 \
  encoder_ticks_per_rev:=7400 \
  encoder_wheel_radius:=0.1636 \
  encoder_wheel_separation:=1.0 \
  encoder_publish_tf:=false
```

Code/launch lessons captured in this milestone:
- `maya.launch.xml` on Humble cannot use the XML frontend expression:
  - `$(eval 'not ' + var('sim'))`
- For Humble XML compatibility, `use_encoder_odom` must remain a plain boolean arg and be passed explicitly in HW launches.
- Raw PCB encoder publishers were discovered to use incompatible reliability with the initial subscriber setup.
- `encoder_odom.py` had to be updated to subscribe with `BEST_EFFORT` QoS so it could receive hardware tick topics reliably.
- With those fixes applied, the minimal stack successfully produced:
  - `/odom`
  - `/odometry/filtered`
  - TF `odom -> base_footprint`

Observed validation result:
- The plumbing path is now confirmed healthy:
  - encoders -> `encoder_odom.py` -> `/odom` -> EKF -> `/odometry/filtered` + TF
- Manual hardware movement produced live odom and TF updates, confirming the minimal chain is functional.
- Example validated behavior:
  - forward motion near `0.51 m`
  - small initial yaw drift around `-0.95 deg`
  - EKF and TF visibly tracked motion in real time

Status update after 2026-04-26 hardware validation:
- Estimation quality is now good enough for controlled motion testing with Nav2 still disabled.
- Wheel encoder odom + BNO055 IMU fusion produced an almost perfect square pattern while correcting yaw orientation online.
- The validated square-test path used movement commands only: launch the minimal EKF/encoder/IMU stack, then run `square_test.py`.
- The prior jumpy-odom concern should remain in mind, but the current validated result means the next work is Nav2 re-enable / stack integration, not basic odom plumbing.

Priority after this milestone:
1. Preserve the validated encoder + BNO055 + EKF baseline as the hardware truth.
2. Reintroduce LD19 + SLAM + Nav2 conservatively on top of that baseline.
3. Keep VSLAM optional during first Nav2 re-enable; do not let it publish competing TF or replace wheel odom.
4. Use simulation next to match the validated hardware-software integration contracts, not to invent a separate autonomy path.

Recommended debug commands for this stage:

```bash
ros2 topic echo /sensors/roboclaw/encoders/left_m1/ticks
ros2 topic echo /sensors/roboclaw/encoders/right_m1/ticks
ros2 topic echo /odom
ros2 topic echo /odometry/filtered
ros2 topic echo /sensors/bno055/imu/data
ros2 run tf2_ros tf2_echo odom base_footprint
```

### 8.12 Humble-Jetson Lightweight VSLAM Validation Milestone (2026-04-16)

- A lightweight ZED VSLAM bringup path was integrated into `maya.launch.xml` on branch `humble-jetson-minimal-odom`.
- Purpose:
  - validate a Jetson-local stereo VSLAM pipeline that is compatible with low-bandwidth remote RViz use over Wi-Fi.
  - keep the VSLAM subsystem separate from Nav2 and EKF during first validation.
- Design rule established in this milestone:
  - primary IMU for this hardware stack remains `/sensors/bno055/imu/data`
  - ZED IMU must not silently become the default IMU for the rover autonomy baseline

Validated isolated VSLAM launch intent (Jetson, Humble):

```bash
ros2 launch maya_bringup maya.launch.xml \
  sim:=false \
  rviz:=false \
  use_nav2:=false \
  use_ekf:=false \
  use_encoder_odom:=false \
  use_ld19:=false \
  use_depth_scan:=false \
  use_aruco:=false \
  use_yolo:=false \
  use_zed:=true \
  use_vslam:=true \
  zed_camera_model:=zed2i \
  zed_camera_name:=zed2i \
  zed_node_name:=zed_node \
  zed_enable_ipc:=false \
  zed_publish_tf:=false \
  zed_publish_map_tf:=false \
  use_zed_static_tf:=false \
  vslam_imu_topic:=/sensors/bno055/imu/data \
  vslam_enable_landmarks_view:=true \
  vslam_enable_observations_view:=false \
  vslam_enable_slam_visualization:=true
```

Implementation/bringup lessons captured:
- `maya.launch.xml` now supports an explicit `use_vslam` path instead of requiring a manual 3-terminal workflow.
- `zed_bgra_to_rgb.py` was parameterized so it no longer depends on one hard-coded ZED namespace.
- `zed_vslam.launch.py` had to be reworked to avoid Humble launch frontend/type issues:
  - component parameter arrays must be passed as real resolved Python values
  - simple literal defaults are more reliable than clever launch substitution composition in this path
- Isaac ROS VSLAM rejects odd image dimensions:
  - previous ZED custom publish size produced `427x240`
  - VSLAM failed with `Odd Image width or height`
  - `pub_downscale_factor` was changed to `5.0`, producing an even image size (`384x216`) suitable for VSLAM

ZED VSLAM profile rules established:
- use a dedicated override file:
  - `src/maya_bringup/config/zed2i_vslam_override.yaml`
- keep the ZED ROS graph minimal:
  - stereo images enabled
  - camera info enabled as needed by wrapper
  - IMU enabled
  - depth disabled
  - point cloud disabled
  - object detection disabled
  - body tracking disabled
  - mapping disabled
  - status disabled
- current lightweight defaults:
  - `pub_frame_rate: 8.0`
  - `pub_resolution: CUSTOM`
  - `pub_downscale_factor: 5.0`

Observed validation result:
- The integrated VSLAM component now loads and initializes successfully.
- Relevant VSLAM topics observed:
  - `/visual_slam/tracking/odometry`
  - `/visual_slam/tracking/slam_path`
  - `/visual_slam/tracking/vo_path`
  - `/visual_slam/tracking/vo_pose`
  - `/visual_slam/tracking/vo_pose_covariance`
  - `/visual_slam/vis/landmarks_cloud`
  - `/visual_slam/vis/observations_cloud`
  - `/visual_slam/vis/slam_odometry`
  - additional pose-graph/localizer visualization topics
- Current remote-operator interest was narrowed to:
  - `/visual_slam/vis/landmarks_cloud`
  - `/visual_slam/vis/observations_cloud`
  - `/visual_slam/tracking/odometry`
  - `/visual_slam/tracking/slam_path`

Bandwidth result from image-side measurements:
- left/right ZED raw rect images and the local rgb8 republished images together are roughly within about `0.9 MB/s` to `1.2 MB/s` steady-state in the tested configuration.
- This is within the practical Wi-Fi target budget, but the remaining bandwidth risk is likely dominated by VSLAM visualization topics rather than the downscaled stereo images themselves.

Important functional note:
- `vslam_enable_slam_visualization:=false` did not remove all of the extra VSLAM visualization outputs as hoped.
- Operationally, topic-level selection on the PC side and/or future deeper launch/runtime pruning is still required.

Interpretation of this milestone:
- isolated lightweight VSLAM is now structurally validated
- minimal encoder odom + EKF is structurally validated
- the next problem is integration quality, not basic bringup

Next logical steps toward autonomy integration (execution order):
1. Measure the actual bandwidth/rate cost of the VSLAM topics that matter most:
   - `/visual_slam/vis/landmarks_cloud`
   - `/visual_slam/vis/observations_cloud`
   - `/visual_slam/tracking/odometry`
   - `/visual_slam/tracking/slam_path`
2. Define the minimal remote VSLAM topic contract for Wi-Fi mode:
   - keep only operator-useful topics subscribed from the PC
   - treat landmarks + tracking odometry/path as preferred candidates
3. Inspect the VSLAM topic/frame contract before fusion:
   - `header.frame_id`
   - `child_frame_id`
   - covariance sanity
   - update rate
   - failure behavior when visual features are weak/lost
4. Integrate VSLAM conservatively with the minimal odom milestone:
   - wheel encoder odom remains the baseline local motion source
   - BNO055 remains the primary IMU
   - VSLAM becomes a secondary visual odometry / drift-reduction source
5. Tune `src/maya_bringup/config/ekf.yaml` for the fused HW stack:
   - wheel odom + BNO055 first
   - then add VSLAM only after its outputs are trusted
6. Re-enable Nav2 only after the fused estimate is stable enough that:
   - `odom -> base_footprint` is smooth
   - pose jumps are eliminated or rare enough to be operationally acceptable
   - bandwidth remains within Wi-Fi operating limits with headroom

Integration principle locked by this milestone:
- VSLAM should augment the minimal encoder-odom baseline, not replace it.
- The rover must retain a usable local odom solution if visual tracking degrades.

EKF inference locked after review of `src/maya_bringup/config/ekf.yaml`:
- The current EKF should continue to serve as the **local odom filter**:
  - `world_frame: odom`
  - EKF owns `odom -> base_footprint`
- The safe baseline remains:
  - `odom0 = /odom` from wheel encoder odom
  - `imu0 = /sensors/bno055/imu/data`
- Do not let VSLAM publish competing local TF ownership into the same chain used by Nav2.
- Do not replace the wheel-odom baseline with VSLAM.
- Do not fuse multiple rotational sources aggressively at first.

Conservative fusion path inferred from the current validated subsystems:
1. Stabilize encoder odom + BNO055 + EKF first.
2. Inspect one real message from:
   - `/visual_slam/tracking/odometry`
3. Verify before fusion:
   - `header.frame_id`
   - `child_frame_id`
   - covariance sanity
   - update rate
   - behavior during feature loss
4. If the topic contract is sane, add VSLAM as a **secondary odometry source** in EKF:
   - preferred first candidate: `odom1 = /visual_slam/tracking/odometry`
5. First fusion should be conservative:
   - prefer planar pose correction from VSLAM (`x`, `y`, `yaw`)
   - keep wheel odom as the main short-term motion prior
   - keep BNO055 as the primary yaw / yaw-rate source
   - do not fuse extra VSLAM twists until they are shown to be stable and useful
6. Re-enable Nav2 only after the fused local odom estimate is smooth under:
   - slow straight drive
   - slow in-place turn

Operational rule from this inference:
- VSLAM is a drift-reduction / visual odom aid layered on top of the minimal odom milestone.
- If vision degrades, the rover must still have a usable EKF odom chain from encoders + BNO055.

### 8.13 Hardware Motion + Wi-Fi VSLAM Validation Milestone (2026-04-26)

Validated minimal motion stack:

```bash
ros2 launch maya_bringup maya.launch.xml \
  sim:=false \
  rviz:=false \
  use_nav2:=false \
  use_zed:=false \
  use_vslam:=false \
  use_ld19:=false \
  use_depth_scan:=false \
  use_aruco:=false \
  use_yolo:=false \
  use_ekf:=true \
  use_encoder_odom:=true \
  use_imu_sanitizer:=true \
  encoder_ticks_per_rev:=7400 \
  encoder_wheel_separation:=1.0 \
  encoder_publish_tf:=false
```

Validated square-test command:

```bash
ros2 run maya_bringup square_test.py --ros-args \
  -p side_length:=5.0 \
  -p turn_angle_deg:=90.0 \
  -p linear_speed:=0.15 \
  -p angular_speed:=0.25 \
  -p heading_gain:=1.25 \
  -p max_heading_correction:=0.1 \
  -p pause_sec:=1.0
```

Observed result:
- Wheel encoder odom + BNO055 IMU fusion achieved an almost perfect square pattern.
- Yaw orientation was corrected online during the drive segments.
- The test did not require Nav2, LiDAR, ZED, VSLAM, ArUco, YOLO, or depth scan layers.
- `square_test.py` now supports both right and left turns through the sign of `turn_angle_deg`.
- `imu_sanitize_relay.py` must be executable because it is launched as a runtime script.

Validated VSLAM-over-Wi-Fi path:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
  camera_model:=zed2i \
  camera_name:=zed2i \
  node_name:=zed_node \
  publish_tf:=false \
  publish_map_tf:=false \
  enable_ipc:=false \
  ros_params_override_path:=/home/ro/AutoNav_Mission_2026/src/maya_bringup/config/zed2i_vslam_override.yaml

ros2 run maya_bringup zed_bgra_to_rgb.py
ros2 launch maya_bringup zed_vslam.launch.py
```

Observed result:
- VSLAM can run over the limited Wi-Fi link in the tested lightweight ZED configuration.
- This upgrades VSLAM from "structurally integrated" to "bandwidth-viable as an optional local/remote aid".
- It still must not replace the wheel encoder + IMU odom baseline or publish competing TF in the Nav2 chain.

Mission-status interpretation:
- The hardware baseline has crossed from bringup/plumbing into controlled autonomy integration.
- Next engineering target is Nav2 re-enable on top of the validated encoder + IMU + EKF motion baseline.
- Simulation work should now be made compatible with this validated hardware-software contract.

### 8.14 Wireless-First Integration Rule and Fresh Validation Order (2026-04-16 / refreshed 2026-04-26)

- After bringing up the combined stack (`encoders + BNO055 + LD19 + EKF + SLAM + Nav2 + ZED VSLAM`), the next architecture decision was clarified:
  - the main problem is no longer "can all subsystems launch together?"
  - the main problem is "what must stay local on Jetson and what is actually worth transmitting over the wireless link?"
- The correct operating model for the field stack is now:
  - compute locally on Jetson
  - fuse locally on Jetson
  - transmit only operator-critical outputs
  - avoid streaming raw intermediate perception topics unless explicitly needed for debugging

Wireless-first integration rule:
- Anything that can be computed locally without being published for remote consumption should remain local.
- The wireless link should be treated as an operator/control channel, not as a full raw-sensor replication channel.
- The goal is to preserve headroom and reliability on the link, not merely to fit under an optimistic throughput limit.

Topics/classes of data that should remain local on Jetson by default:
- raw encoder tick topics
- raw IMU streams
- raw `/scan` as a permanent remote feed
- raw stereo image topics
- most VSLAM internal/debug visualization topics
- dense intermediate clouds unless a specific debugging need justifies them
- high-rate estimator internals that do not directly improve operator decisions

Topics/classes of data that are appropriate to expose remotely for normal operation:
- compressed operator image stream
- `/map`
- `/tf`
- `/tf_static`
- `/odometry/filtered`
- goal / waypoint / spin interfaces
- Nav2 status / feedback
- optionally a small number of reduced, operator-useful visualization topics if they are proven worth the bandwidth

Odometry-source interpretation refined during this stage:
- The rover now has multiple odometry/localization ingredients available or partially available:
  - wheel encoder odom
  - IMU
  - lidar-driven localization / SLAM contribution
  - visual odometry / visual SLAM contribution
- These must not be treated as equal just because they exist.
- Before any source is trusted as part of the field autonomy baseline, it must be evaluated for:
  - frame contract
  - covariance sanity
  - update rate
  - stationary drift
  - behavior during degradation or loss

Current practical architecture direction:
- local motion prior / fallback:
  - wheel encoders
  - BNO055 IMU
- local/global correction sources:
  - lidar-based SLAM/localization
  - VSLAM only if it provides net value
- local fused pose for autonomy:
  - `/odometry/filtered`
  - EKF-owned `odom -> base_footprint`
- localization layer:
  - `map -> odom`

Important integration principle locked here:
- Do not keep all available odometry sources in the field stack by default just because they can be launched together.
- The first field-capable wireless autonomy stack should prefer the smallest onboard stack that works reliably.
- VSLAM is now considered optional until it proves that it materially improves autonomy quality without destabilizing TF, overloading Jetson, or consuming too much wireless/debug budget.

Recommended minimum field-capable autonomy baseline for wireless operation:
- wheel encoder odom
- BNO055 IMU
- EKF
- LD19
- `slam_toolbox`
- Nav2
- one compressed ZED operator-view stream

Recommended interpretation of VSLAM at this stage:
- keep VSLAM as an optional local aid
- do not assume it belongs in the first wireless field baseline
- only retain it in the baseline if side-by-side tests show a clear net gain

Fresh validation order for the next session:
1. Re-test the minimum local autonomy stack **without VSLAM**:
   - encoders + BNO055 + EKF + LD19 + SLAM + Nav2
2. Validate that this minimum stack is autonomously usable before adding more odometry sources.
3. Measure the remote wireless budget for only:
   - compressed image
   - `/map`
   - `/tf`
   - `/tf_static`
   - `/odometry/filtered`
   - goals / feedback
4. Confirm that remote RViz and operator control remain reliable with that minimal export contract.
5. Only then repeat the same test with VSLAM enabled locally.
6. Compare:
   - autonomy quality
   - TF stability
   - Jetson load
   - bandwidth/debug cost
7. Keep VSLAM in the baseline only if it is clearly a net win.

Next logical step for a fresh session:
- start from the minimum wireless-ready autonomy stack **without VSLAM**
- verify it can:
  - launch cleanly
  - maintain `odom -> base_footprint`
  - maintain `map -> odom`
  - accept short Nav2 goals
  - remain usable over the intended wireless link
- after that baseline is accepted, run the exact same autonomy test again with VSLAM enabled and judge whether it should remain part of the field stack.
