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
