# Run AutoNav with Ubuntu 24.04 and ROS 2 Jazzy

This guide covers an Ubuntu 24.04 desktop **or a Fedora 44 desktop running an Ubuntu 24.04 container**, with an Intel/AMD GPU. It starts the Maya simulation with both Gazebo and RViz windows. Use a terminal opened **on the graphical desktop**, not an SSH session. NVIDIA, macOS, Windows, and headless machines need a different, tested GUI setup.

The image installs ROS 2 Jazzy, Gazebo, RViz, and workspace dependencies. Your Git clone contains the code and meshes; your `colcon` build stays in that clone. The two are connected by a Docker bind mount. You do **not** need ROS installed on Ubuntu itself.

## First time (one computer)

1. Install [Docker Engine and the Compose plugin](https://docs.docker.com/engine/install/ubuntu/). Make sure you can run `sudo docker compose version`. Install the host tools:

   ```bash
   sudo apt-get update
   sudo apt-get install -y git git-lfs xauth
   git lfs install
   ```

2. Get repository access from the team. If GitHub SSH is not set up yet, follow [GitHub's SSH instructions](https://docs.github.com/en/authentication/connecting-to-github-with-ssh). Clone the branch containing this guide and download the large rover meshes:

   ```bash
   cd ~
   git clone --branch develop git@github.com:MarsRoverUdeGSpace/AutoNav_Mission_2026.git
   cd AutoNav_Mission_2026
   git lfs pull
   ```

   Stay on the branch containing this guide. Checking out the older `559e36b4...` commit removes these newer Docker files. If you clone this history from the 2027 repository instead, explicitly clone its `dev` branch into a directory named `AutoNav_Mission_2026` until the hard-coded container workspace path is migrated.

3. Check the display, GPU access, and mesh files. Run this **without sudo**:

   ```bash
   bash tools/prepare_gui_env.sh
   ```

   It must say `Host preflight passed`. It creates an ignored `.env.autonav` with paths specific to your computer; the X11 access cookie stays outside Git.

4. Build the image, start the container, and build the ROS workspace. Run these from the repository root:

   ```bash
   sudo docker compose --env-file .env.autonav build jazzy
   sudo docker compose --env-file .env.autonav up -d jazzy
   sudo docker compose --env-file .env.autonav exec jazzy bash -lc '. /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
   ```

   The workspace build should finish with `2 packages finished`. Keep the container running for the next step. The ROS commands run **inside the Ubuntu 24.04 container**, not directly on the host.

5. Launch the simulation (no ROS launch arguments):

   ```bash
   sudo docker compose --env-file .env.autonav exec jazzy bash -lc '. /opt/ros/jazzy/setup.bash && . install/setup.bash && exec ros2 launch maya_bringup maya.launch.xml'
   ```

   You should see **both Gazebo and RViz**, with Maya visible in Gazebo. In a second host desktop terminal, check that the simulated lidar is publishing:

   ```bash
   sudo docker compose --env-file .env.autonav exec jazzy bash -lc '. /opt/ros/jazzy/setup.bash && . install/setup.bash && ros2 topic echo --once /scan'
   ```

   Press Ctrl-C in the launch terminal when done. A launched process, two visible windows, and a live `/scan` message are the first-run checks. A successful image build alone is not enough.

## Next time

Open a desktop terminal, go to your clone, and run:

```bash
cd ~/AutoNav_Mission_2026
bash tools/prepare_gui_env.sh
sudo docker compose --env-file .env.autonav up -d --force-recreate jazzy
sudo docker compose --env-file .env.autonav exec jazzy bash -lc '. /opt/ros/jazzy/setup.bash && . install/setup.bash && exec ros2 launch maya_bringup maya.launch.xml'
```

Run the preflight again after logging out or rebooting: `/run/user/$UID` is temporary, so the host X11 authorization file disappears and the display cookie can change. `exec` only works after the `jazzy` service is running. Recreating the container refreshes its display/GPU mounts; your bind-mounted build stays intact. You do **not** have to rebuild the Docker image or ROS workspace each day.

On Fedora, from the repository root (for Ro: `~/Documents/autonav/AutoNav_Mission_2026`), use the Fedora override on both Compose commands:

```bash
bash tools/prepare_gui_env.sh  # run as your graphical desktop user, without sudo
sudo docker compose -f compose.yaml -f compose.fedora.yaml --env-file .env.autonav up -d --force-recreate jazzy
sudo docker compose -f compose.yaml -f compose.fedora.yaml --env-file .env.autonav exec jazzy bash -lc '. /opt/ros/jazzy/setup.bash && . install/setup.bash && exec ros2 launch maya_bringup maya.launch.xml'
```

## Fedora 44 first run (Ubuntu 24.04 runs inside Docker)

Install Docker Engine and its Compose plugin on the host first, then install Fedora's host X11-cookie and LFS tools if missing: `sudo dnf install git-lfs xorg-x11-xauth`. Do **not** install ROS 2 on Fedora for this workflow. In a graphical desktop terminal after cloning `develop` and running `git lfs pull`:

```bash
cd ~/Documents/autonav/AutoNav_Mission_2026
bash tools/prepare_gui_env.sh  # no sudo; expect "Host preflight passed"
sudo docker compose -f compose.yaml -f compose.fedora.yaml --env-file .env.autonav build jazzy
sudo docker compose -f compose.yaml -f compose.fedora.yaml --env-file .env.autonav up -d jazzy
sudo docker compose -f compose.yaml -f compose.fedora.yaml --env-file .env.autonav exec jazzy bash -lc '. /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
sudo docker compose -f compose.yaml -f compose.fedora.yaml --env-file .env.autonav exec jazzy bash -lc '. /opt/ros/jazzy/setup.bash && . install/setup.bash && exec ros2 launch maya_bringup maya.launch.xml'
```

The last command stays running. In a second host terminal, from the same repository root, verify the simulated lidar:

```bash
sudo docker compose -f compose.yaml -f compose.fedora.yaml --env-file .env.autonav exec jazzy bash -lc '. /opt/ros/jazzy/setup.bash && . install/setup.bash && ros2 topic echo --once /scan'
```

Stop the launch with Ctrl-C. After reboot use the shorter Fedora sequence in "Next time," not `exec` before `up`. The GUI launch on Fedora was reported working; a clean-clone Ubuntu host run remains unverified.

## Keep working on the code

The repo on Ubuntu and `/workspace/AutoNav_Mission_2026` inside the container are the **same files**. Edit with your normal host editor, then build and test inside the container:

```bash
sudo docker compose --env-file .env.autonav exec jazzy bash -lc '. /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
```

For an interactive ROS shell inside the container:

```bash
sudo docker compose --env-file .env.autonav exec jazzy bash
cd /workspace/AutoNav_Mission_2026
. /opt/ros/jazzy/setup.bash
. install/setup.bash
```

Each new shell needs the `.` commands. Use your team's Git branch/PR process for code changes. If a change adds a dependency to `package.xml` or changes the Dockerfile, rebuild the image and recreate the container before rebuilding the workspace:

```bash
sudo docker compose --env-file .env.autonav build jazzy
sudo docker compose --env-file .env.autonav up -d --force-recreate jazzy
sudo docker compose --env-file .env.autonav exec jazzy bash -lc '. /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
```

Stop the launch with Ctrl-C, then stop/remove the idle container with `sudo docker compose --env-file .env.autonav down`. The image and your source/build files remain. If you simply leave the container running, the next `up -d` reuses it.

## If a step fails

- No `Host preflight passed`: run the script from a graphical desktop terminal, not with sudo or via SSH. It names the missing display, cookie, GPU, or LFS mesh. NVIDIA needs a separate GPU setup.
- Clone says `Permission denied (publickey)`: set up your own GitHub SSH access on Ubuntu; do not copy private keys into the container.
- Mesh is an LFS pointer: run `git lfs pull` in the host clone, then rerun preflight.
- `ros2: command not found` or `maya_bringup` missing: source ROS and the built workspace **in the same shell**, as in the commands above.
- `service "jazzy" is not running` after reboot: `exec` cannot start a stopped container. Run the "Next time" preflight and `up -d --force-recreate jazzy` first, then launch.
- `error mounting ".../autonav-docker.xauth" ... not a directory`: Docker may have created a **directory** at the missing bind-mount source after reboot. Check with `stat -Lc '%F %U:%G %n' "$XDG_RUNTIME_DIR/autonav-docker.xauth"`. If it says `directory`, run `rmdir "$XDG_RUNTIME_DIR/autonav-docker.xauth"` (this removes it only if empty), then run `bash tools/prepare_gui_env.sh` **without sudo**. Check that `stat` now says `regular file`; retry the `up -d --force-recreate jazzy` command for your OS, then `exec`. If the directory is not empty, inspect its contents before removing anything. Do not remove the path if it is already a regular file, and do not make the cookie world-readable or use `xhost +`.
- No windows, blank map, or graphics errors: save the *first* Qt/OpenGL/Gazebo error and your GPU model. Do not fix it with `xhost +` (that disables display access control).

## Maintainer notes

On Fedora, add `-f compose.yaml -f compose.fedora.yaml` immediately after `docker compose` in each command above. The Fedora override disables SELinux labeling for this container; do not use it on Ubuntu.

Ro tested the image build and no-argument GUI launch on Fedora on October 4, 2026. Both packages built incrementally, he reported the GUI working, and the log shows a navigation goal succeeding after initial planning retries. RViz logged a GLSL shader-link error, so check whether maps render correctly on each machine. A **fresh Ubuntu 24.04 clone has not yet been tested**; do that before calling the guide validated for the team.
