# Ubuntu 24.04 + ROS 2 Jazzy. Source and build artifacts stay in the host bind mount.
FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       git git-lfs openssh-client python3-rosdep python3-colcon-common-extensions \
       ros-jazzy-xacro ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Resolve dependencies from the same package manifests used by the workspace.
# Only manifests enter the image; the large Git LFS meshes do not.
COPY src/maya_bringup/package.xml /opt/autonav-deps/src/maya_bringup/package.xml
COPY src/maya_description/package.xml /opt/autonav-deps/src/maya_description/package.xml
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update \
    && apt-get update \
    && rosdep install --from-paths /opt/autonav-deps/src --ignore-src --rosdistro jazzy -y \
    && rm -rf /var/lib/apt/lists/*

# maya.launch.xml appends the installed world's path but assumes this variable exists.
ENV GZ_SIM_RESOURCE_PATH=/workspace
WORKDIR /workspace/AutoNav_Mission_2026
