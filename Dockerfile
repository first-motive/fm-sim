# fm-sim image — the sim layer, FROM the fm-robot robot layer.
#
# Adds the simulation engines and their ros2_control bridges on top of the robot
# layer's controllers, so this image can step a robot in MuJoCo or Gazebo as well
# as drive it. The robot layer is itself FROM the shared fm-docker base, so the
# viz/description tooling and the controller stack are inherited here, not rebuilt.
# The entrypoint, WORKDIR, and the ros2_control deps are inherited — this layer
# only adds the sim apt packages and the MuJoCo Python binding.
FROM ghcr.io/first-motive/fm-robot:humble

ARG DEBIAN_FRONTEND=noninteractive

# packages.ros.org currently ships libignition-gazebo6 6.18.0 (which needs
# libignition-sensors6 >= 6.8.1) but only libignition-sensors6 6.8.0, so the gz
# stack will not resolve from ros.org alone. Add the OSRF Gazebo repo, which
# carries the matching 6.8.1 sensors libs. Drop this once ros.org is consistent.
RUN apt-get update && apt-get install -y --no-install-recommends \
      wget gnupg lsb-release \
    && wget -qO /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
         https://packages.osrfoundation.org/gazebo.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# Sim engines + their ros2_control bridges, plus the headless GL stack MuJoCo's
# GLFW viewer needs under a virtual display (xvfb). All on the Humble apt mirror
# (gz libs via the OSRF repo above) for both arm64 and amd64, so no source builds.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-mujoco-ros2-control \
      ros-humble-gz-ros2-control \
      ros-humble-ros-gz-sim \
      ros-humble-ros-gz-bridge \
      xvfb \
      libgl1-mesa-dri \
      libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# The MuJoCo Python binding — fm_sim_backends loads the MJCF models through it.
RUN pip install --no-cache-dir mujoco

# --- Bake the workspace ------------------------------------------------------
# Copy the fm-sim package sources and colcon build them into the image, so the
# published image carries a built /ws/install overlay the inherited entrypoint
# sources. `curl run.sh | bash` then reaches a running sim with no clone and no
# runtime build. A host mount at /ws (run.sh's dev loop) shadows this baked
# overlay entirely, so mounting source overrides the baked build for the
# edit-rebuild loop — baked for the demo, mounted for development.
#
# The runtime deps (rclpy, sensor_msgs, launch, launch_ros) already ship on the
# base layers, so no rosdep step is needed here. The MJCF model externals are
# file-only sources (COLCON_IGNORE'd, never colcon packages) and the default
# sim_loop demo falls back to a 1-DOF model, so they are left out to keep the
# image lean — mount them for full backend bringup.
COPY . /ws/src/fm-sim
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build --symlink-install
