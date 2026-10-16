# fm-sim image — the sim layer, FROM the shared fm-docker Jazzy base.
#
# Adds the ros2_control stack, the simulation engines and their ros2_control
# bridges, so this image can step a robot in MuJoCo or Gazebo, or bridge one to
# Isaac Sim. fm-robot publishes only a Humble image, so this layer starts from
# fm-docker directly and installs the control packages that the fm-robot layer
# used to carry. The robot descriptions stay in fm-robot; this image installs no
# fm-robot package (the Humble fm-robot image installed none either).
# The entrypoint, WORKDIR, and the viz/xacro tooling are inherited from the base.
FROM ghcr.io/first-motive/fm-docker:jazzy

ARG DEBIAN_FRONTEND=noninteractive

# ros2_control + the controllers fm-robot's descriptions are written against, the
# MuJoCo and Gazebo Harmonic bridges, and the headless GL stack MuJoCo's GLFW
# viewer needs under a virtual display (xvfb). Every package here ships arm64 and
# amd64 on the Jazzy (noble) mirror, so the image needs no per-arch split and no
# extra Gazebo apt repo; Harmonic comes from the ros.org gz vendor packages.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-joint-trajectory-controller \
      ros-jazzy-mujoco-ros2-control \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-ros-gz-bridge \
      xvfb \
      libgl1-mesa-dri \
      libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

# The MuJoCo Python binding — fm_sim_core's stepper loads MJCF models through it,
# pinned in requirements-sim.txt (shared with CI). Noble marks the system Python
# externally managed (PEP 668), and ROS runs on that interpreter, so the override
# is what puts the wheel where rclpy nodes import it.
COPY requirements-sim.txt /tmp/requirements-sim.txt
RUN PIP_BREAK_SYSTEM_PACKAGES=1 pip3 install --no-cache-dir -r /tmp/requirements-sim.txt

# --- Bake the workspace ------------------------------------------------------
# Copy the fm-sim package sources and colcon build them into the image, so the
# published image carries a built /ws/install overlay the inherited entrypoint
# sources. `curl run.sh | bash` then reaches a running sim with no clone and no
# runtime build. A host mount at /ws (run.sh's dev loop) shadows this baked
# overlay entirely, so mounting source overrides the baked build for the
# edit-rebuild loop — baked for the demo, mounted for development.
#
# topic_based_ros2_control (the Isaac backend's hardware plugin) has no Jazzy apt
# release, so it is built from source at the commit fm-sim.repos pins. The commit
# is read from the manifest and must be a full 40-hex sha before git sees it. The
# clone URL is duplicated from the manifest, so change both together. The MJCF
# model externals are file-only sources and the default sim_loop demo falls back
# to a 1-DOF model, so they are left out to keep the image lean — mount them for
# full bringup.
COPY . /ws/src/fm-sim
RUN ref="$(awk '/external\/topic_based_ros2_control:/{found=1} found && /version:/{print $2; exit}' \
        /ws/src/fm-sim/fm-sim.repos)" \
    && { printf '%s\n' "$ref" | grep -Eqx '[0-9a-f]{40}' \
         || { echo "fm-sim.repos: bad topic_based_ros2_control version '$ref'" >&2; exit 1; }; } \
    && git clone --quiet https://github.com/PickNikRobotics/topic_based_ros2_control.git \
         /ws/src/external/topic_based_ros2_control \
    && git -C /ws/src/external/topic_based_ros2_control checkout --quiet "$ref" -- \
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
