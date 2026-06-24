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

# Sim engines + their ros2_control bridges, plus the headless GL stack MuJoCo's
# GLFW viewer needs under a virtual display (xvfb). All on the Humble apt mirror
# for both arm64 and amd64, so no source builds.
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
