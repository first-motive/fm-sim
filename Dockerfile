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
# MuJoCo and Gazebo Harmonic bridges (with the JPEG image_transport plugin the
# camera relays compress with), and the headless GL stack MuJoCo's GLFW
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
      ros-jazzy-ros-gz-interfaces \
      ros-jazzy-compressed-image-transport \
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
# Import the manifest's sources and build them with fm-sim into the image, so the
# published image carries a built /ws/install overlay the inherited entrypoint
# sources. `curl run.sh | bash` then reaches a running sim with no clone and no
# runtime build. A host mount at /ws (run.sh's dev loop) shadows this baked
# overlay entirely, so mounting source overrides the baked build for the
# edit-rebuild loop — baked for the demo, mounted for development.
#
# fm-sim.repos lands at /ws/external, where fm_sim_models looks for the MJCF
# models. The sources built here are the ones with no Jazzy apt release:
# topic_based_ros2_control (the Isaac backend's plugin) and fm-robot (the robot
# descriptions run.launch.py expands) with the two externals it builds against.
# Every other external is file-only and COLCON_IGNORE'd. CI's build-externals
# input names the same list. The import and rosdep sit before the source copy so
# an fm-sim edit does not refetch them; jsonschema (the sim catalog's validator)
# rides this layer so the engine layer above stays cached.
COPY fm-sim.repos /ws/fm-sim.repos
# Every source must be pinned to a full commit sha; a tag or branch can move.
RUN cd /ws \
    && bad="$(awk '/^ *version:/{print $2}' fm-sim.repos | grep -Evx '[0-9a-f]{40}' || true)" \
    && { [ -z "$bad" ] || { echo "fm-sim.repos: unpinned version(s): $bad" >&2; exit 1; }; } \
    && vcs import < fm-sim.repos \
    && for d in external/*/; do touch "${d}COLCON_IGNORE"; done \
    && for name in topic_based_ros2_control fm-robot openarm_description unitree_ros2; do \
         rm "external/$name/COLCON_IGNORE"; \
       done \
    && touch external/unitree_ros2/example/src/COLCON_IGNORE \
    && apt-get update \
    && apt-get install -y --no-install-recommends python3-jsonschema \
    && rosdep update --rosdistro "$ROS_DISTRO" \
    && rosdep install --from-paths external --ignore-src -y -r \
    && rm -rf /var/lib/apt/lists/*

COPY . /ws/src/fm-sim
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
