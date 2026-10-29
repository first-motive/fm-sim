#!/usr/bin/env bash
# Build the native Mac sim workspace: the packages the fm-sim-controllers image
# carries, at the same pins, built in this repo's pixi Jazzy environment.
#
#   pixi run --locked build          # every stage, from the fm-sim checkout
#   pixi run --locked build ctl      # one stage: ws or ctl
#   pixi run --locked build test     # the patched mujoco_ros2_control's own tests
#
# Two colcon workspaces under $FM_SIM_NATIVE_HOME (~/.local/share/fm-sim/native):
#
#   ws/      fm-sim at this checkout's HEAD, fm-sim.repos' externals, and
#            mujoco_ros2_control (source-built: RoboStack has none), as /ws in the image
#   ws_ctl/  the controllers overlay on top, as /ws_ctl in the controllers image:
#            fm-skills at FM_SKILLS_SHA, anvil-embodied-ai and fm-teleop at the SHAs
#            that fm-skills commit's Dockerfile pins. fm_skills_perception is left out
#            (see stage_ctl).
#
# A ws build records ws/install/.fm-ws-pins, the hash of what it built from (this
# checkout's HEAD, scripts/native, fm-sim.repos, pixi.lock). An overlay build then
# writes ws_ctl/install/.fm-pins over that record and the overlay SHAs, so the stamp
# names what was built, even when only one stage ran. env.sh runs nothing until the
# stamp exists, and a build removes it first, so a failed rebuild never passes for a
# finished one. See scripts/native/README.md.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
# shellcheck source=lib.sh
. "$here/lib.sh"

: "${CONDA_PREFIX:?run inside the pixi env: pixi run build}"
[ "$(uname -s)" = Darwin ] || { echo "native build: macOS only; Linux runs the container" >&2; exit 1; }

home="${FM_SIM_NATIVE_HOME:-$HOME/.local/share/fm-sim/native}"
[[ "$home" = /?* && "$home" != "$HOME" ]] || {
  echo "FM_SIM_NATIVE_HOME must be an absolute directory other than \$HOME: $home" >&2
  exit 1
}
ws="$home/ws"
ws_ctl="$home/ws_ctl"
sources="$home/sources"
stamp="$ws_ctl/install/.fm-pins"
ws_pins="$ws/install/.fm-ws-pins"
ws_record="$ws/.fm-ws-inputs"
overlay="$sources/overlay-shas"

# The image's mujoco_ros2_control (ros-jazzy-mujoco-ros2-control 0.1.2), by commit.
MUJOCO_ROS2_CONTROL_SHA=178b1c39e5010185116bc89b4e94cbb25b825f61
# The fm-skills commit the pinned controllers image was built from. Moves with
# CONTROLLERS_IMAGE_DIGEST in agent/src/fm_sim_agent/service.py.
FM_SKILLS_SHA=ecdb02e5013ded59ad70596b6c9eed9d378472c6
# Packages the controllers Dockerfile takes from each public repo.
ANVIL_PACKAGES=(anvil_robot_profiles classical_control_msgs classical_control)
TELEOP_PACKAGES=(fm_teleop_core fm_teleop_policy)

# Externals the image builds; every other external is file-only. The Isaac backend's
# topic_based_ros2_control stays ignored: Isaac does not run natively.
BUILT_EXTERNALS=(fm-robot openarm_description unitree_ros2)

# CMake's FindPython cannot locate the conda interpreter's dev component on its own.
# -dead_strip_dylibs drops linked dylibs a binary never uses, as RoboStack builds do:
# without it a C++ node links (and load-crashes on) rosidl Python-extension dylibs.
# shellcheck disable=SC2054  # the commas are -Wl, linker flags
COLCON_ARGS=(
  --event-handlers console_cohesion- console_direct-
  --cmake-args
  -DBUILD_TESTING=OFF
  -DPython_EXECUTABLE="$CONDA_PREFIX/bin/python"
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"
  -DCMAKE_SHARED_LINKER_FLAGS=-Wl,-dead_strip_dylibs
  -DCMAKE_EXE_LINKER_FLAGS=-Wl,-dead_strip_dylibs
)
# setuptools warns once per package about the deprecated tests_require.
export PYTHONWARNINGS="ignore:Unknown distribution option"

# stage_ws: fm-sim as committed, its externals, and patched mujoco_ros2_control.
stage_ws() {
  if [ -n "$(git -C "$repo" status --porcelain)" ]; then
    echo "native build: uncommitted changes are not built; the workspace takes HEAD" >&2
  fi
  # The record of what ws is built from is taken here, with the sources, not after
  # colcon: a commit made while colcon runs must not land in it.
  snapshot_ws "$repo" "$ws/src/fm-sim" "$ws_record"
  mkdir -p "$ws/external"
  # The agent is a uv project, not a ROS package.
  touch "$ws/src/fm-sim/agent/COLCON_IGNORE"

  check_pinned "$repo/fm-sim.repos"
  (cd "$ws" && vcs import --shallow --force < "$repo/fm-sim.repos" >/dev/null)
  local dir name
  for dir in "$ws"/external/*/; do touch "${dir}COLCON_IGNORE"; done
  for name in "${BUILT_EXTERNALS[@]}"; do rm -f "$ws/external/$name/COLCON_IGNORE"; done
  touch "$ws/external/unitree_ros2/example/src/COLCON_IGNORE"

  local mrc="$ws/external/mujoco_ros2_control"
  fetch_at https://github.com/ros-controls/mujoco_ros2_control.git "$MUJOCO_ROS2_CONTROL_SHA" "$mrc"
  git -C "$mrc" apply "$here/patches/mujoco-ros2-control-macos.patch"
  # The image installs the three runtime packages; upstream's demos and tests stay out.
  touch "$mrc/mujoco_ros2_control_demos/COLCON_IGNORE" "$mrc/mujoco_ros2_control_tests/COLCON_IGNORE"
  rm -rf "$ws/external/mujoco_vendor"
  cp -R "$here/shims/mujoco_vendor" "$ws/external/mujoco_vendor"
}

build_ws() {
  rm -f "$ws_pins"
  (cd "$ws" && colcon build --base-paths src external "${COLCON_ARGS[@]}")
  cp "$ws_record" "$ws_pins"
}

# test_patch: build the patched packages with their tests in a separate tree and run
# them, so the patch's macOS paths (CGL, rpaths, link flags) are exercised.
test_patch() {
  (
    cd "$ws"
    # Cleared before, not after, so a failing run's logs stay to read.
    rm -rf build_test install_test
    colcon build --packages-up-to mujoco_ros2_control_plugins mujoco_ros2_control \
      --build-base build_test --install-base install_test \
      "${COLCON_ARGS[@]/-DBUILD_TESTING=OFF/-DBUILD_TESTING=ON}"
    colcon test --packages-select mujoco_ros2_control_plugins \
      --build-base build_test --install-base install_test --event-handlers console_cohesion-
    colcon test-result --test-result-base build_test/mujoco_ros2_control_plugins
  )
}

# stage_ctl: the overlay's sources at the pinned controllers image's SHAs.
stage_ctl() {
  local skills="$sources/fm-skills" anvil_sha teleop_sha pkg
  fetch_at https://github.com/first-motive/fm-skills.git "$FM_SKILLS_SHA" "$skills"
  read -r anvil_sha teleop_sha < <(overlay_shas "$skills/docker/sim-controllers.Dockerfile")
  fetch_at https://github.com/first-motive/anvil-embodied-ai.git "$anvil_sha" "$sources/anvil-embodied-ai"
  fetch_at https://github.com/first-motive/fm-teleop.git "$teleop_sha" "$sources/fm-teleop"

  rm -rf "$ws_ctl/src"
  mkdir -p "$ws_ctl/src/anvil-embodied-ai" "$ws_ctl/src/fm-teleop"
  cp -R "$skills/src" "$ws_ctl/src/fm-skills"
  for pkg in "${ANVIL_PACKAGES[@]}"; do
    cp -R "$sources/anvil-embodied-ai/ros2/src/$pkg" "$ws_ctl/src/anvil-embodied-ai/"
  done
  for pkg in "${TELEOP_PACKAGES[@]}"; do
    cp -R "$sources/fm-teleop/$pkg" "$ws_ctl/src/fm-teleop/"
  done
  # fm_skills_perception's core links OpenCV 4's calib3d, which RoboStack's OpenCV 5
  # split up. The image builds the core only (no OpenVINO, so no node), and the sim
  # runs ground-truth perception, so leaving it out changes nothing that runs.
  touch "$ws_ctl/src/fm-skills/fm_skills_perception/COLCON_IGNORE"
  echo "$FM_SKILLS_SHA $anvil_sha $teleop_sha" > "$overlay"
}

build_ctl() {
  # colcon's setup scripts read unset variables.
  set +u
  # shellcheck disable=SC1091
  . "$ws/install/setup.bash"
  set -u
  (cd "$ws_ctl" && colcon build --base-paths src "${COLCON_ARGS[@]}")
}

write_stamp() {
  [ -f "$ws_pins" ] || { echo "native build: ws is not built; run build.sh ws first" >&2; exit 1; }
  pins_stamp "$ws_pins" "$overlay" > "$stamp"
  echo "native workspace built: $home (pins $(cat "$stamp"))"
  if [ "$(cat "$ws_pins")" != "$(ws_stamp "$repo" "$(git -C "$repo" rev-parse HEAD)")" ]; then
    echo "native build: ws was built from other inputs than this checkout's; run build.sh ws" >&2
  fi
}

mkdir -p "$ws" "$ws_ctl" "$sources"
# Every build stage invalidates the stamp; the test stage builds in its own tree.
# A ws build leaves the overlay built against the old ws, so only a build that ends
# with the overlay writes the stamp; ws records its own inputs for it.
[ "${1:-all}" = test ] || rm -f "$stamp"
case "${1:-all}" in
  ws) stage_ws; build_ws ;;
  ctl) stage_ctl; build_ctl; write_stamp ;;
  test) stage_ws; test_patch ;;
  all) stage_ws; build_ws; stage_ctl; build_ctl; write_stamp ;;
  *) echo "usage: build.sh [all|ws|ctl|test]" >&2; exit 2 ;;
esac
