#!/usr/bin/env bash
# Standalone front door for fm-sim. Builds the workspace and launches the headless
# sim loop — Foxglove Studio renders it at ws://localhost:8765.
#
# The host OS picks the path (override with --native / --container):
#   Linux  -> native:    build + launch directly on the host (ROS2 Humble + the
#                        sim deps must be installed)
#   Darwin -> container: build the fm-sim image, bring it up via the fm-docker
#                        compose overlays, build + launch inside it (OrbStack)
#
# --backend selects the host overlay (the standalone launch here is the sim loop;
# the per-robot, per-backend orchestration lives in fm-app's fm_bringup):
#   mock, mujoco   -> compose.macos   (Mac daily driver, CPU)
#   gazebo, isaac  -> compose.linux   (Linux/GPU)
# so --backend can override the uname auto-detect of the overlay (e.g. force the
# Linux/GPU overlay for gazebo on a Linux host).
#
#   ./run.sh                       # auto-detect path, macOS overlay (mujoco)
#   ./run.sh --backend gazebo      # force the Linux/GPU overlay
#   ./run.sh --native              # force the host path (Linux)
#   ./run.sh --container           # force the container path (macOS / OrbStack)
#   ./run.sh params_file:=my.yaml  # extra args pass through to ros2 launch
set -euo pipefail

cd "$(dirname "$0")"

# --- Per-repo config (downstream repos retune these two) ----------------------
IMAGE=fm-sim:humble                                # local image tag for the container path
LAUNCH=(ros2 launch fm_sim_core sim.launch.py)     # what `run.sh` launches
# -----------------------------------------------------------------------------

# The sim backend selects the host overlay only (the packaged sim.launch.py takes
# params_file, not a backend switch — that orchestration is fm-app's fm_bringup).
VALID_BACKENDS=(mock mujoco gazebo isaac)

BACKEND=mujoco
MODE=""                  # "" = auto-detect; else native | container
PASSTHROUGH=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend)   BACKEND="$2"; shift 2 ;;
    --backend=*) BACKEND="${1#--backend=}"; shift ;;
    --native)    MODE=native; shift ;;
    --container) MODE=container; shift ;;
    *)           PASSTHROUGH+=("$1"); shift ;;
  esac
done

# Normalize hyphen -> underscore and validate the backend.
BACKEND="${BACKEND//-/_}"
ok=false
for b in "${VALID_BACKENDS[@]}"; do [[ "$BACKEND" == "$b" ]] && ok=true && break; done
if [[ "$ok" != true ]]; then
  echo "error: unknown backend '$BACKEND' — valid: ${VALID_BACKENDS[*]}" >&2
  exit 1
fi

# Auto-detect the path from the host OS when not forced by a flag.
if [[ -z "$MODE" ]]; then
  case "$(uname -s)" in
    Linux)  MODE=native ;;
    Darwin) MODE=container ;;
    *) echo "error: unsupported host '$(uname -s)' — pass --native or --container" >&2; exit 1 ;;
  esac
fi

# The backend picks the compose overlay: CPU sim on macOS, GPU sim on Linux.
case "$BACKEND" in
  mock|mujoco)   OVERLAY=docker/compose.macos.yaml ;;
  gazebo|isaac)  OVERLAY=docker/compose.linux.yaml ;;
esac

# Only the passthrough args reach the launch — sim.launch.py declares params_file.
LAUNCH+=(${PASSTHROUGH[@]+"${PASSTHROUGH[@]}"})

if [[ "$MODE" == native ]]; then
  # Host path: pull externals once, build in place, launch on the host.
  set +u  # ROS setup scripts reference unbound vars; nounset would abort the source
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  set -u
  if [[ ! -d external ]]; then
    vcs import < fm-sim.repos
  fi
  rosdep install --from-paths . external --ignore-src -y -r 2>/dev/null || true
  colcon build --symlink-install
  set +u
  source install/setup.bash
  set -u
  echo ">> launching the sim loop on the host — Foxglove Studio: ws://localhost:8765"
  exec "${LAUNCH[@]}"
fi

# Container path: build the local image, bring it up, build + launch inside it.
# The fm-docker compose overlays live in docker/, imported via fm-sim.repos —
# pull them on first run so a fresh clone works with no manual setup.
if [[ ! -d docker ]]; then
  vcs import < fm-sim.repos
fi
COMPOSE=(docker compose -f docker/compose.yaml -f "$OVERLAY")
export FM_IMAGE="$IMAGE"
export FM_WS="$PWD"

echo ">> building $IMAGE (FROM the fm-robot layer)"
docker build -t "$IMAGE" .
echo ">> bringing the container up (idempotent) — overlay $(basename "$OVERLAY")"
"${COMPOSE[@]}" up -d
echo ">> building the workspace inside the container"
"${COMPOSE[@]}" exec fm /ros_entrypoint.sh colcon build --symlink-install
echo ">> launching the sim loop — Foxglove Studio: ws://localhost:8765"
echo ">> tear down with: ${COMPOSE[*]} down"
# The mujoco backend's launch already wraps its ros2_control_node in `xvfb-run -a`
# (see fm_sim_backends/launch/mujoco.launch.py), so the headless Mac container
# needs no outer xvfb-run here — the virtual display is scoped to the node that
# needs it. `exec` skips the image ENTRYPOINT, so route through it to source ROS.
exec "${COMPOSE[@]}" exec fm /ros_entrypoint.sh "${LAUNCH[@]}"
