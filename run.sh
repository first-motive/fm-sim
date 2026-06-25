#!/usr/bin/env bash
# Standalone front door for fm-sim. Builds the workspace and launches the headless
# sim loop — Foxglove Studio renders it at ws://localhost:8765.
#
# Curl-able (no clone needed) on macOS:
#   curl -fsSL https://raw.githubusercontent.com/first-motive/fm-sim/main/run.sh | bash
#
# From a clone:
#   ./run.sh [--backend NAME] [--native|--container] [params_file:=my.yaml]
#
# The host OS picks the path (override with --native / --container):
#   linux  -> native:    build + launch directly on the host (ROS2 Humble + the
#                        sim deps must be installed)
#   macos  -> container: run the fm-sim image via the fm-docker compose overlays,
#                        build + launch inside it (OrbStack)
#
# Piped via curl, the shared host checks (fm-docker's scripts/lib.sh) and the
# compose overlays are fetched from fm-docker and cached under ~/.cache/fm-sim,
# so later runs work offline.
#
# --backend names the sim engine (the standalone launch here is the sim loop; the
# per-robot, per-backend orchestration lives in fm-app's fm_bringup). On the
# container path every backend runs under the one macOS overlay — mock, mujoco,
# and gazebo all come up headless in the Mac container (gazebo server-only via
# `gz -s` under software GL). Linux runs bare-metal native, so it takes no overlay
# at all. isaac is documented out: its Sim app is Linux + NVIDIA only, never on
# macOS or the container base image (see README / ARCHITECTURE).
#
#   ./run.sh                       # auto-detect path (mujoco, the Mac default)
#   ./run.sh --backend gazebo      # gazebo, headless in the Mac container
#   ./run.sh --native              # force the host path (Linux)
#   ./run.sh --container           # force the container path (macOS / OrbStack)
#   ./run.sh params_file:=my.yaml  # extra args pass through to ros2 launch
set -euo pipefail

# --- Per-repo config (downstream repos retune these) --------------------------
LOCAL_IMAGE=fm-sim:humble                          # locally-built tag for the clone dev loop
BAKED_IMAGE=ghcr.io/first-motive/fm-sim:humble     # published image for the no-clone baked path
LAUNCH=(ros2 launch fm_sim_core sim.launch.py)     # what `run.sh` launches
FM_SIM_RAW="https://raw.githubusercontent.com/first-motive/fm-sim/main"
FM_DOCKER_RAW="https://raw.githubusercontent.com/first-motive/fm-docker/main"
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/fm-sim"
# -----------------------------------------------------------------------------

# Keep the caller's directory: it is the workspace for the native build and the
# mount (FM_WS) for the container.
INVOKE_DIR="$PWD"

# Resolve the script's own dir; empty when piped via curl|bash. A clone has the
# repo files next to the script (REPO_DIR set); a piped run does not (REPO_DIR
# empty), so deps are fetched from the raw URLs instead.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-}")" 2>/dev/null && pwd)" || SCRIPT_DIR=""
if [ -n "$SCRIPT_DIR" ] && [ -f "$SCRIPT_DIR/fm-sim.repos" ]; then
  REPO_DIR="$SCRIPT_DIR"
else
  REPO_DIR=""
fi

# Load the shared host checks (fm-docker's scripts/lib.sh). docker/ is vcs-imported
# into a clone, so prefer the imported copy; otherwise reuse a cached fetch, else
# fetch from fm-docker and cache it. The checks must run in this shell, so source
# rather than execute.
load_lib() {
  local imported="${REPO_DIR}/docker/scripts/lib.sh"
  if [ -n "$REPO_DIR" ] && [ -f "$imported" ]; then
    # shellcheck source=/dev/null
    source "$imported"
    return
  fi
  local cached="$CACHE_DIR/lib.sh"
  if [ ! -f "$cached" ]; then
    mkdir -p "$CACHE_DIR"
    # Fetch to a temp file and rename only on success: an interrupted download
    # must never leave a partial file later runs treat as cached.
    local tmp="$cached.tmp.$$"
    curl -fsSL "$FM_DOCKER_RAW/scripts/lib.sh" -o "$tmp" \
      || { rm -f "$tmp"; echo "error: failed to fetch lib.sh from fm-docker" >&2; exit 1; }
    [ -s "$tmp" ] || { rm -f "$tmp"; echo "error: empty lib.sh download" >&2; exit 1; }
    mv "$tmp" "$cached"
  fi
  # shellcheck source=/dev/null
  source "$cached"
}
load_lib

# The backend names the sim engine. sim.launch.py dispatches it (mujoco runs the
# in-process sim_loop; gazebo/isaac delegate to fm_sim_backends). mock has no
# sim.launch backend — it is the fm_bringup no-engine path — so it falls through
# to the mujoco default below.
VALID_BACKENDS=(mock mujoco gazebo isaac)
# The interactive picker offers only the engines sim.launch.py dispatches.
PICK_BACKENDS=(mujoco gazebo isaac)

BACKEND=mujoco
BACKEND_SET=false        # true once --backend is given; gates the picker
MODE=""                  # "" = auto-detect; else native | container
PASSTHROUGH=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend)   [[ $# -gt 1 ]] || { echo "error: --backend requires a value" >&2; exit 1; }
                 BACKEND="$2"; BACKEND_SET=true; shift 2 ;;
    --backend=*) BACKEND="${1#--backend=}"; BACKEND_SET=true; shift ;;
    --native)    MODE=native; shift ;;
    --container) MODE=container; shift ;;
    *)           PASSTHROUGH+=("$1"); shift ;;
  esac
done

# No --backend and a real terminal: offer a menu. Skips on --backend (explicit
# choice) and on `curl | bash` (no TTY), so the curl-to-launch path is untouched.
# Uses the fm-tools rich picker when uv can reach it; falls back to a plain bash
# `select` otherwise. The chosen value flows through the same validation below.
if [[ "$BACKEND_SET" != true ]] && [ -t 0 ] && [ -t 1 ]; then
  picked=""
  if command -v uv >/dev/null 2>&1; then
    picked="$(uv run --quiet --no-project \
      --with "fm-tools @ git+https://github.com/first-motive/fm-tools@5d9ef62f9449321730b8ebcacef7be3bc13448f5" \
      fm-pick "Select a sim backend" "${PICK_BACKENDS[@]}" 2>/dev/null)" || picked=""
  fi
  if [[ -z "$picked" ]]; then
    PS3="Select a sim backend: "
    select picked in "${PICK_BACKENDS[@]}"; do [[ -n "$picked" ]] && break; done
  fi
  [[ -n "$picked" ]] && BACKEND="$picked"
fi

# Normalize hyphen -> underscore and validate the backend.
BACKEND="${BACKEND//-/_}"
ok=false
for b in "${VALID_BACKENDS[@]}"; do [[ "$BACKEND" == "$b" ]] && ok=true && break; done
if [[ "$ok" != true ]]; then
  echo "error: unknown backend '$BACKEND' — valid: ${VALID_BACKENDS[*]}" >&2
  exit 1
fi

# Auto-detect the path from the host OS when not forced by a flag. detect_os
# (from lib.sh) echoes macos|linux.
if [[ -z "$MODE" ]]; then
  case "$(detect_os)" in
    linux)  MODE=native ;;
    macos)  MODE=container ;;
    *) echo "error: could not resolve host path — pass --native or --container" >&2; exit 1 ;;
  esac
fi

# The container path runs every backend under the one macOS overlay — compose.linux
# was removed (Linux runs bare-metal native, never in a container), so there is no
# per-backend overlay split anymore. The native path ignores this entirely.
OVERLAY=docker/compose.macos.yaml

# CI self-test hook: deps loaded, OS + backend resolved — stop before any runtime
# work. Lets the curl-path test exercise the piped fetch without OrbStack.
if [ -n "${FM_SELFTEST:-}" ]; then
  echo "selftest ok: lib loaded, mode=$MODE, backend=$BACKEND"
  exit 0
fi

# Forward the chosen engine to sim.launch.py's backend dispatch. mock has no
# sim.launch backend, so it falls through to the default (mujoco = sim_loop).
# Passthrough args (e.g. params_file:=my.yaml) reach the launch alongside it.
if [[ "$BACKEND" != mock ]]; then
  LAUNCH+=("backend:=$BACKEND")
fi
LAUNCH+=(${PASSTHROUGH[@]+"${PASSTHROUGH[@]}"})

if [[ "$MODE" == native ]]; then
  # Host path: pull externals once, build in place, launch on the host.
  set +u  # ROS setup scripts reference unbound vars; nounset would abort the source
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  set -u
  cd "$INVOKE_DIR"
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

# Container path (macOS / OrbStack). Bring up a runtime if none is present, then
# dispatch on clone vs pipe:
#   pipe (no source on disk) -> pull the baked image and run it with no mount, so
#                               the entrypoint sources the workspace baked in.
#   clone (source on disk)   -> mount source at /ws, rebuild inside, launch, so
#                               edits override the baked build (the dev loop).
cd "$INVOKE_DIR"

# Bring up a container runtime if missing — install + start OrbStack via install.sh.
if ! has_docker; then
  echo ">> no container runtime — setting up OrbStack via install.sh"
  if [ -n "$REPO_DIR" ]; then
    bash "$REPO_DIR/install.sh" --no-pull
  else
    curl -fsSL "$FM_SIM_RAW/install.sh" | bash -s -- --no-pull
  fi
  has_docker || { echo "error: container runtime still unavailable after setup." >&2; exit 1; }
fi

if [ -z "$REPO_DIR" ]; then
  # Baked path: curl-to-launch, no clone, no mount. The image carries a built
  # /ws/install overlay (see Dockerfile), so route through the entrypoint to
  # source ROS + that overlay, then launch. --pull missing fetches on first run;
  # arm64 matches the macOS overlay's platform pin. The mujoco launch wraps its
  # node in `xvfb-run -a` itself, so no outer virtual display is needed here.
  echo ">> running the baked image $BAKED_IMAGE (no clone, no mount)"
  echo ">> launching the sim loop — Foxglove Studio: ws://localhost:8765"
  exec docker run --rm --pull missing --platform linux/arm64 \
    -p 8765:8765 "$BAKED_IMAGE" /ros_entrypoint.sh "${LAUNCH[@]}"
fi

# Mounted path: build the local image, bring it up, build + launch inside it.
# The fm-docker compose overlays live in docker/, imported via fm-sim.repos —
# pull them on first run so a fresh clone works with no manual setup.
if [[ ! -d docker ]]; then
  vcs import < fm-sim.repos
fi
COMPOSE=(docker compose -f docker/compose.yaml -f "$OVERLAY")
export FM_IMAGE="$LOCAL_IMAGE"
export FM_WS="$INVOKE_DIR"

echo ">> building $LOCAL_IMAGE (FROM the fm-robot layer)"
docker build -t "$LOCAL_IMAGE" .
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
