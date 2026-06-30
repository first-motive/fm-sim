#!/usr/bin/env bash
# macOS smoke test for the fm-sim backends. Builds the sim image from the working
# tree, then runs each backend bounded inside the container (real arm64 + OrbStack)
# and prints a pass/skip/fail table. This is the hand-run truth on a Mac that the
# ubuntu container CI (amd64) cannot give — gazebo headless on arm64 under software
# GL has never been proven, so each backend is captured as PASS/SKIP/FAIL rather
# than assumed.
#
#   ./scripts/smoke.sh                 # build + smoke all backends
#   SMOKE_TIMEOUT=120 ./scripts/smoke.sh   # widen the per-backend window
#   SMOKE_PLATFORM=linux/amd64 ./scripts/smoke.sh   # override the arch pin
#
# Exit status is 0 when nothing FAILed (PASS/SKIP only), non-zero otherwise, so it
# is scriptable. Run from a clone — it builds the local Dockerfile.
#
# nounset on; NOT errexit — every backend must run so the table is complete even
# when one fails.
set -uo pipefail

IMAGE="fm-sim:humble"
TIMEOUT="${SMOKE_TIMEOUT:-90}"
PLATFORM="${SMOKE_PLATFORM:-linux/arm64}"   # macOS Apple silicon; override for amd64

# Resolve the repo root from this script's location (scripts/ lives at the root).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_DIR" || exit 1

if ! command -v docker >/dev/null 2>&1; then
  echo "error: docker not found — run ./install.sh first to set up OrbStack." >&2
  exit 1
fi

# The mujoco check boots the binding under a virtual display and steps one frame —
# proves the wheel loads and headless GL works without needing a full robot model.
MUJOCO_CHECK='import mujoco
m = mujoco.MjModel.from_xml_string("<mujoco><worldbody/></mujoco>")
mujoco.mj_step(m, mujoco.MjData(m))
print("mujoco stepped headless")'

# Per-backend bounded command, run through the entrypoint (sources ROS + overlay).
# Software GL (LIBGL_ALWAYS_SOFTWARE=1) keeps gazebo + mujoco off a real GPU.
backend_cmd() {
  case "$1" in
    mock)   echo 'ros2 launch fm_sim_core sim.launch.py' ;;
    # %q escapes the multi-line payload into one shell-safe token for `bash -lc`.
    mujoco) printf 'xvfb-run -a python3 -c %q' "$MUJOCO_CHECK" ;;
    # Server-only headless gazebo: no GUI, run a bounded number of iterations on
    # the empty world. `gz` (Garden+) or `ign` (Fortress) depending on the image.
    gazebo) echo 'if command -v gz >/dev/null 2>&1; then gz sim -s -r --iterations 200 empty.sdf; else ign gazebo -s -r --iterations 200 empty.sdf; fi' ;;
    *)      return 1 ;;
  esac
}

# Run one backend in the container and classify the outcome. Two shapes:
#   oneshot (mujoco): expect a clean exit 0 — the check boots, steps, returns.
#   long (mock, gazebo): came up if it ran the window. gazebo is bounded by
#     --iterations, so it exits 0 on success; mock runs until the timeout kills
#     it (124) or SIGTERM (143). Accept 0/124/143 for these; anything else FAILs.
RESULTS=()
run_backend() {
  local name="$1" oneshot="$2" cmd code
  cmd="$(backend_cmd "$name")"
  echo ">> [$name] running bounded ${TIMEOUT}s in the container ..."
  docker run --rm --platform "$PLATFORM" \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    "$IMAGE" /ros_entrypoint.sh bash -lc "timeout ${TIMEOUT} ${cmd}"
  code=$?
  if [ "$oneshot" = oneshot ]; then
    [ "$code" -eq 0 ] && RESULTS+=("$name PASS") || RESULTS+=("$name FAIL (exit $code)")
  else
    case "$code" in
      0|124|143) RESULTS+=("$name PASS") ;;
      *)         RESULTS+=("$name FAIL (exit $code)") ;;
    esac
  fi
}

echo ">> building $IMAGE from the working tree ..."
if ! docker build -t "$IMAGE" .; then
  echo "error: image build failed — need ghcr access to the fm-robot base FROM." >&2
  exit 1
fi

run_backend mock   long
run_backend mujoco oneshot
run_backend gazebo long
RESULTS+=("isaac SKIP (Linux + NVIDIA only — never on macOS or the container base)")

echo
echo "================ fm-sim smoke (${PLATFORM}) ================"
printf '%s\n' "${RESULTS[@]}"
echo "==========================================================="

# Non-zero exit if any backend FAILed, so the script is usable as a gate.
for r in "${RESULTS[@]}"; do
  case "$r" in *" FAIL"*) exit 1 ;; esac
done
exit 0
