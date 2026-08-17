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

# The mock check brings the sim loop up and asserts it reached a working state: the
# node registers, and /joint_states carries a real message. The loop's stepper
# imports the mujoco wheel lazily, so a node that builds clean can still die on its
# first import — the topic assert is what catches that. Self-bounded, so the caller
# adds no timeout of its own.
#
# CI runs the same two assertions from first-motive/.github's
# scripts/ros2-smoke-assert.sh. This stays a local copy on purpose: a hand-run macOS
# smoke should not need the network to tell you whether your sim came up.
MOCK_ASSERT='set -uo pipefail
ros2 launch fm_sim_core sim.launch.py >/tmp/sim.log 2>&1 &
pid=$!
for _ in $(seq 1 30); do
  ros2 node list 2>/dev/null | grep -qx /sim_loop && break
  sleep 2
done
fails=0
if ros2 node list 2>/dev/null | grep -qx /sim_loop; then
  echo "PASS: /sim_loop up"
else
  echo "FAIL: /sim_loop never appeared"; fails=1
fi
if timeout 15 ros2 topic echo --once /joint_states >/tmp/js.out 2>/dev/null && [ -s /tmp/js.out ]; then
  echo "PASS: /joint_states publishing"
else
  echo "FAIL: /joint_states silent"; fails=1
fi
kill $pid 2>/dev/null || true
[ $fails -eq 0 ] || { echo "== launch log =="; tail -40 /tmp/sim.log; exit 1; }'

# Per-backend command, run through the entrypoint (sources ROS + overlay). Software
# GL (LIBGL_ALWAYS_SOFTWARE=1) keeps gazebo + mujoco off a real GPU. Each command
# carries its own bound, and each one must exit 0 — no backend passes merely by
# outliving a timer.
backend_cmd() {
  case "$1" in
    # %q escapes a multi-line payload into one shell-safe token for `bash -lc`.
    mock)   printf 'bash -c %q' "$MOCK_ASSERT" ;;
    mujoco) printf 'timeout %s xvfb-run -a python3 -c %q' "$TIMEOUT" "$MUJOCO_CHECK" ;;
    # Server-only headless gazebo: no GUI, run a bounded number of iterations on
    # the empty world. `gz` (Garden+) or `ign` (Fortress) depending on the image.
    # --iterations makes it finite, so exit 0 is the assertion — a hang is a FAIL.
    gazebo) printf 'timeout %s sh -c %q' "$TIMEOUT" 'if command -v gz >/dev/null 2>&1; then gz sim -s -r --iterations 200 empty.sdf; else ign gazebo -s -r --iterations 200 empty.sdf; fi' ;;
    *)      return 1 ;;
  esac
}

# Run one backend in the container and classify the outcome. Every backend is
# judged the same way now: exit 0 is PASS, anything else FAILs. A bounded workload
# that hit its timeout (124) did not finish the work it was given, which is a
# failure, not a pass.
RESULTS=()
run_backend() {
  local name="$1" cmd code
  cmd="$(backend_cmd "$name")"
  echo ">> [$name] running in the container (bound ${TIMEOUT}s) ..."
  docker run --rm --platform "$PLATFORM" \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    "$IMAGE" /ros_entrypoint.sh bash -lc "$cmd"
  code=$?
  if [ "$code" -eq 0 ]; then
    RESULTS+=("$name PASS")
  else
    RESULTS+=("$name FAIL (exit $code)")
  fi
}

echo ">> building $IMAGE from the working tree ..."
if ! docker build -t "$IMAGE" .; then
  echo "error: image build failed — need ghcr access to the fm-robot base FROM." >&2
  exit 1
fi

run_backend mock
run_backend mujoco
run_backend gazebo
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
