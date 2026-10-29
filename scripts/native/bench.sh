#!/usr/bin/env bash
# Run one sim spec in the container and natively, and compare them.
#
#   scripts/native/bench.sh [robot] [scenario] [controller]    # openarm empty idle
#
# Each runtime runs the same MuJoCo launch the sim agent runs, with
# scripts/native/bench_probe.py beside it in the same ROS graph. The probe reports
# time-to-ready, real-time factor and each camera's compressed frame rate, and
# bench_verdict.py prints the gate: exit 0 GO, 3 NO-GO; any other exit is no verdict
# (2 keeps the logs and prints their path).
# Needs Docker with BENCH_IMAGE already pulled, and a native build (pixi run build).
#
# Env: BENCH_IMAGE (the controllers image), BENCH_CAMERAS (cameras the robot has,
# 2 for openarm), BENCH_WINDOW_S (30), BENCH_TIMEOUT_S (240), ROS_DOMAIN_ID (77,
# kept off any real robot's domain).
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
repo="$(cd "$here/../.." && pwd)"

robot="${1:-openarm}"
scenario="${2:-empty}"
controller="${3:-idle}"
image="${BENCH_IMAGE:-ghcr.io/first-motive/fm-sim-controllers:jazzy}"
cameras="${BENCH_CAMERAS:-2}"
window_s="${BENCH_WINDOW_S:-30}"
timeout_s="${BENCH_TIMEOUT_S:-240}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

launch=(ros2 launch fm_sim_bringup run.launch.py backend:=mujoco
        "robot:=$robot" "scenario:=$scenario" "controller:=$controller" use_sim:=true)
probe_args=("$cameras" "$window_s" "$timeout_s")
out="$(mktemp -d)"
container="fm-sim-bench-$$"
native_pgid=""

cleanup() {
  docker rm -f "$container" >/dev/null 2>&1 || true
  if [ -n "$native_pgid" ] && kill -0 -- "-$native_pgid" 2>/dev/null; then
    kill -INT -- "-$native_pgid" 2>/dev/null || true
    for _ in $(seq 20); do kill -0 -- "-$native_pgid" 2>/dev/null || break; sleep 1; done
    kill -KILL -- "-$native_pgid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "container: $image" >&2
# Confined as the agent confines a run (agent/src/fm_sim_agent/runner.py).
docker run --detach --rm --pull=never --name "$container" --memory 16g --pids-limit 4096 \
  --cap-drop ALL --security-opt no-new-privileges \
  --env ROS_DOMAIN_ID --env RMW_IMPLEMENTATION --env ROS_AUTOMATIC_DISCOVERY_RANGE \
  "$image" /ros_entrypoint.sh "${launch[@]}" >/dev/null
docker exec -i --env ROS_DOMAIN_ID --env RMW_IMPLEMENTATION --env ROS_AUTOMATIC_DISCOVERY_RANGE \
  "$container" /ros_entrypoint.sh python3 - "${probe_args[@]}" \
  < "$here/bench_probe.py" > "$out/container.json" || true
# Already gone when the launch died at once: --rm removed it.
docker rm -f "$container" >/dev/null 2>&1 || true

echo "native: ${FM_SIM_NATIVE_HOME:-$HOME/.local/share/fm-sim/native}" >&2
# A job of its own (set -m), so the whole launch tree stops as one process group.
set -m
pixi run --manifest-path "$repo/pixi.toml" bash "$here/env.sh" "${launch[@]}" \
  > "$out/native.log" 2>&1 &
native_pgid=$!
set +m
pixi run --manifest-path "$repo/pixi.toml" bash "$here/env.sh" \
  python3 "$here/bench_probe.py" "${probe_args[@]}" > "$out/native.json" || true
cleanup
native_pgid=""

status=0
pixi run --manifest-path "$repo/pixi.toml" python3 "$here/bench_verdict.py" \
  "$out/container.json" "$out/native.json" || status=$?
[ "$status" -ne 2 ] || echo "logs: $out (native.log, container.json, native.json)" >&2
[ "$status" -eq 2 ] || rm -rf "$out"
exit "$status"
