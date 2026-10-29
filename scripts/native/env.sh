#!/usr/bin/env bash
# Run a command in the native sim workspace, as /ros_entrypoint.sh does in the image:
# the built ws, then the controllers overlay, on the pixi Jazzy env.
#
#   pixi run --manifest-path <fm-sim>/pixi.toml bash scripts/native/env.sh ros2 launch ...
#
# Discovery stays on this host, as the agent's container runs do
# (agent/src/fm_sim_agent/runner.py DEFAULT_DDS_RANGE): a sim on a shared
# ROS_DOMAIN_ID must never reach a real robot, so no inherited env var widens it.
# No -u: colcon's setup scripts read unset variables.
set -eo pipefail

[ "$#" -gt 0 ] || { echo "usage: env.sh <command> [args...]" >&2; exit 2; }

: "${CONDA_PREFIX:?run inside the pixi env}"
home="${FM_SIM_NATIVE_HOME:-$HOME/.local/share/fm-sim/native}"
[ -f "$home/ws_ctl/install/.fm-pins" ] || {
  echo "native workspace not built at $home: run pixi run --locked build" >&2
  exit 1
}

# shellcheck disable=SC1091
. "$home/ws/install/setup.bash"
# shellcheck disable=SC1091
. "$home/ws_ctl/install/local_setup.bash"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# A Cyclone profile with peers would widen discovery the same way.
unset CYCLONEDDS_URI
exec "$@"
