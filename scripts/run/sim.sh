#!/usr/bin/env bash
#
# sim — drive a First Motive sim host.
#
#   fm sim catalog --host fm-ws-01
#   fm sim up      --host fm-ws-01 --scenario can_on_table --controller fm_skills_gt --seed 3
#   fm sim status  --host fm-ws-01
#   fm sim logs    --host fm-ws-01
#   fm sim down    --host fm-ws-01
#   fm sim runs    --host fm-ws-01
#   fm sim trial   --host fm-ws-01 --scenario can_on_table --controller fm_skills_gt --seeds 0..19
#   fm sim up      --host local                  this Mac, in the foreground, on OrbStack
#   fm sim host start|stop|status aws --stack fm-sim-cloud-host   the AWS sim host
#   fm sim host status|pull|start|stop|restart local [--controllers]   this Mac's image and managed agent
#
# fm-tools forwards every argument verbatim; the work lives in
# fm_sim_agent.client (agent/), so the CLI and the agent share one verb set and
# one RunSpec check. `--host local` runs that agent in-process.
#
# Exit codes: 0 success, 1 the host refused, 2 usage, 3 precondition
# (no router endpoint, or the host did not answer; for `host`, AWS could not
# be asked), 130 interrupted.

set -euo pipefail

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FM_ROOT="$(cd "$_here/../.." && pwd)"

exec uv run --quiet --locked --project "$FM_ROOT/agent" fm-sim "$@"
