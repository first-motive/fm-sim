# fm-sim

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

Simulation layer for First Motive's ROS2 stack. One launch runs a backend, a
robot, a scenario and a controller together; a sim agent runs that launch on a
sim host (this Mac, fm-ws-01 or AWS), and the `fm sim` CLI and the desktop app's
Simulation screen drive it over Zenoh. The headless dev loop, the backend hosts
and the MJCF model registry sit underneath.

Part of First Motive's ROS2 stack. Builds standalone here; assembled
with the other six package repos by
[`fm-ros2`](https://github.com/first-motive/fm-ros2).

## Packages

| Package | Build | Role |
|---------|-------|------|
| `fm_sim_core` | ament_python | Headless dev loop, launch, and shared sim config |
| `fm_sim_backends` | ament_python | Backend hosts (MuJoCo and others) behind one interface |
| `fm_sim_models` | ament_python | MJCF model registry |
| `fm_sim_ee_follower` | ament_python | DLS IK node that follows `/commanded_ee_right` in place of the loader's IK |
| `fm_sim_bringup` | ament_python | `run.launch.py` (backend × robot × scenario × controller) and the ament-index catalog |
| `fm_sim_scenarios` | ament_python | Seeded spawns, the success predicate, the MuJoCo, Gazebo and Isaac Sim generators and the scenario node (`/sim/*`) |
| `fm_sim_cameras` | ament_python | `sim_camera` node: a pinhole render warped into the real fisheye, JPEG on `/cam_<role>/image_raw/compressed` with the real intrinsics on `/cam_<role>/camera_info` |
| `fm_sim_msgs` | ament_cmake | `ObjectStates`, `Outcome` and the `Reset` service the scenario node serves |
| `fm_sim` | ament_cmake | Metapackage tying the eight together for a single install |

## Standalone Build

Clone into a colcon workspace's `src/`, pull dependencies, then build:

```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/first-motive/fm-sim.git
vcs import < fm-sim/fm-sim.repos     # externals: MJCF models, fm-robot, topic_based_ros2_control
cd .. && rosdep install --from-paths src external --ignore-src -y
colcon build --symlink-install
colcon test && colcon test-result --verbose
```

## Run

`run.sh` is the standalone front door for the dev loop: it launches
`fm_sim_core`'s `sim.launch.py`, the headless sim loop on mujoco or a backend
host on gazebo and isaac. The container path publishes port 8765 for Foxglove
Studio, but the launch starts no `foxglove_bridge`. A full run, with a scenario,
a controller and cameras, goes through [the run contract](#run-contract) or
[`fm sim`](#sim-agent). The sim stack targets ROS2
Jazzy only (Humble sim is retired). Jazzy has no macOS build, so on a Mac every
backend runs inside the fm-docker Linux container (OrbStack) —
there is no native-Mac path for any sim engine. Linux runs bare-metal native.

### Curl quickstart (macOS, no clone)

The image is published to GHCR, so `curl | bash` reaches a running sim with no
clone and no local build:

```bash
# Launch the baked mujoco demo straight from the published image. Piped through
# `bash` there is no terminal, so the backend menu is skipped and mujoco runs:
curl -fsSL https://raw.githubusercontent.com/first-motive/fm-sim/main/run.sh | bash

# Or set the host up first (install OrbStack + pull the image), then launch:
curl -fsSL https://raw.githubusercontent.com/first-motive/fm-sim/main/install.sh | bash
```

`install.sh` is macOS-only and idempotent; it delegates the OrbStack setup to
fm-docker and pulls the `fm-sim` image. Install gets it running — clone is the
dev path.

### Clone (the dev loop)

A clone mounts your working tree and rebuilds inside the container, so edits take
effect; the no-clone path runs the image's prebuilt workspace as-is.

```bash
git clone https://github.com/first-motive/fm-sim.git && cd fm-sim
./run.sh                       # on a terminal: pick the backend from a menu
./run.sh --backend gazebo      # skip the menu; gazebo, headless in the Mac container
./run.sh --native              # force the host path (Linux)
./run.sh params_file:=my.yaml  # extra args pass through to ros2 launch
```

#### Backend picker

Run `./run.sh` from a terminal with no `--backend` and it opens an interactive
menu — mujoco, gazebo, or isaac. The menu uses the branded
[`fm-tools`](https://github.com/first-motive/fm-tools) picker when `uv` can reach
it, and falls back to a plain bash `select` otherwise, so the choice always works.
It is terminal-gated: `--backend` and the `curl | bash` path (no TTY) skip it, so
the curl-to-launch flow is untouched. The chosen engine is forwarded to
`sim.launch.py` as `backend:=<engine>`. `mujoco` runs the in-process `sim_loop`;
`gazebo`/`isaac` need a robot description and a controllers file passed in,
which `sim.launch.py` does not build (`run.launch.py` builds both from the
catalog), and fail with a clear pointer when run without them. The description is
[`fm-robot`](https://github.com/first-motive/fm-robot)'s
`fm_control/urdf/<robot>.sim.urdf.xacro`, expanded with `sim_backend:=mujoco|gz|isaac`.

The standalone launch here is the sim loop. The clone container path imports
the shared compose overlays from
[`fm-docker`](https://github.com/first-motive/fm-docker) into `docker/` (via
`fm-sim.repos`) and builds this repo's `Dockerfile`, which is `FROM`
`ghcr.io/first-motive/fm-docker:jazzy`. fm-robot publishes only a Humble image, so
this image installs the ros2_control stack itself rather than inheriting it. Tear it down with
`docker compose -f docker/compose.yaml -f docker/compose.macos.yaml down`.

### Backend × host

Every Mac backend runs headless in the container; Linux runs native. `gazebo`
runs server-only (`gz -s`) under software GL — proven on the amd64 and arm64 CI
lanes, and on a Mac by `scripts/smoke.sh` (a smoke runner that drives the matrix
below and prints a pass/skip/fail table).

| Backend | macOS (container) | Linux (native) | Notes |
|---------|-------------------|----------------|-------|
| `mock` | headless | yes | sim loop, no engine |
| `mujoco` | headless; cameras on EGL (`MUJOCO_GL`) | yes | daily driver, CPU |
| `gazebo` | headless (`gz -s`) | yes (GPU) | server-only, software GL |
| `isaac` | — | yes (NVIDIA) | Isaac Sim app is Linux + NVIDIA only — never on macOS |

`isaac` is documented out on macOS by design: its Sim application is Linux +
NVIDIA only and never runs on Apple silicon or the container base image. Only its
topic bridge node is portable, and that stays on the Linux/NVIDIA host.

## Sim Agent

[`agent/`](agent/README.md) is a uv project, not a colcon package. It serves a
sim host's run control verbs (`status`, `catalog`, `up`, `down`, `reset`, `runs`,
`logs`) under `fm/sim/<host>/` over Zenoh, and runs each run as a container of
this repo's image.

`fm sim` is its client, mounted from this repo's `fm.json`:

```bash
fm sim catalog --host fm-ws-01
fm sim up      --host fm-ws-01 --scenario empty --controller idle --seed 3
fm sim down    --host fm-ws-01
fm sim trial   --host fm-ws-01 --seeds 0..19      # one run per seed, scored, JSON report
fm sim up      --host local                       # the agent in-process, on this Mac's OrbStack
fm sim host start aws --stack fm-sim-cloud-host   # the AWS sim host, through its controller
fm sim host start local                           # this Mac as a sim host the app can drive
```

See [the agent's README](agent/README.md#the-fm-sim-cli) for every verb and flag.

### Run Contract

One launch runs a backend × robot × scenario × controller combination the catalog
marks supported; each value is an entry registered through the ament index:

```bash
ros2 launch fm_sim_bringup run.launch.py \
  backend:=mujoco robot:=openarm scenario:=can_on_table controller:=idle seed:=3
ros2 run fm_sim_bringup catalog --json   # every registered entry
```

See [`fm_sim_bringup`](fm_sim_bringup/README.md) for the contract, the catalog
convention and the JSON schema.

### Cloud Host

[`infra/aws/sim/`](infra/aws/sim/README.md) is a CloudFormation stack for a sim
host on an AWS GPU instance. It holds zero or one instance, starts behind a
monthly budget gate, returns to zero after 15 idle minutes, and terminates at a
hard deadline.

### This Mac

`fm sim host status|pull|start|stop local` makes this Mac a sim host for the
desktop app: it checks that OrbStack (or Docker) answers, pulls the image at
the digest pinned in `agent/src/fm_sim_agent/service.py`, and runs the sim
agent in the background, with its pidfile and log under
`$XDG_STATE_HOME/fm-sim/local-agent/`. The agent serves
`fm/sim/<this machine>/*` (the machine card's name, else the hostname, `-` as
`_`; `local` is only the CLI's alias) through the fleet router, so the Mac needs `FM_ROUTER_ENDPOINT` (from
`/etc/fm-comms.env` or the environment). `--json` is what the app reads. See
[This Mac](agent/README.md#this-mac) in the agent's README.

## Architecture

Each engine hosts a `controller_manager` behind one interface, so the same
controllers drive any backend. `fm_sim_models` keeps the single map from robot to
MuJoCo model; `fm_sim_core` runs a headless dev loop split from ROS comms for
testability.

![backends](docs/diagrams/backends.svg)

Sim hosts, the run contract and its four dimensions, the agent's verbs and
error codes, the trust boundary, how to add a robot, scenario or controller, the
backend table, MJCF registry, and dev loop:
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Governance

Owner-free-on-main — see [CONTRIBUTING.md](CONTRIBUTING.md) and
[`.github/CODEOWNERS`](.github/CODEOWNERS).
