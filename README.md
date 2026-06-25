# fm-sim

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

Simulation layer for First Motive's ROS2 stack. Groups the headless dev loop, the
backend hosts, and the MJCF model registry — the packages that run the robot in
simulation.

Part of First Motive's ROS2 (Humble) stack. Builds standalone here; assembled
with the other six package repos by
[`fm-ros2`](https://github.com/first-motive/fm-ros2).

## Packages

| Package | Build | Role |
|---------|-------|------|
| `fm_sim_core` | ament_python | Headless dev loop, launch, and shared sim config |
| `fm_sim_backends` | ament_python | Backend hosts (MuJoCo and others) behind one interface |
| `fm_sim_models` | ament_python | MJCF model registry |
| `fm_sim` | ament_cmake | Metapackage tying the three together for a single install |

## Standalone Build

Clone into a colcon workspace's `src/`, pull dependencies, then build:

```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/first-motive/fm-sim.git
vcs import < fm-sim/fm-sim.repos     # externals (MJCF model sources)
cd .. && colcon build --symlink-install
colcon test && colcon test-result --verbose
```

## Run

`run.sh` is the standalone front door: it launches the headless sim loop that
Foxglove Studio renders at `ws://localhost:8765`. ROS2 Humble has no macOS build,
so on a Mac every backend runs inside the fm-docker Linux container (OrbStack) —
there is no native-Mac path for any sim engine. Linux runs bare-metal native.

### Curl quickstart (macOS, no clone)

The image is published to GHCR, so `curl | bash` reaches a running sim with no
clone and no local build:

```bash
# Launch the baked mujoco demo straight from the published image:
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
./run.sh                       # mount source, rebuild, launch (mujoco, the Mac default)
./run.sh --backend gazebo      # gazebo, headless in the Mac container
./run.sh --native              # force the host path (Linux)
./run.sh params_file:=my.yaml  # extra args pass through to ros2 launch
```

The standalone launch here is the sim loop; the per-robot, per-backend
orchestration lives in `fm-app`'s `fm_bringup`. The clone container path imports
the shared compose overlays from
[`fm-docker`](https://github.com/first-motive/fm-docker) into `docker/` (via
`fm-sim.repos`) and builds this repo's `Dockerfile`, which is `FROM` the
`fm-robot` layer. Tear it down with
`docker compose -f docker/compose.yaml -f docker/compose.macos.yaml down`.

### Backend × host

Every Mac backend runs headless in the container; Linux runs native. `gazebo`
runs server-only (`gz -s`) under software GL — proven on the amd64 CI lane, with
arm64 captured by `scripts/smoke.sh` (a Mac smoke runner that drives the matrix
below and prints a pass/skip/fail table).

| Backend | macOS (container) | Linux (native) | Notes |
|---------|-------------------|----------------|-------|
| `mock` | headless | yes | sim loop, no engine |
| `mujoco` | headless (xvfb) | yes | daily driver, CPU |
| `gazebo` | headless (`gz -s`) | yes (GPU) | server-only, software GL |
| `isaac` | — | yes (NVIDIA) | Isaac Sim app is Linux + NVIDIA only — never on macOS |

`isaac` is documented out on macOS by design: its Sim application is Linux +
NVIDIA only and never runs on Apple silicon or the container base image. Only its
topic bridge node is portable, and that stays on the Linux/NVIDIA host.

## Architecture

Each engine hosts a `controller_manager` behind one interface, so the same
controllers drive any backend. `fm_sim_models` keeps the single map from robot to
MuJoCo model; `fm_sim_core` runs a headless dev loop split from ROS comms for
testability.

![backends](docs/diagrams/backends.svg)

Full backend table, MJCF registry, and dev loop:
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Governance

Owner-free-on-main — see [CONTRIBUTING.md](CONTRIBUTING.md) and
[`.github/CODEOWNERS`](.github/CODEOWNERS).
