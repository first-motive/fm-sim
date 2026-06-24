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

`run.sh` is the standalone front door: it builds the workspace and launches the
headless sim loop that Foxglove Studio renders at `ws://localhost:8765`. The host
OS picks the path, overridable with `--native` / `--container`:

```text
Linux  -> native     build + launch on the host (needs ROS2 Humble installed)
Darwin -> container  build the fm-sim image, run it via the fm-docker overlays
```

`--backend` selects the host overlay — `mujoco` (the macOS default, CPU) and
`mock` run under the macOS overlay, while `gazebo` and `isaac` run under the
Linux/GPU overlay. The standalone launch here is the sim loop; the per-robot,
per-backend orchestration lives in `fm-app`'s `fm_bringup`.

```bash
./run.sh                       # auto-detect path, macOS overlay (mujoco)
./run.sh --backend gazebo      # force the Linux/GPU overlay
./run.sh --container           # force the container path (macOS / OrbStack)
./run.sh params_file:=my.yaml  # extra args pass through to ros2 launch
```

The container path imports the shared compose overlays from
[`fm-docker`](https://github.com/first-motive/fm-docker) into `docker/` (via
`fm-sim.repos`) and builds this repo's `Dockerfile`, which is `FROM` the
`fm-robot` layer. Tear down the container with
`docker compose -f docker/compose.yaml -f docker/compose.macos.yaml down`.

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
