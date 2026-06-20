# fm-sim

[![License: Proprietary](https://img.shields.io/badge/License-Proprietary-blue.svg)](LICENSE)

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

## Governance

Owner-free-on-main — see [CONTRIBUTING.md](CONTRIBUTING.md) and
[`.github/CODEOWNERS`](.github/CODEOWNERS).
