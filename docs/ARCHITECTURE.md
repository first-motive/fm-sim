# Architecture

The simulation layer of First Motive's ROS2 (Humble) stack. It separates the
physics stepper (ROS-free, testable in isolation) from the ROS comms, hosts a
`controller_manager` per engine so the same controllers drive any backend, and
keeps one registry mapping each robot to its MuJoCo model.

This repo is the sim layer in isolation. The full sim stack — how a backend is
selected, the description built, and controllers spawned — is composed by
`fm_bringup` in [`fm-app`](https://github.com/first-motive/fm-app). For the
system-wide view see [`fm-ros2`](https://github.com/first-motive/fm-ros2).

## Packages

| Package | Build | Responsibility |
|---------|-------|----------------|
| `fm_sim_core` | ament_python | Headless MuJoCo stepper + `sim_loop` node; physics split from ROS comms for testability |
| `fm_sim_backends` | ament_python | One launch file per engine (MuJoCo, Gazebo, Isaac), each hosting a `controller_manager` |
| `fm_sim_models` | ament_python | MJCF model registry — single source of truth mapping robot key → MuJoCo model path |
| `fm_sim` | ament_cmake (meta) | Metapackage tying the three together for a single install |

## Backend Hosts

`fm_bringup/sim.launch.py` picks a backend by the `sim_backend` argument and
includes the matching `fm_sim_backends` launch file. Each launch file hosts a
`controller_manager` inside its engine, so the controllers above the hardware
interface are identical across every backend.

![backends](diagrams/backends.svg)

Source: [`diagrams/backends.d2`](diagrams/backends.d2).

| Backend | Launch file | Hosts | Notes |
|---------|-------------|-------|-------|
| `mujoco` | `mujoco.launch.py` | `mujoco_ros2_control/ros2_control_node` under `xvfb-run` | **Daily driver**, native arm64 on M5 (CPU, no GPU) |
| `gazebo` | `gazebo.launch.py` | `gz_sim` + `ros_gz_sim/create` + `ros_gz_bridge` (clock) | Wired; Linux/GPU only |
| `isaac` | `isaac.launch.py` | standalone `controller_manager` bridged over `/isaac_joint_states` ↔ `/isaac_joint_commands` | Wired; Linux + NVIDIA, Isaac runs externally |

The `mock` and `real` backends do not live here — they spawn a standalone
`controller_manager` directly from `fm_bringup`. The `<hardware>` plugin each
backend binds to is defined in [`fm-robot`](https://github.com/first-motive/fm-robot)
(`fm_control`); this layer provides the engine hosts, not the interface.

## Host Verification

ROS2 Humble has no native macOS build, so on a Mac every backend runs inside the
fm-docker Linux container (OrbStack) — there is no native-Mac path for any sim
engine. Linux runs bare-metal native. `run.sh` picks the path from the host OS;
`scripts/smoke.sh` drives the matrix on a Mac and the `smoke-container` CI job
drives it on amd64 ubuntu.

| Backend | macOS (container) | Linux (native) | How it comes up headless |
|---------|-------------------|----------------|--------------------------|
| `mock` | headless | yes | `sim_loop`, no engine |
| `mujoco` | headless | yes | `ros2_control_node` under `xvfb-run` |
| `gazebo` | headless | yes (GPU) | `gz -s` server-only, software GL |
| `isaac` | — | yes (NVIDIA) | not on macOS — Isaac Sim is Linux + NVIDIA |

`gazebo` headless on arm64 under software GL is the genuine unknown — it had
never been run, so `smoke.sh` captures PASS / SKIP / FAIL rather than assuming
success, and the amd64 CI lane proves the launch logic regardless.

`isaac` is out on macOS by design. The Isaac Sim application is Linux + NVIDIA
only and never runs on Apple silicon or the container base image; only its topic
bridge node is portable, and that stays on the Linux/NVIDIA host. There is no
macOS isaac code, by intent — not an omission.

## MJCF Registry

`fm_sim_models.mjcf_path(robot_key)` is the sole source of MuJoCo model paths — no
path is hardcoded in any xacro. When `sim_backend == "mujoco"`, `fm_bringup`
injects the looked-up path as the `mujoco_model` xacro argument.

| Robot key | MJCF | Status |
|-----------|------|--------|
| `openarm` | `openarm_mujoco/v2/openarm_bimanual.xml` | Joint names match description |
| `so101` | `so_arm/Simulation/SO101/so101_new_calib.xml` | Joint names match system |
| `g1_d` | `unitree_mujoco/.../g1_29dof.xml` | Wired, not yet validated — bipedal legs differ from wheeled G1-D |

Models are vendored under `external/` (gitignored on host, mounted at `/ws` in the
dev container) via `fm-sim.repos`.

## Headless Dev Loop

`fm_sim_core` runs a different, lighter path: a headless control loop with no
controllers and no `ros2_control`. The `MujocoStepper` is ROS-free — it owns a
loaded model, steps it, and returns a `JointSample`. The `sim_loop` node wraps it
and publishes `/joint_states`.

![sim_loop](diagrams/sim_loop.svg)

Source: [`diagrams/sim_loop.d2`](diagrams/sim_loop.d2).

| Parameter | Type | Default | Role |
|-----------|------|---------|------|
| `model_path` | string | `""` (1-DOF fallback model) | MJCF to step |
| `rate_hz` | double | `100.0` | Step + publish frequency |

The stepper takes an injected `mujoco` module, so unit tests run against a fake —
no native wheel required. Note `fm_sim_core/sim.launch.py` is a *different* file
from `fm_bringup/sim.launch.py` (the full sim stack); they share a name but neither
includes the other.

## Design Notes

| Principle | How it shows up | Payoff |
|-----------|-----------------|--------|
| **Physics split from ROS** | `MujocoStepper` (no ROS) + `SimLoop` (node wrapper) | Step logic unit-tests without rclpy or the native wheel |
| **One interface, many engines** | Each backend hosts a `controller_manager`; controllers are backend-agnostic | Sim ↔ sim ↔ real is a launch arg |
| **Single MJCF source** | `fm_sim_models.mjcf_path` is the only path lookup | No model path duplicated across xacros |

Per-package detail lives in each `<package>/README.md`.
