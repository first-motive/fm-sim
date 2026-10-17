# Architecture

The simulation layer of First Motive's ROS2 stack. It separates the
physics stepper (ROS-free, testable in isolation) from the ROS comms, hosts a
`controller_manager` per engine so the same controllers drive any backend, and
keeps one registry mapping each robot to its MuJoCo model.

This repo is the sim layer, on ROS2 Jazzy only (Humble sim is retired). The robot
descriptions it hosts come from
[`fm-robot`](https://github.com/first-motive/fm-robot):
`fm_control/urdf/<robot>.sim.urdf.xacro`, whose `sim_backend:=mock|mujoco|gz|gazebo|isaac|real`
arg selects the `<hardware>` plugin and fails on any other value. For the
system-wide view see [`fm-ros2`](https://github.com/first-motive/fm-ros2).

## Packages

| Package | Build | Responsibility |
|---------|-------|----------------|
| `fm_sim_core` | ament_python | Headless MuJoCo stepper + `sim_loop` node; physics split from ROS comms for testability |
| `fm_sim_backends` | ament_python | One launch file per engine (MuJoCo, Gazebo, Isaac), each hosting a `controller_manager` |
| `fm_sim_models` | ament_python | MJCF model registry — single source of truth mapping robot key → MuJoCo model path; vendored models resolve under `/ws/external`, while `axol`'s committed MJCF resolves from this package's share |
| `fm_sim_ee_follower` | ament_python | `ee_follower` node: damped-least-squares IK from `/commanded_ee_right` to the right arm's forward position controller, standing in for the loader's IK; ROS-free DLS core |
| `fm_sim_bringup` | ament_python | `run.launch.py` and the catalog of backends, robots, scenarios and controllers, resolved from the ament index |
| `fm_sim_scenarios` | ament_python | Backend-neutral scenario logic (seeded spawns, success predicate), the per-backend world generators (MuJoCo, and the Isaac Sim stage spec) and the scenario node serving `/sim/objects`, `/sim/markers`, `/sim/reset`, `/sim/outcome` |
| `fm_sim_cameras` | ament_python | Simulated cameras as the real ones publish them: camera mounts from the description, the fisheye warp of a pinhole render, and the `sim_camera` node publishing `/cam_<role>/image_raw/compressed` and `/cam_<role>/camera_info` |
| `fm_sim_msgs` | ament_cmake | The scenario interfaces: `ObjectState(s)`, `Outcome`, `Reset` |
| `fm_sim` | ament_cmake (meta) | Metapackage tying the eight together for a single install |

## Run Contract

`fm_sim_bringup/run.launch.py` takes four names, `backend`, `robot`, `scenario` and
`controller`, plus `use_sim`, `seed` and `timeout_s`. Each is an entry in an ament resource type
(`fm_sim.backends`, `fm_sim.robots`, `fm_sim.scenarios`, `fm_sim.controllers`)
whose content points at the entry's yaml, so any package registers an entry by
installing two files. The launch validates all four, expands the robot's fm-robot
xacro with the backend's args, includes the backend host, spawns the robot's
controllers by role, and includes the controller's own launch. On MuJoCo it first
builds the scenario into one world with the robot and its cameras, and a scenario
with objects adds the scenario node; see
[Scenarios](../fm_sim_bringup/README.md#scenarios) and
[Cameras](../fm_sim_bringup/README.md#cameras).

![run](diagrams/run.svg)

Source: [`diagrams/run.d2`](diagrams/run.d2). Entry fields, the `catalog --json`
output and its schema: [`fm_sim_bringup`](../fm_sim_bringup/README.md). A robot
lists the backends it runs on (`supported_backends`) and the launch refuses the
rest.

fm-robot is built from source (pinned in `fm-sim.repos`) because it publishes no
Jazzy image. Its `fm_control` needs moveit_servo and the Unitree messages: rosdep
installing MoveIt is most of the cost, while building fm-robot and its two
externals takes seconds.

## Backend Hosts

A robot bringup picks a backend by its `sim_backend` argument and includes the
matching `fm_sim_backends` launch file. Each launch file hosts a
`controller_manager` inside its engine, so the controllers above the hardware
interface are identical across every backend. Each one also runs the
`robot_state_publisher`: the Jazzy `controller_manager` reads the description
from the `/robot_description` topic, never from a node parameter.

![backends](diagrams/backends.svg)

Source: [`diagrams/backends.d2`](diagrams/backends.d2).

| Backend | Launch file | Hosts | Notes |
|---------|-------------|-------|-------|
| `mujoco` | `mujoco.launch.py` | `mujoco_ros2_control/ros2_control_node`, headless; its CameraPlugin renders on EGL, or as `MUJOCO_GL` says | **Daily driver**, native arm64 on M5 (CPU, no GPU) |
| `gazebo` | `gazebo.launch.py` | Gazebo Harmonic: `gz sim` in its own process group, `create`, `ros_gz_bridge` (clock, object poses, camera images, `set_pose`); `gz_ros2_control` in-engine | Linux/GPU, or server-only with headless rendering anywhere. Runs the generated world SDF, the robot with camera sensors, and the camera relays |
| `isaac` | `isaac.launch.py` | standalone `controller_manager` with `topic_based_ros2_control`, bridged over `/isaac_joint_states` ↔ `/isaac_joint_commands` | Wired; Linux + NVIDIA. Builds the Isaac Sim 4.5 stage (URDF, objects, cameras, ROS 2 bridge graph) and starts Isaac when `FM_ISAAC_PYTHON` is set and a world is given |

`topic_based_ros2_control` has no Jazzy apt release, so `fm-sim.repos` pins it as
a source external that CI and the image build.

The `mock` and `real` backends do not live here — a robot bringup spawns a
standalone `controller_manager` for them directly. The `<hardware>` plugin each
backend binds to is defined in [`fm-robot`](https://github.com/first-motive/fm-robot)
(`fm_control`); this layer provides the engine hosts, not the interface.

## Host Verification

ROS2 Jazzy has no native macOS build, so on a Mac every backend runs inside the
fm-docker Linux container (OrbStack) — there is no native-Mac path for any sim
engine. Linux runs bare-metal native. `run.sh` picks the path from the host OS;
`scripts/smoke.sh` drives the matrix on a Mac and the `smoke-container` CI job
drives it on amd64 ubuntu.

| Backend | macOS (container) | Linux (native) | How it comes up headless |
|---------|-------------------|----------------|--------------------------|
| `mock` | headless | yes | `sim_loop`, no engine |
| `mujoco` | headless | yes | `ros2_control_node` headless; cameras on EGL with no display (`MUJOCO_GL=osmesa` forces Mesa's software rasteriser) |
| `gazebo` | headless | yes (GPU) | `gz -s` server-only, software GL |
| `isaac` | — | yes (NVIDIA) | not on macOS — Isaac Sim is Linux + NVIDIA |

On Jazzy, Gazebo Harmonic ships arm64 binaries, and `gazebo` runs headless on
arm64 under software GL. `smoke.sh` still captures PASS / SKIP / FAIL rather than
assuming success, and the amd64 CI lane covers the same matrix.

`isaac` is out on macOS by design. The Isaac Sim application is Linux + NVIDIA
only and never runs on Apple silicon or the container base image; only its topic
bridge node is portable, and that stays on the Linux/NVIDIA host. There is no
macOS isaac code, by intent — not an omission.

## MJCF Registry

`fm_sim_models.mjcf_path(robot_key)` is the sole source of MuJoCo model paths — no
path is hardcoded in any xacro. With `sim_backend:=mujoco`, the caller expanding
fm-robot's sim xacro passes the looked-up path as its `mujoco_model` argument.

| Robot key | MJCF | Status |
|-----------|------|--------|
| `openarm` | `openarm_mujoco/v2/openarm_bimanual.xml` | Joint names match description |
| `so101` | `so_arm/Simulation/SO101/so101_new_calib.xml` | Joint names match system |
| `g1_d` | `unitree_mujoco/.../g1_29dof.xml` | Wired, not yet validated — bipedal legs differ from wheeled G1-D |
| `axol` | `models/axol/axol.xml` (package share) | Almond Bot Axol, bimanual 2×7-DOF — mujoco validated; real CAN deferred |

Most models are vendored under `external/` (gitignored on host, mounted at `/ws` in
the dev container) via `fm-sim.repos`. `axol` is the exception: it has no upstream
MJCF, so First Motive compiled one from `axol.urdf` and committed it into
`fm_sim_models`' share at `models/axol/axol.xml`; the registry resolves it from the
package share rather than `/ws/external`.

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
no native wheel required.

## EE Follower

On the real OpenArm, the Anvil loader turns a commanded tool pose into joint commands.
In the sim, `fm_sim_ee_follower`'s `ee_follower` node does that job: it reads
`/commanded_ee_right` (`PoseStamped` in the profile's `base_frame`), takes one
damped-least-squares step per tick from `/joint_states`, and streams the result to the
right arm's forward position controller. Missing, stale or non-finite input means no
command that tick, and a command in another frame clears the last good one. See the
[package README](../fm_sim_ee_follower/README.md).

![ee_follower](diagrams/ee_follower.svg)

Source: [`diagrams/ee_follower.d2`](diagrams/ee_follower.d2).

## Design Notes

| Principle | How it shows up | Payoff |
|-----------|-----------------|--------|
| **Physics split from ROS** | `MujocoStepper` (no ROS) + `SimLoop` (node wrapper) | Step logic unit-tests without rclpy or the native wheel |
| **One interface, many engines** | Each backend hosts a `controller_manager`; controllers are backend-agnostic | Sim ↔ sim ↔ real is a launch arg |
| **Single MJCF source** | `fm_sim_models.mjcf_path` is the only path lookup | No model path duplicated across xacros |
| **Catalog by registration** | Every run dimension is an ament index entry checked against one schema | A controller or robot joins from its own package |

Per-package detail lives in each `<package>/README.md`.
