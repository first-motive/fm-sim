# fm_sim_backends

Simulation backend hosts: one launch file per physics engine. Each hosts the
`controller_manager` inside the sim, so the same controllers and description drive
any engine. These are included by `fm_bringup/sim.launch.py` and selected with its
`sim_backend` argument — they are not run standalone in normal use.

## Role

The robot description carries one `<ros2_control>` system whose `<hardware>` plugin
is chosen by `sim_backend`. Each backend here brings up the `controller_manager`
that the plugin talks to; `fm_bringup/sim.launch.py` then spawns the controllers
against it. Everything above the hardware interface is identical across backends —
see [Hardware Abstraction Layer](../../docs/ARCHITECTURE.md#hardware-abstraction-layer).

## Backends

| Launch file | Engine | Plugin | Host | Status |
|-------------|--------|--------|------|--------|
| `mujoco.launch.py` | MuJoCo | `mujoco_ros2_control/MujocoSystemInterface` | CPU (arm64 ok) | Daily driver, incl. M5 |
| `gazebo.launch.py` | Gazebo (gz-sim) | `gz_ros2_control/GazeboSimSystem` | Linux/GPU | Wired, not Mac-verifiable |
| `isaac.launch.py` | Isaac Sim | `topic_based_ros2_control/TopicBasedSystem` | Linux + NVIDIA | Wired, not Mac-verifiable |

- **MuJoCo** hosts the `controller_manager` inside the simulation: it loads the
  description, steps the MJCF physics, and exposes the hardware interfaces the
  controllers drive. The MJCF is the vendored `openarm_mujoco` v2 model, loaded in
  place so its relative `meshdir="assets"` resolves.
- **Gazebo** starts gz-sim, spawns the robot from `/robot_description`, and bridges
  the sim clock; a `<gazebo>` world plugin in the description hosts the
  `controller_manager`.
- **Isaac** runs as its own process and is the physics authority; the connection is
  purely topic-based over `/isaac_joint_states` and `/isaac_joint_commands`.

## Usage

Normally selected through the stack launch, not run directly:

```bash
./scripts/sim.sh --backend mujoco    # default
./scripts/sim.sh --backend gazebo    # Linux/GPU overlay
./scripts/sim.sh --backend isaac     # Linux/NVIDIA overlay
```

`scripts/sim.sh` picks the matching compose overlay; `gazebo` and `isaac` require
the Linux overlay.

## Build Type

`ament_python`. Ships launch files only — no nodes of its own.
