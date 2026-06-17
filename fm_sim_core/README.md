# fm_sim_core

Headless MuJoCo simulation core. A ROS-free stepper plus the `sim_loop` node that
publishes joint states, for control and orchestration development without the full
`ros2_control` stack.

## Role

```
MujocoStepper (ROS-free physics)  →  sim_loop (ROS comms)  →  /joint_states
```

The split is deliberate: `MujocoStepper` owns the model and physics so it unit-tests
without a node, and `sim_loop` handles ROS comms only. The `mujoco` module is injected
into the stepper (lazy import by default), so the package loads and tests run without
the native wheel installed. This is the headless path — no controllers, no
`ros2_control`. For the full control stack in a sim, see
[`fm_sim_backends`](../fm_sim_backends/README.md).

Runs native arm64 on the M5 (CPU, no GPU).

## Usage

```bash
ros2 run fm_sim_core sim_loop                       # built-in 1-DOF fallback model
ros2 launch fm_sim_core sim.launch.py               # same, params from config/sim.yaml
ros2 launch fm_sim_core sim.launch.py params_file:=/path/to/my.yaml
```

Override the packaged defaults with your own `params_file` rather than editing
`config/sim.yaml`.

## Nodes

### `sim_loop`

Steps a MuJoCo model at a fixed rate and publishes the resulting joint state.

**Publishes**

| Topic | Type | Description |
|-------|------|-------------|
| `joint_states` | `sensor_msgs/JointState` | Per-step joint name, position, velocity |

**Parameters**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_path` | string | `""` | MJCF file to load; empty loads the built-in 1-DOF fallback |
| `rate_hz` | double | `100.0` | Step + publish rate |

## Build Type

`ament_python`. `mujoco` is a pip dependency (arm64 CPU wheel) installed via the base
image, not a rosdep.
