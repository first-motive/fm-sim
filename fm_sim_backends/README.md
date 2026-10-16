# fm_sim_backends

Simulation backend hosts for ROS2 Jazzy: one launch file per physics engine. Each
hosts the `controller_manager` inside the sim, so the same controllers and
description drive any engine. A robot bringup includes one, selected by its
`sim_backend` argument, and leaves the description publisher to it (see Role);
`fm_sim_core`'s `sim.launch.py backend:=gazebo|isaac` is the standalone way in.

## Role

The robot description is fm-robot's `fm_control/urdf/<robot>.sim.urdf.xacro`. It
carries one `<ros2_control>` system whose `<hardware>` plugin is chosen by its
`sim_backend` arg (`mujoco`, `gz` or `gazebo`, `isaac`). Each backend here runs the
`robot_state_publisher` and brings up the `controller_manager` that the plugin
talks to; the including launch then spawns the controllers against it. On Jazzy the
`controller_manager` reads the description from the `/robot_description` topic,
which is why every backend publishes it. All three build that node from
`fm_sim_backends.description.robot_state_publisher_node`. Everything above the
hardware interface is identical across backends — see
[Backend Hosts](../docs/ARCHITECTURE.md#backend-hosts).

Because the backend owns the `robot_state_publisher`, a launch that includes one
must not start its own: two publishers would own the same TF and node name.
fm-app's Humble `fm_bringup/sim.launch.py` still does, so it runs a duplicate until
a follow-up moves the Teleoperate sim onto this stack. The backends
also keep passing `robot_description` to the `controller_manager` as a parameter,
which Jazzy ignores and that Humble launch still needs.

## Backends

| Launch file | Engine | Plugin | Host | Status |
|-------------|--------|--------|------|--------|
| `mujoco.launch.py` | MuJoCo | `mujoco_ros2_control/MujocoSystemInterface` | CPU (arm64 ok) | Daily driver, incl. M5 |
| `gazebo.launch.py` | Gazebo Harmonic | `gz_ros2_control/GazeboSimSystem` | Linux/GPU | Wired; engine smoked headless |
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
  `topic_based_ros2_control` has no Jazzy apt release; `fm-sim.repos` pins it as a
  source external.

## Catalog

The package registers each backend in the sim catalog (`fm_sim.backends` in the
ament index; see [`fm_sim_bringup`](../fm_sim_bringup/README.md)). An entry in
`catalog/backends/` names its launch file and the xacro args that select its
hardware plugin, so `run.launch.py backend:=<name>` needs no per-backend code.

## Usage

Include one from a robot bringup, or reach gazebo and isaac standalone through
`fm_sim_core`:

```bash
desc="$(xacro "$(ros2 pkg prefix --share fm_control)/urdf/openarm.sim.urdf.xacro" \
  robot_preset:=default_bimanual sim_backend:=isaac)"
ros2 launch fm_sim_core sim.launch.py backend:=isaac \
  robot_description:="$desc" controllers_file:=/path/to/controllers.yaml
```

| Argument | Backends | Description |
|----------|----------|-------------|
| `robot_description` | all | Description XML expanded with the matching `sim_backend` |
| `controllers_file` | all (gazebo: reserved) | `controllers.yaml` for the preset |
| `gz_args` | gazebo | Passed to gz-sim; default `-r -v 3 empty.sdf` |
| `entity_name` | gazebo | Spawned entity name; default `openarm` |

## Tests

`test/test_launch.py` reads each launch's actions: the `robot_state_publisher`
gets the `robot_description` argument, and the mujoco and isaac
`controller_manager` still get it as a parameter. `test/test_description_topic.py`
launches mujoco (under `xvfb-run`) and isaac live with the one-joint probe robots
in `test/fixtures/` and waits for `/robot_description`.

## Build Type

`ament_python`. Ships launch files only — no nodes of its own. Its exec
dependencies name every engine host and hardware plugin, so `rosdep` installs the
whole backend set.
