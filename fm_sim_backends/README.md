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
  controllers drive. The MJCF is the `mujoco_model` the description names: under
  `run.launch.py`, the world `fm_sim_scenarios` generates around the vendored
  `openarm_mujoco` v2 model, which it attaches in place so its relative
  `meshdir="assets"` resolves. [`config/mujoco_plugins.yaml`](config/mujoco_plugins.yaml)
  always loads mujoco_ros2_control's free-joint state publisher, putting every
  free body's world pose on `/mujoco/free_joint_states` at 30 Hz of sim time, and
  its CameraPlugin under the key `mujoco_cameras`, which renders every camera in
  the world offscreen at 10 Hz of sim time and publishes it raw on
  `/mujoco_cameras/<camera>/color` (also `depth` and `camera_info`, which nothing
  reads). `fm_sim_cameras` turns those into the robot's `/cam_<role>` topics. A
  world with no cameras starts no render thread. These are mujoco_ros2_control
  0.1.2's CameraPlugin behaviours (`camera_plugin.cpp`), checked live by
  `fm_sim_bringup`'s `test_camera_launch.py`.
- **MuJoCo rendering** follows `MUJOCO_GL`. The engine itself always runs headless
  (the description's `headless` parameter); the variable picks how the
  CameraPlugin gets its GL context:

  | `MUJOCO_GL` | Context | Where |
  |-------------|---------|-------|
  | `egl` (default on Linux) | EGL surfaceless; the process gets no display, so the plugin's GLFW init fails and it falls back to EGL. glvnd picks the GPU's EGL vendor when one is installed, else Mesa | Container, GPU host |
  | `osmesa` | As `egl`, with EGL pinned to Mesa (`__EGL_VENDOR_LIBRARY_FILENAMES`) and its software rasteriser forced (`LIBGL_ALWAYS_SOFTWARE=1`): llvmpipe, the renderer OSMesa wraps. The plugin has no OSMesa path of its own | Host with no GPU, e.g. the Mac under OrbStack |
  | `glfw` (default and only choice on macOS) | A hidden GLFW window, under `xvfb-run` on Linux | Debugging |

  Any other value, set but empty included, fails the launch, as do `egl` and
  `osmesa` on macOS.
- **Gazebo** starts gz-sim, spawns the robot from `/robot_description`, and bridges
  the sim clock; a `<gazebo>` world plugin in the description hosts the
  `controller_manager`.
- **Isaac** runs as its own process and is the physics authority; the connection is
  purely topic-based over `/isaac_joint_states` and `/isaac_joint_commands`.
  `topic_based_ros2_control` has no Jazzy apt release; `fm-sim.repos` pins it as a
  source external. Under `run.launch.py` this launch also builds the Isaac stage
  (see [Isaac Stage](#isaac-stage)).

## Isaac Stage

Isaac Sim 4.5 code, not run in CI: the stage spec is generated and unit-tested
here; building it needs Isaac Sim on the GPU host.

Given `world` (the scenario half `fm_sim_scenarios.isaac_stage.write` wrote) the
isaac launch writes `robot.urdf` (every mesh filename an absolute path: the
robot entry's `mesh_roots` first, then `package://` through the package's share)
and `stage.json` beside it, then:

- starts, per camera in `cameras_file`, image_transport's republisher
  (`/cam_<role>/image_raw` → `/cam_<role>/image_raw/compressed`), and one
  `camera_info` node that answers every image with the cameras file's intrinsics
  on `/cam_<role>/camera_info` (same header; none for an image of another size);
- runs [`isaac/fm_sim_isaac.py`](isaac/fm_sim_isaac.py) with Isaac Sim's Python,
  under `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, when `isaac_python` (default
  `$FM_ISAAC_PYTHON`) is set and a world is given; with no `isaac_python` it logs
  the command with `$ISAAC_SIM/python.sh` in its place. Isaac Sim must see the
  same filesystem as the launch.

The world must sit in the run's private `fm_sim_world_*` directory, a mesh path
must stay inside the mesh root or package share it resolved against (a relative
one no root matches is refused), and a camera renders at most 1920x1080
(more with `allow_large_image: true`) at a `rate_hz` that divides the 60 Hz
render rate. The cameras file is checked by `fm_sim_cameras.cameras_file`, the
loader every backend shares (roles `chest` and `wrist_right`, the size cap, the
distortion models); the 60 Hz rule and the distortion models Isaac renders are
Isaac's own checks in `isaac_stage.check_cameras`, and the `camera_info` node's
CameraInfo comes from `fm_sim_cameras.camera_info`. The republisher's remap names `out/compressed` itself: on Jazzy a
remap of the base name `out` leaves the output on `/out/compressed`.

The runner imports the URDF (fixed joints merged, base fixed) at the stage origin
with position drives on every joint, adds the scenario objects (collision, a
rigid body with the scenario mass when free, a physics material with the
scenario friction), a camera on each camera frame's moving link (OpenCV fisheye
from the real intrinsics for an `equidistant` camera), and this OmniGraph:

| Graph output or input | Topic | Type |
|-----------------------|-------|------|
| Simulation time | `/clock` | `rosgraph_msgs/Clock` |
| Articulation joint states | the description's `joint_states_topic` (`/isaac_joint_states`) | `sensor_msgs/JointState` |
| Joint commands, applied by name | the description's `joint_commands_topic` (`/isaac_joint_commands`) | `sensor_msgs/JointState` |
| One RGB image per camera, frame = its optical frame | `/cam_<role>/image_raw` | `sensor_msgs/Image` |
| Free object poses from `World` | `/isaac/object_poses` | `tf2_msgs/TFMessage` |

It also applies object teleports from `/isaac/object_pose_commands`
(`tf2_msgs/TFMessage`) through Isaac Sim's bundled rclpy: the scenario node's
reset. A command moves only a free object, to a finite position inside the
spec's `object_workspace` (the objects' poses and spawn ranges plus 0.25 m) with
a non-zero quaternion, normalised; anything else, and a teleport that raises, is
logged and skipped. A standard type, because Isaac Sim's Python cannot load `fm_sim_msgs`.

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `''` | Scenario half of the stage; empty builds no stage |
| `cameras_file` | `''` | The robot's cameras file; empty renders no cameras |
| `mesh_roots` | `[]` | JSON `[[prefix, directory]]` for mesh URIs no package resolves (the robot entry's `mesh_roots`) |
| `isaac_python` | `$FM_ISAAC_PYTHON` | Isaac Sim's `python.sh`; empty logs the runner command |
| `isaac_gui` | `false` | `true` opens the Isaac Sim window |

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
| `world`, `cameras_file`, `mesh_roots`, `isaac_python`, `isaac_gui` | isaac | See [Isaac Stage](#isaac-stage) |
| `gz_args` | gazebo | Passed to gz-sim; default `-r -v 3 empty.sdf` |
| `entity_name` | gazebo | Spawned entity name; default `openarm` |

## Tests

`test/test_launch.py` reads each launch's actions: the `robot_state_publisher`
gets the `robot_description` argument, the mujoco and isaac
`controller_manager` still get it as a parameter, mujoco's also gets the
plugins file, and `MUJOCO_GL` sets its environment and prefix as the table above
says. `test/test_description_topic.py`
launches mujoco (default `egl`) and isaac live with the one-joint probe robots
in `test/fixtures/`, waits for `/robot_description`, and after shutdown checks that
no controller_manager is left running. `test/test_isaac.py` covers the stage
wiring without Isaac Sim: the runner's pure helpers (schema check, the
`og.Controller.edit` arguments, object command validation), the `camera_info` node live,
the relays and runner command the launch starts, and the launch arguments. `test/conftest.py` gives the package's tests a
ROS domain of their own, so packages tested in parallel do not see each other's topics.

## Build Type

`ament_python`. Ships launch files, the mujoco plugins config, the Isaac stage
runner and one node, `camera_info`. Its exec dependencies name every engine host and hardware plugin, so
`rosdep` installs the whole backend set.
