# fm_sim_bringup

The sim run contract: one `run.launch.py` that runs a backend, a robot, a scenario
and a controller together, and the catalog that registers what each of those four
can be. Every value comes from the ament resource index, so a package outside
fm-sim (a controller in fm-skills, say) joins the catalog by installing an entry,
with no change here.

## Run Contract

```bash
ros2 launch fm_sim_bringup run.launch.py \
  backend:=mujoco robot:=so101 scenario:=empty controller:=idle
```

| Argument | Default | Catalog type | Description |
|----------|---------|--------------|-------------|
| `backend` | `mujoco` | `fm_sim.backends` | Engine host from `fm_sim_backends` |
| `robot` | `so101` | `fm_sim.robots` | Robot description, MJCF and controllers |
| `scenario` | `empty` | `fm_sim.scenarios` | Scene contents |
| `controller` | `idle` | `fm_sim.controllers` | Controller under test |
| `use_sim` | `true` | — | Passed to the controller's launch |
| `seed` | `0` | — | Spawn seed for the scenario's objects, an integer in [0, 2^32 - 1] |
| `timeout_s` | empty | — | Episode timeout in seconds; empty keeps the scenario's |
| `overrides_file` | empty | — | JSON of tunable overrides, `<dimension>.<name>: value` (see [Run Params](#run-params)) |
| `params_file` | empty | — | ROS params file keyed by fully qualified node name, appended last (see [Run Params](#run-params)) |

An unknown value fails before anything starts and lists the registered ones; an
entry marked `available: false` is listed but refused, as is one whose
`requires_packages` are not all installed (the error names the missing ones), and
so is a robot on a backend missing from its `supported_backends`.
`use_sim` takes only `true` or `false`; `seed` and `timeout_s` refuse anything
outside their ranges.

```mermaid
flowchart LR
    args["backend robot scenario controller"] --> idx[(ament index<br/>fm_sim.*)]
    idx --> desc["xacro: robot args<br/>+ backend args"]
    desc --> be["backend launch<br/>robot_state_publisher<br/>controller_manager"]
    idx --> sp["spawners<br/>broadcasters + activated roles: active<br/>other roles: inactive"]
    sp --> be
    idx --> ctl["controller launch<br/>use_sim, robot_profile, scenario"]
    idx --> gen["scenario generator<br/>mujoco: one MJCF world<br/>gazebo: world SDF<br/>isaac: stage objects"]
    gen --> desc
    idx --> sn["scenario node<br/>/sim/objects /sim/markers<br/>/sim/reset /sim/outcome"]
```

What `run.launch.py` does, in order:

1. On a backend with a scenario generator, spawns the scenario's objects from
   `seed` into a generated world (see [Scenarios](#scenarios)): on mujoco the
   robot's MJCF and the objects as one world; on gazebo the world SDF and on
   isaac the scenario half of the Isaac stage, which the backend launch
   completes. A backend without one runs the robot's own model and refuses a
   scenario that has objects.
2. Expands the robot's xacro with the robot's `description_args` and the backend's.
   The backend's args select its hardware plugin; `{mjcf_path}` (the generated
   world, or the robot's model from `fm_sim_models`) and `{controllers_file}` are
   filled in.
3. Includes the backend launch with that description, the robot's controllers
   file and the backend's optional `launch_args` (gazebo's run server-only). A
   backend that finishes its own stage (`run.STAGE_BACKENDS`: gazebo, isaac) also
   gets the generated `world` and the robot's `cameras_file` (each empty without
   one), its `mesh_roots` resolved to share directories, as JSON, and the robot
   entry's name as `robot` (gazebo names the spawned model with it).
4. Spawns the robot's controllers. The `broadcasters` role and every role the
   controller entry `activate`s start active; the others load inactive, so a
   controller can switch to them.
5. Includes each launch role the controller entry `activate`s (the robot's
   `launch_roles`, below) once the active spawner has exited cleanly, with the
   role's `launch_args` and the run's `robot_description`. A launch role's
   `requires` roles start active with it, and a role listed twice starts once. If
   the spawner fails, the run shuts down.
6. Includes the controller entry's `launch`, if any, with its optional
   `launch_args` (`perception: model`, say), `use_sim`, `robot_profile` and
   `scenario`. A robot whose `robot_profile` is `null` refuses a controller with a
   `launch`, since that launch reads its names from the profile. ROS launch does
   not scope included arguments, so a launch role started later reads a controller
   `launch_args` value for a name it declares and its own `launch_args` leave unset.
7. For a scenario with objects, starts the scenario node (`fm_sim_scenarios`) with
   `use_sim_time`. `empty` has none, so it has no `/sim/reset` or `/sim/outcome`.

Before step 1, the overrides in `overrides_file` are checked and merged into the
four entries, and the robot's MJCF is patched; `params_file` reaches every node
from step 3 on. See [Run Params](#run-params).

## Run Params

An entry may declare `params`: tunables a run can override, such as the can's
mass or the controller's arm speed. A caller names one `<dimension>.<name>`
(`scenario.can_mass`) and sends only the values it changes; the sim agent checks
them, writes them to the run directory, and starts the launch with
`overrides_file` and `params_file` pointing at the read-only copies. No value is
ever on a command line.

```yaml
params:
  can_mass:
    label: Can mass              # required, the app's label
    description: ...             # optional help text
    type: float                  # float | int | bool | enum | array
    default: 0.35                # required, within its own type and bounds
    min: 0.05                    # float, int, array: required, with max
    max: 2.0
    step: 0.05                   # optional, > 0
    unit: kg                     # optional
    enum: [ptp, lin]             # enum only: the members
    items: float                 # array only: float | int
    length: 2                    # array only: the fixed length; min and max bound each element
    ordered: true                # array only: the elements ascend, as a [low, high] range does
    advanced: true               # optional: the advanced tier (default basic)
    backends: [mujoco]           # optional: refused on any other backend
    target: {kind: entry, path: objects.can.mass}
```

| Target | Shape | Applied |
|--------|-------|---------|
| `ros_param` | `node` (fully qualified, no wildcard), `param` (dotted) | Through the params file, appended last to each node |
| `entry` | `path`: dotted into the entry; a list item by its `name` | Merged into the entry before the world is built; the entry is checked again as the catalog checks it |
| `description` | `xacro_arg`, or `mjcf: {element, name, attribute}` | A xacro arg of the robot description, or one attribute of one MJCF element (`joint` `damping`; `position` `kp` or `forcerange`), patched after `fm_sim_models` compiles the model; an `mjcf` target needs `backends: [mujoco]` |

`fm_sim_bringup.params` checks every declaration on load, beyond the schema:
`min <= max` (integers for an int tunable), the default within its own type
and bounds, an `entry` target's path holding the default and ending on a
value rather than an identity field (`name`, `shape`, `static`, `object`,
`region`), no two `ros_param` targets of one entry setting one ROS param (or
one inside another), and no `ros_param` target on a launch-owned parameter
(`params.guarded`): a safety gate (`use_sim_time`, `sim_only`,
`engage_on_start`), a topic (`command`, `observation`, `enable`, `status`, any
`*_topic`), the robot description (`robot_description`, `_semantic`,
`_kinematics`; MoveIt's `robot_description_planning` joint limits stay
tunable), an identity or path param (`checkpoint`,
`policy_type`, `robot_profile`), or what this launch sets on its scenario and
camera nodes. A value must be the tunable's type (`true` is not a number; an
int is a float, and is written as one), within its bounds and, for an array,
its length and order; NaN and infinities are refused, and an int too large for
a float is out of bounds. `test/param_cases.json` holds the cases the launch and the
agent both check.

**The params file.** Every launch that owns tunables declares `params_file` and
appends it last to each node it starts (`fm_sim_backends.params_file`), so its
values win; it forwards it to the launches it includes. launch_ros writes a dict
parameter under `/**`, which rcl applies before node-keyed entries whatever the
order, so a node-keyed params file beats every launch dict. What the launch owns
is taken back with a node-keyed guard file after it: here the camera and
scenario nodes' parameters (`params.SCENARIO_OWNED`, `params.CAMERA_OWNED`),
and on mujoco, for both nodes of the engine process (the controller_manager
and `/mujoco_ros2_control_node`), `use_sim_time`, the description and every
plugin key but its rate. Each spawner gets the params file as a second
`--param-file`, so it beats the robot's controllers file. The
robot_state_publisher gets none: it owns no tunables. `SetParametersFromFile` is
never used: it applies before a node's own parameters.

**Testing a launch.** `fm_sim_backends.params_check` resolves a launch's `Node`
action through a real rclpy node, so a test sees rcl's precedence, not the
list's order. `assert_params_file_wins(node, context, {param: value})` and
`assert_guard_holds(...)` are its two asserts; another repo's launch test
imports them where `fm_sim_backends` is installed, or copies the file (it needs
only rclpy and launch_ros). `test/test_run_params.py` uses them on this launch.

| Entry | Tunables |
|-------|----------|
| `mujoco` | Object pose and camera render rates of the MuJoCo plugins (advanced) |
| `so101` | Control rate (mujoco), each JTC's end-velocity flag, and each joint's damping and actuator force limit in the MJCF (all advanced) |
| `axol` | Control rate (mujoco, advanced) |
| `openarm` | Control rate, each arm JTC's end-velocity flag, each camera's rate (at most the 10 Hz the engine renders and the bridge carries) and JPEG quality, each finger's stiffness and force limit (mujoco for all but the JTC flags; camera rates basic), and MoveIt Servo's joint speed (basic) and linear and rotational speeds for the `servo` launch role, on every backend, used only by a controller that activates `servo` |
| `can_on_table` | Can mass, friction and timeout (basic); can size (no taller than it is, since it spawns at a fixed height on the paper), spawn ranges, and the success tolerance, hold and at-rest speed, bounded so a success is still a can standing placed on the paper (advanced) |

Not tunable, by design: frames, joint and controller names, topics, paths,
safety gates, calibration, and the table and paper (`table_only` mirrors them).

## Scenarios

A scenario entry is backend-neutral: objects, regions and a success predicate,
with poses in the robot description's world frame (for OpenArm, the
`loader_openarm_v2` profile's `base_frame`, `follower_body_link0`, which sits on it). Each backend has a
generator that turns robot model + scenario + seed into the world its engine
loads; MuJoCo's is `fm_sim_scenarios.mujoco_world`, Gazebo's
`fm_sim_scenarios.gazebo_world` (see the [Gazebo backend](../fm_sim_backends/README.md#gazebo-world)), Isaac Sim's
`fm_sim_scenarios.isaac_stage` (see the [Isaac backend](../fm_sim_backends/README.md#isaac-stage)). The scenario node is the
same on every backend apart from how it reads and moves objects.

```mermaid
flowchart LR
    yaml["scenario yaml<br/>objects, regions, success"] --> spawn["spawn_poses(seed)"]
    mjcf["robot MJCF"] --> gen["mujoco_world"]
    spawn --> gen
    gen --> world["one MJCF world<br/>robot at mjcf_mount_xyz<br/>static geoms, free bodies"]
    world --> mrc["mujoco_ros2_control<br/>steps the physics"]
    mrc -->|"/mujoco/free_joint_states"| node["scenario node"]
    node -->|"set_free_joint_state"| mrc
    spawn --> node
    node --> objects["/sim/objects<br/>/sim/markers"]
    node --> outcome["/sim/outcome<br/>transient local"]
    reset["/sim/reset seed"] --> node
```

| Field | Meaning |
|-------|---------|
| `objects[]` | `name`, `shape` (`box` full extents, `cylinder` radius and height, `sphere` radius), `size`, `friction` (sliding), `color` (rgba), `pose` (`xyz`, optional `rpy`). Not `static`: also `mass`, and optional `spawn` ranges |
| `objects[].static` | Fixed to the world: a geom with no joint, never respawned |
| `objects[].spawn` | `x`, `y`, `z`, `yaw` as `[low, high]`: uniform offsets added to `pose`, drawn from `seed`: objects in yaml order, per object in x, y, z, yaw order |
| `regions[]` | `name`, `size` (full extents), `pose`, `color`: a box volume, drawn as a marker, never a body |
| `success` | `type: object_in_region`: `object`'s origin stays inside `region` grown by `tolerance` for `hold_s`, moving no faster than the optional `max_speed` (m/s); the episode fails at `timeout_s` |

Names match `^[A-Za-z][A-Za-z0-9_]*$` and are unique across objects and
regions; the success references and spawn ranges are checked on every load.

The MuJoCo world attaches the robot model unchanged (`<model>` asset and
`<attach>`), so its own compiler settings and relative mesh paths still resolve,
inside a frame at the robot entry's `mjcf_mount_xyz`. That puts the MuJoCo world
on the description's root frame (OpenArm's model is compiled from its description,
so the two origins already coincide and the mount is the default). Contact parameters: an object geom's friction
is its sliding coefficient with MuJoCo's default torsional and rolling terms; its
priority (2) is above the robot's collision geoms (1), so the object's friction
governs finger contacts; a free object uses `condim` 4 (torsional friction). The
world is written to a new `fm_sim_world_*` temporary directory per launch.

| Topic or service | Type | Behaviour |
|------------------|------|-----------|
| `/sim/objects` | `fm_sim_msgs/ObjectStates` | Every object, frame `world`, on each engine pose update (30 Hz of sim time on MuJoCo); static objects from their yaml pose |
| `/sim/markers` | `visualization_msgs/MarkerArray` | Namespace `objects` (one per object, by yaml index) and `regions` |
| `/sim/reset` | `fm_sim_msgs/srv/Reset` | Moves every free object to `spawn_poses(seed)` with zero velocity and starts a new episode under that seed; the old episode gives no verdict while objects move. `ok: false` with a message, keeping the old seed and episode, when the engine's service is not up yet, refuses, raises, or does not answer in 5 s |
| `/sim/outcome` | `fm_sim_msgs/Outcome` | Transient local, one per episode: success once the predicate holds, failure at the timeout. Episodes are timed in sim seconds; the first opens with the first `/clock` |

On MuJoCo the node reads poses from mujoco_ros2_control's
`FreeJointStatePublisherPlugin`, which `fm_sim_backends`' mujoco launch always
loads, and moves objects through `/mujoco_ros2_control_node/set_free_joint_state`.
The robot is not reset; a controller under test returns it to its start.

| Scenario | Objects | Success |
|----------|---------|---------|
| `empty` | none | none |
| `can_on_table` | `table` (top at 0.207), `paper` (A4), `can` (66 mm x 135 mm, 0.35 kg, spawned in a 60 mm x 50 mm box with any yaw) | `can` upright on the paper (`target`) for 2 s within 120 s |
| `table_only` | `can_on_table`'s `table` and `paper`, no can: the empty scene a background-difference detector subtracts | none |
| `tabletop_can`, `tabletop_block`, `tabletop_puck` | `can_on_table`'s `table`, an A5 `paper` whose centre a top-down release reaches, and one shared layout: green `can` (66 mm x 135 mm, 0.35 kg), red `block` (120 mm x 50 mm x 40 mm, 0.15 kg), blue `puck` (60 mm x 40 mm, 0.10 kg), each spawned in a 40 mm x 40 mm box, can and block with any yaw and puck at a fixed yaw | The named object on the paper (`target`, at its resting height) for 2 s within 120 s |
| `tabletop_simready` | The tabletop `table` and `paper`, with a SimReady can, canned food and boxed food in the three object slots. Isaac only: its entry records each asset's choice and bounds | none: it scores perception, with no closed-loop grasp |

`can_on_table` takes its geometry from fm-skills (`openarm_v2_skills_config`
`specs.yaml` and `fake_detections.yaml`): the table top, the can, and the can and
paper positions on the right arm's side. Its table starts 0.15 m ahead of the base
so the arms hanging at rest clear it.

An object may name a `usd` asset, relative to the assets directory, instead of
taking its geometry and physics from the entry. Only Isaac loads USD, so MuJoCo and
Gazebo refuse such a scenario. The assets are NVIDIA SimReady packages under
CC-BY-4.0; [`tools/README.md`](../tools/README.md) says how to fetch them and carries
their attribution.

## Cameras

A robot entry's optional `cameras_file` lists its cameras by role (`chest`,
`wrist_right`; any other role is refused): each role's optical frame in the
robot description, its `rate_hz` and the real camera's intrinsics. OpenArm's is
[`config/openarm/cameras.yaml`](config/openarm/cameras.yaml): the chest fisheye's
calibration, copied from fm-skills `openarm_v2_skills_config/config/camera_chest.yaml`
(the wrist camera has none of its own, and stands in with the same numbers, as
in fm-robot and fm-skills).

On MuJoCo, `run.launch.py` puts every role into the generated world and starts
one `fm_sim_cameras` `sim_camera` node per role. Simulators render pinhole
images and the cameras are fisheyes, so each MJCF camera is a pinhole wide
enough for its fisheye's edge midpoints (`fisheye.render_camera`), mounted where
the description puts the optical frame: `fm_sim_cameras.mount` walks the frame's
fixed joints up to the nearest link the MJCF also has as a body (the right
wrist camera rides the description's `right_wrist_cam` and the chest camera its
`chest_cam`), or to the description's root (a world camera). The engine's
CameraPlugin renders each at 10 Hz of sim time (mujoco_ros2_control 0.1.2's behaviour; `test_camera_launch.py`
checks it live); `sim_camera` warps the render into the real fisheye image,
JPEG-compresses it, and publishes:

| Topic | Type | Contents |
|-------|------|----------|
| `/cam_<role>/image_raw/compressed` | `sensor_msgs/CompressedImage` | `jpeg`, the real image size, the render's sim-time stamp, `frame_id` the role's optical frame |
| `/cam_<role>/camera_info` | `sensor_msgs/CameraInfo` | The real K, `equidistant` distortion (cv2.fisheye's k1..k4), same header |

`sim_camera` publishes at most at the role's `rate_hz`, judged on the stamps; the
10 Hz render bounds it below that (OpenArm's file says 15). Fisheye pixels whose
rays fall outside the render, towards the image corners (about 12 % of OpenArm's
frame, asserted in fm_sim_cameras' `test_fisheye.py`), are black. Gazebo and
Isaac render the same cameras file through their own camera sensors and the
shared relays; Gazebo renders no lens distortion (see fm_sim_backends). The cameras file is checked by
`fm_sim_cameras.cameras_file` for every backend, and each backend refuses what
it cannot render on top. Camera topics carry no integrity check beyond the sim
host's own DDS graph.

## Catalog

An entry is two installed files:

```text
share/ament_index/resource_index/fm_sim.<kind>/<name>   content: <package>/catalog/<kind>/<name>.yaml
share/<package>/catalog/<kind>/<name>.yaml              the entry
```

`<kind>` is `backends`, `robots`, `scenarios` or `controllers`. The `fm_sim.`
prefix keeps these resource types apart from any other package's. The entry yaml
must name itself `<name>` and match its definition in
[`schema/catalog.schema.json`](schema/catalog.schema.json), which is checked on
every load. Every path, the marker's content and each `{package, path}`, is
relative with no `..` segment and must stay inside that package's share as
written (symlinks are not followed, since `--symlink-install` links every share
file back to its source). An
`ament_python` package installs both through `data_files`; see this package's
`setup.py`.

| Kind | Registered here | Fields beyond `name`, `description`, `available`, optional `requires_packages` |
|------|-----------------|---------------------------------------------------|
| `backends` | `mujoco`, `gazebo`, `isaac` (by `fm_sim_backends`) | `launch`, `description_args`, optional `launch_args` |
| `robots` | `openarm`, `so101`, `axol` (each `supported_backends: [mujoco, gazebo, isaac]`) | `description_file`, `description_args`, `mjcf`, `controllers_file`, `controller_roles`, `robot_profile`, `supported_backends`, optional `mjcf_mount_xyz`, optional `mesh_roots` (mesh URI prefix → share directory, for URIs no package resolves), optional `cameras_file` (per role: optical frame, `rate_hz`, real intrinsics; openarm's is `config/openarm/cameras.yaml`), optional `launch_roles` |
| `scenarios` | `empty`, `can_on_table` | `objects`, optional `regions` and `success` (see [Scenarios](#scenarios)) |
| every kind | | optional `params` (see [Run Params](#run-params)) |
| `controllers` | `idle`, `teleop` | `activate`, `launch`, optional `launch_args` (fixed string arguments for that launch; the loader refuses a name `run.launch.py` sets itself, `catalog.CONTROLLER_LAUNCH_OWNED`, so an entry cannot overwrite `use_sim`, `robot_profile`, `scenario` or `params_file`), optional `family` (groups variants of one controller), optional `scenarios` (the only scenarios it runs on; absent means all), optional `control` (see [Control Descriptor](#control-descriptor)) |

### Control Descriptor

A controller entry's `control` names a `control.yaml` in its package: the
controller's pipeline as blocks and edges, shaped by the schema's
`control_descriptor`. The loader validates it, checks that every edge names a
block, and adds it to the entry as `control_descriptor`, so `catalog --json`
carries it for a pre-run preview. An entry yaml may not carry
`control_descriptor` itself. Each controller's own launch, in its own repo,
publishes the same descriptor as JSON, latched on `/control/descriptor`
(`std_msgs/String`, transient local), on the robot and in sim. JSON keeps the
consumer free of a YAML parser.

```yaml
name: fm_skills_classical          # the catalog entry's name
blocks:
  - id: perception
    kind: perception               # perception | state_machine | skill | motion
    variant: YOLO-seg
    live: /fm_skills/detections    # a topic whose rate shows the block running
    sim: {variant: ground truth}   # replaces variant or live in sim: a sim stand-in
  - id: task
    kind: state_machine
    variant: pick and place
    states: [IDLE, PICK, PLACE]
    source: {topic: /fm_skills/task_state, type: std_msgs/msg/String, field: data}
edges:
  - {from: perception, to: task}
```

`ros2 run fm_sim_bringup catalog --check-control FILE...` validates descriptor
files on their own, for a controller repo's CI.

The OpenArm entry runs the OpenArm v2 on fm-rob-01 as Anvil modelled it, under the
loader's names: fm-robot's `fm_control/urdf/openarm_loader.sim.urdf.xacro` over
fm-skills' `openarm_v2_anvil_description`, with `fm_sim_models`' `openarm_anvil`
MJCF compiled from the same description. That description is private, so the entry
names it in `requires_packages`: it runs in the private fm-sim-controllers image,
which installs it, and the public fm-sim image lists it as unavailable. The entry
owns the sim controllers file,
[`config/openarm/openarm.controllers.yaml`](config/openarm/openarm.controllers.yaml),
since fm-robot ships none. Its roles follow the `loader_openarm_v2` robot profile:

| Role | Controllers |
|------|-------------|
| `broadcasters` | `joint_state_broadcaster` |
| `arm_trajectory` | `follower_{l,r}_jtc` (JTC, the seven arm joints) |
| `arm_forward_position` | `follower_{l,r}_forward_position_controller` (the seven arm joints, then `finger_joint1`) |

As on the loader, the forward position controller carries the finger, so there is
no `gripper` role. The finger is prismatic, 0 closed to 0.05 m open, and
`finger_joint2` mimics it.

A controller entry's `activate` names controller roles or launch roles; any other
name fails the run. The schema checks each launch role's shape on load; whether its
`requires` are controller roles, and whether a name is in both maps, is checked
when a run resolves the robot. OpenArm has one launch role:

| Launch role | Requires | Starts |
|-------------|----------|--------|
| `servo` | `arm_trajectory` | fm-robot's `fm_control/launch/openarm_sim_servo.launch.py` with `sim_only:=true`, `command_type:=joint_jog`, `names:=loader`: sim-only MoveIt Servo on `/servo_node`, streaming to `follower_r_jtc`. That launch refuses to start unless ROS discovery is host-local (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, as the fm-sim agent sets) |

The SO-101 and Axol entries expand fm-robot's `so101.sim.urdf.xacro` and
`axol.sim.urdf.xacro` with no extra args, and own their controllers files under
`config/so101/` and `config/axol/`, named as fm_ros2's `fm_bringup` names them. No
robot profile describes either yet, so their `robot_profile` is `null`.

| Robot | `arm_trajectory` | `gripper` |
|-------|------------------|-----------|
| `so101` | `so101_arm_controller` | `so101_gripper_controller` |
| `axol` | `axol_{left,right}_arm_controller` | none (the grippers are fixed) |

Both also have `broadcasters: [joint_state_broadcaster]`. Their vendored URDFs
name meshes no package resolves (the SO-101's relative `assets/`, the Axol's
`package://assembly/`); each entry's `mesh_roots` maps that prefix to the
directory in fm_description's share. The Gazebo and Isaac backends read it.

| Controller | Activates | Launch |
|------------|-----------|--------|
| `idle` | nothing beyond the broadcasters | none |
| `teleop` | `arm_trajectory`, for an operator's teleop source such as fm-teleop's leader source, which runs outside this launch | none |

### `catalog`

```bash
ros2 run fm_sim_bringup catalog          # one line per entry
ros2 run fm_sim_bringup catalog --json   # the whole catalog
```

`--json` prints `{"schema_version": 1, "backends": [...], "robots": [...],
"scenarios": [...], "controllers": [...]}`, each entry being its yaml plus the
`package` that registered it. `schema/catalog.schema.json` (JSON Schema draft-07)
is the contract for that output.

## Tests

| Test | Checks |
|------|--------|
| `test_catalog.py` | Lookup, unknown and unavailable entries, `requires_packages` (a missing package lists the entry as unavailable and is named when it is refused), schema validation (`robot_profile` a name or `null`, launch role shape, controller `launch_args`), run-owned `launch_args` names refused, `catalog --json` against the schema, path confinement, robot roles against the controllers file, `can_on_table` geometry, malformed scenarios |
| `test_tabletop_scenarios.py` | The three tabletop entries share one layout; the named object is the success object; target at its resting height; worst-case spawns never overlap, start on the paper or as a success, and stay reachable and on the table; colours clearly apart |
| `test_run.py` | Description args, active and inactive controller sets for `idle` and `teleop` on every robot, launch roles and what they require, OpenArm's `servo` role, unknown values, unsupported robot-backend pairs, a launch controller on a robot with no profile, the controller launch's arguments, `seed` and `timeout_s` parsing, the world per backend, mesh roots, the stage arguments, when the scenario node runs, the cameras per backend and robot, OpenArm's camera mounts, and — with the mujoco binding — both cameras in the compiled world at the description's camera frames at a random arm pose. The OpenArm rows skip without its private description |
| `test_params.py` | Tunable declarations the loader refuses (default out of bounds or of the wrong type, `min` above `max`, an unknown target kind, a relative or wildcard node, a guarded param, an `mjcf` target without `backends`, an entry path naming nothing or not holding the default) and the shared value cases in `param_cases.json` |
| `test_run_params.py` | Overrides through the launch: an entry override changes the generated world (can mass), an override the entry would refuse is refused, a description override changes the compiled MJCF (joint damping) and a xacro arg; through rcl, the params file beats the scenario node's own parameters and the controllers file, and the guard keeps what the launch owns |
| `test_entry_params.py` | fm-sim's own tunables: every default is what the node, file or compiled model holds (OpenArm's MJCF ones skip without its description), none names what the launch owns, and overrides reach the world through the launch |
| `test_openarm_entry.py` | The OpenArm entry's description against its MJCF, built as `run.launch.py` builds them: joints, limits, prismatic fingers and actuators; a position servo on every `ros2_control` joint; the hand and camera frames at random poses; a clean spawn; the fingers' travel; the right hand closing on the `can_on_table` can between its pads and holding it, still and while the elbow lifts the hand. Skips without the private description |
| `test_openarm_hand.py` | fm-robot's `openarm.sim.urdf.xacro` (loader hand) against `fm_sim_models`' `openarm` model, which the entry ran before it moved to the Anvil description; both are retired once the Anvil model passes a live pick in sim |
| `test_run_launch_roles.py` | `run.launch.py`'s setup with SO-101 given OpenArm's `servo` role (SO-101 is public, so this runs in every image) and a controller that activates it: Servo waits on the active spawner and gets the run's description and `sim_only:=true`; a failed spawner shuts the run down; a missing launch file fails at setup; a controller's `launch_args` reach its launch, and every name the launch sets is refused in them |
| `test_scenario_launch.py` | Live: `so101` on `mujoco` with `can_on_table` and `idle` — the can on `/sim/objects`, `/sim/reset` moving it to the seed's spawn inside its ranges, a latched timeout failure on `/sim/outcome`, its bound judged in sim time. Needs the `so_arm` external and Mesa's EGL |
| `test_camera_launch.py` | Live: `openarm` on `mujoco` with `can_on_table` — both roles publish 1920x1080 JPEGs with the profile frames, sim-time stamps no more than 10 Hz apart, the real K and distortion on `camera_info`, and the can, projected through TF and the real chest intrinsics, lands on can-green pixels. Needs the private OpenArm description (skips without it) and Mesa's EGL |
| `test_gazebo_scenario_launch.py` | Live: `openarm` on `gazebo` with `can_on_table` and `idle` — the same object, reset and outcome checks as on MuJoCo; both cameras publishing JPEG on `/cam_<role>/image_raw/compressed` and a `camera_info` with the cameras file's intrinsics, its frame and zero distortion; after shutdown, no gz-sim server of the run left. Gazebo renders on software GL there, well below real time, so the episode's bound is judged in sim time and each reset is waited out before another. Linux only, and needs the private OpenArm description (skipped otherwise) |
| `test_run_launch.py` | Live: every robot on `mujoco`, `gazebo` (server-only) and `isaac` (no Isaac Sim running), `empty`, `idle` — broadcaster active, every other controller inactive, all joints in one hardware component on the simulators and the per-arm systems on Isaac. Each row runs on its own ROS domain and Gazebo partition; OpenArm's rows exist only where its private description is installed. Needs the MJCF and description externals, `topic_based_ros2_control` and Mesa's EGL |

## Build Type

`ament_python`. One console script, `catalog`. fm-robot's `fm_control` is an exec
dependency (the robot xacros); `fm-sim.repos` builds it
from source. `fm_sim_scenarios` is one too: the catalog checks scenarios with it and
`run.launch.py` generates worlds and starts its node.
