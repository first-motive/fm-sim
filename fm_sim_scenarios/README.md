# fm_sim_scenarios

Backend-neutral scenarios for the sim run: where a scenario's objects spawn for a
seed, when an episode succeeds, the per-backend generators that build a world
from a robot model and a scenario, and the scenario node that serves the
scenario on the sim host. The scenario entries themselves, and their schema, live
in the catalog ([`fm_sim_bringup`](../fm_sim_bringup/README.md#scenarios)).

## Modules

| Module | Role |
|--------|------|
| `scenario` | ROS-free: `check` (finite numbers, unique names, spawn ranges, success references), `spawn_poses(scenario, seed)` (offsets drawn objects in yaml order, in x, y, z, yaw order per object), `static_poses`, `in_region`, and `EpisodeMonitor`, which judges the `object_in_region` predicate one episode at a time |
| `mujoco_world` | `generate` / `write`: robot MJCF + scenario + seed (+ cameras) → one MJCF world, the robot attached at its mount, static objects as world geoms, the others as free bodies. A world camera goes in the world; cameras on robot bodies go into a copy of the robot model written beside the world, which drops the model's own cameras and makes its asset directories absolute (a model with `<include>` is refused) |
| `gazebo_world` | Gazebo Harmonic generator: `write` (scenario + seed → world SDF: physics, the Sensors system, ground, light, static models and free models with mass, inertia, `mu`/`mu2` friction and a PosePublisher; `WORLD_GENERATORS['gazebo']`) and `write_robot` (+ robot description, cameras, mesh roots → the URDF Gazebo spawns, with absolute mesh URIs and a pinhole camera sensor per camera, and the ros_gz_bridge config) |
| `gazebo_objects` | The scenario node's Gazebo backend: poses from `/model/<object>/pose` (PoseStamped, world frame `fm_sim`), paced by the first free object and leaving out a pose more than 0.1 s of sim time from it; non-finite, non-world or future-stamped poses dropped. A move is all or nothing: every `/world/fm_sim/set_pose` at once under one deadline, then each object's first pose after the move within 5 mm of its target; on any failure every object goes back to its pre-move pose, and a reply after its move gave up is ignored. Gazebo keeps a teleported model's velocity |
| `isaac_stage` | Isaac Sim 4.5 generator, pure Python: `write` (scenario + seed → the stage's objects with mass, friction and seeded spawn; `WORLD_GENERATORS['isaac']`) and `write_stage` (+ robot URDF, cameras → `stage.json` with the URDF import settings, cameras on their optical frames with USD lenses from the real intrinsics, and the ROS 2 bridge OmniGraph). `fm_sim_backends/isaac/fm_sim_isaac.py` replays it inside Isaac Sim |
| `isaac_objects` | The scenario node's Isaac backend: poses from `/isaac/object_poses` (tf2_msgs/TFMessage from the bridge graph), teleports on `/isaac/object_pose_commands`, acknowledged once a pose update stamped strictly after the command puts every object within 5 mm of its target |
| `scenario_node` | The `scenario_node` executable: `/sim/objects`, `/sim/markers`, `/sim/reset`, `/sim/outcome` |

`run.launch.py` calls the generator and starts the node; nothing here reads the
catalog, so the dependency runs one way (`fm_sim_bringup` → `fm_sim_scenarios`).

## Scenario Node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scenario` | — | The catalog entry as JSON |
| `seed` | `0` | The seed the world was generated with; `/sim/outcome` carries it until a reset |
| `backend` | — | How objects are read and moved: `mujoco`, `gazebo` or `isaac`. Any other value fails at start |
| `timeout_s` | `0.0` | Episode timeout; 0 keeps the scenario's `success.timeout_s` |
| `frame_id` | `world` | Frame of `/sim/objects` and `/sim/markers` |

Run it with `use_sim_time`. On MuJoCo, poses come from mujoco_ros2_control's
`FreeJointStatePublisherPlugin` on `/mujoco/free_joint_states` (loaded by
`fm_sim_backends`' mujoco launch) and a reset calls
`/mujoco_ros2_control_node/set_free_joint_state`. The node spins on a
multi-threaded executor: the reset service and that client have their own
callback groups, so pose updates keep publishing while a reset waits on the
engine, and a lock guards the episode state only. A pose that is missing or not
finite for a free object leaves it out of `/sim/objects` and never counts toward
success. Poses must be in the MuJoCo world frame: the plugin's default stamps
`frame_id` `''` (mujoco_ros2_control 0.1.2), and `''` or `world` is accepted;
any other frame is skipped and logged. A reset the engine refuses, or that
raises, answers `ok: false` and keeps the old seed and episode.

A backend joins by adding a class to `scenario_node.BACKENDS` (subscribe to its
poses, move objects) and a generator module (`world_path`, `body_names`, `write`)
to `fm_sim_bringup.run.WORLD_GENERATORS`.

## Tests

The fixture is the `can_on_table` entry `fm_sim_bringup` ships, read from the source tree, plus a test-only free sphere with no spawn ranges.

| Test | Checks |
|------|--------|
| `test_scenario.py` | Seeded spawns (deterministic, in range, uint32 seeds), `check` failures, quaternions, `in_region` with tolerance and yaw, the monitor's hold, timeout and one verdict per episode |
| `test_mujoco_world.py` | Same bytes per seed; cameras in the world and the robot copy (own camera dropped, meshdir absolute, missing body and `<include>` refused), compiled by the engine build and, with the binding, looking along their optical axes as the link turns; the world compiles with mujoco_vendor's `compile` (the engine mujoco_ros2_control runs); with the mujoco Python binding, the robot mounted unchanged, objects at their spawn, contact parameters, and objects settling on the table. The binding tests skip when the wheel (`requirements-sim.txt`) is absent |
| `test_gazebo_world.py` | Gazebo generator: same bytes per seed, free objects at their spawn with a PosePublisher, static models, mass, inertia and friction, the systems loaded, `gz sdf -k` on the world; mesh URIs resolved or refused, a camera on a missing frame refused, and through `gz sdf -p` the camera sensor's pose once the parser folds the optical frame into its parent, and its intrinsics; zero-distortion cameras; the bridge config; object names checked; the robot half written only in a run directory and never over a file or symlink |
| `test_gazebo_objects.py` | The node on `backend: gazebo` against a fake bridge: poses reach `/sim/objects`, a stale pose and a non-world frame left out, non-finite and future-stamped poses dropped, a reset moving each free object through `set_pose`, a refused or unserved `set_pose`, a second object refused restoring the first, a late reply ignored, a move that does not land undone, a static-only scenario on its timer |
| `test_isaac_stage.py` | Isaac stage: seeded object specs and physics, package URI resolution, joint topics read from the description, camera mounts through fixed joints, USD lens round trip, camera file checks, a bridge graph that wires only nodes it creates and carries the contract topics; with fm_control installed, the real OpenArm description and the shipped cameras file |
| `test_isaac_objects.py` | The node on `backend: isaac` against a fake stage: poses reach `/sim/objects`, a reset teleports through the command topic, a stage that never moves, a stage that starts listening late, no stage listening, a non-world frame, and a stale pose not acknowledging a move |
| `test_scenario_node.py` | The node on a fake backend: topic contents, a slow reset that does not stall `/sim/objects`, the latched outcome per episode and seed, success, a failed or raising reset keeping the old seed, non-finite poses left out, the MuJoCo adapter's world-frame guard |

## Build Type

`ament_python`. Exec-depends on `fm_sim_msgs`, `mujoco_ros2_control_msgs`, `tf2_msgs`,
`ros_gz_interfaces`, `geometry_msgs`, `visualization_msgs`, `python3-yaml` and
`rclpy`; tests use `mujoco_vendor` and sdformat's `gz sdf`.
