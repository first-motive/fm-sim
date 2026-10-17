# fm_sim_ee_follower

Follows a commanded tool pose with damped-least-squares (DLS) IK. The `ee_follower`
node reads `/commanded_ee_right` and streams joint positions to the right arm's forward
position controller.

## Why

On the real OpenArm, the Anvil loader runs the IK: a controller such as
`classical_control` publishes a tool pose on `/commanded_ee_right`, and the loader turns
it into joint commands. The sim has no loader, so this node does that job. Any controller
that streams poses then drives the sim the same way it drives the robot.

## How

![ee_follower](../docs/diagrams/ee_follower.svg)

Source: [`docs/diagrams/ee_follower.d2`](../docs/diagrams/ee_follower.d2).

At each tick, the node takes one bounded step from the measured joint positions toward
the latest commanded pose:

```
e  = [p_target - p, rotvec(R_target R^T)]          base frame
dq = J^T (J J^T + damping^2 I)^-1 (gain * e)
dq is scaled down so no joint moves more than its per-step bound
q  = clip(q + dq, URDF joint limits), each joint's change still capped at its bound
```

Damping keeps `dq` bounded near a singularity, where the plain pseudo-inverse blows up.
Each joint's step bound is `max_joint_step`, or its speed bound times the tick period if
that is smaller. The speed bound is the URDF velocity limit, or the smaller of that and the
joint's entry in the `joint_speed_cap` parameter. A joint with neither (urdfdom reads an
omitted velocity as 0) has no bound, and the node refuses the chain rather than step it
without one. When `max_joint_step × rate_hz` is above a joint's speed bound, the node logs
a warning naming the joints that step slower. The cap after the limit clip means a joint
that starts outside its limits walks back in at its bound instead of jumping.

| Module | Role |
|--------|------|
| `dls.py` | The DLS step and the pose error. ROS-free; takes FK and the Jacobian as functions |
| `kinematics.py` | Builds a KDL chain from the URDF, from `base_frame` to `ee_frame`, with its joint limits and step bounds |
| `gate.py` | The motion gate: input times and flags to a blocked reason or None. ROS-free |
| `profile.py` | Reads `base_frame`, the arm's `joints`, `arm_controller` and `ee_frame` from a shipped robot profile |
| `follower_node.py` | ROS wiring, message checks and the tick loop |

The kinematics use PyKDL (`python_orocos_kdl_vendor`) and `urdf_parser_py`
(`urdfdom_py`). Both are already in the `fm-sim:jazzy` image. Jazzy has no Python
`kdl_parser`, so `kinematics.py` builds the chain from the URDF itself, the same way
`kdl_parser` does.

### Safety Gate

A tick sends nothing, and the forward position controller holds its last command, when:

- No usable `/robot_description` has been received (see below).
- No `/joint_states` message with this arm's joints has arrived, or its age is outside
  `[0, max_joint_state_age]`.
- No command has arrived, or its age is outside `[0, max_command_age]`.
- The latest command or joint state has a non-finite value. Such a message replaces the
  last good one, and the DLS step returns no step for it.

An input's age is the node's clock minus its `header.stamp`. An input with a zero stamp
(an unstamped `JointState`, say) is aged from its arrival on the node's clock instead.
A negative age, from a future stamp or a clock that jumped back, counts as stale. When the
node's clock goes backward between ticks, as on a sim reset, it also drops the stored
command and joint state.

The node builds its chain from the first `/robot_description` it can use. A description
over 2 MiB, one that fails to parse, one with no chain from `base_frame` to `ee_frame`, a
revolute or prismatic joint without a `<limit>` or with `lower` not below `upper`
(urdfdom reads an omitted `lower` or `upper` as 0, so a missing range reads as 0..0 and
would lock the joint), a joint with no speed bound, or chain
joints that differ from the profile's `joints` is logged and leaves the node without
kinematics. Once a description is accepted, a later one that differs from it is rejected
and logged; the chain does not change under a running follower.

A command is rejected when its `header.frame_id` is not the profile's `base_frame`, or
its quaternion is not finite or has near-zero norm. A rejected command also clears the
last good one, so once the sender publishes bad data the arm stops instead of finishing
its move toward an older target. A `/joint_states` message without all of this arm's
joints is ignored, since another publisher may own it.

## Usage

```bash
ros2 run fm_sim_ee_follower ee_follower --ros-args -p use_sim_time:=true
```

`robot_profile` takes a shipped profile name only (the yaml files in `profiles/`), never a
path. The arm's forward position controller (for `fm_openarm_v2`,
`openarm_right_forward_position_controller`) must be active. Its trajectory controller
twin claims the same joints, so that one must be inactive.

## Node

### `ee_follower`

**Subscribes**

| Topic | Type | Description |
|-------|------|-------------|
| `/commanded_ee_right` | `geometry_msgs/PoseStamped` | Target tool pose, `frame_id` = profile `base_frame` |
| `/joint_states` | `sensor_msgs/JointState` | Measured joint positions |
| `/robot_description` | `std_msgs/String` | URDF, transient local |

**Publishes**

| Topic | Type | Description |
|-------|------|-------------|
| `/<arm_controller>/commands` | `std_msgs/Float64MultiArray` | Joint positions in profile `joints` order |

**Parameters** (all read-only; numeric ranges are enforced at startup)

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `robot_profile` | string | `fm_openarm_v2` | | Shipped profile name |
| `rate_hz` | double | `100.0` | 1 to 1000 | Step and command rate (Hz) |
| `damping` | double | `0.05` | 1e-4 to 1 | DLS damping lambda |
| `gain` | double | `0.5` | 0.01 to 1 | Fraction of the pose error closed per step |
| `max_joint_step` | double | `0.02` | 1e-4 to 0.5 | Largest joint change per step (rad) |
| `joint_speed_cap` | double[] | unset | each > 0 | Speed cap per joint (rad/s) in profile `joints` order; unset or empty uses the URDF velocity limits alone. A wrong length or a non-positive entry stops startup |
| `max_joint_state_age` | double | `0.2` | 0.01 to 10 | Oldest `/joint_states` a step may start from (s) |
| `max_command_age` | double | `0.5` | 0.01 to 10 | Oldest command the node still follows (s) |

## Robot Profile

`profiles/fm_openarm_v2.yaml` copies the profile that `anvil_robot_profiles`,
`openarm_v2_skills_config` and `fm_teleop_policy` also ship. `anvil_robot_profiles`,
`fm_skills_task` and `fm_teleop_policy` each carry a loader for it, and `profile.py` is
one more. Both stay local for now; one shared profile package should replace every copy
and every loader ([fm-robot#49](https://github.com/first-motive/fm-robot/issues/49)).

## Tests

```bash
uv run --no-project --with numpy --with pytest --with pyyaml \
  python -m pytest test/test_dls.py test/test_gate.py test/test_profile.py   # no ROS
colcon test --packages-select fm_sim_ee_follower                              # in fm-sim:jazzy
```

`test_dls.py` checks the DLS step against a numpy toy arm: it converges on a reachable
target, respects the step bound and the joint limits, stays bounded at a singularity, and
rejects non-finite input. `test_gate.py` checks the gate decision directly.
`test_kinematics.py` checks the KDL chain and its step bounds against a fixture URDF.
`test_follower_node.py` runs the shipped node over real topics as a wiring check of each
gate, with a fresh, valid input case as the control, and drives `/clock` for the clock
jump, waiting on sim time rather than wall time. Under ROS, `test/conftest.py` gives the
tests a ROS domain of their own, so packages tested in parallel do not see each other's
topics.

## Build Type

`ament_python`.
