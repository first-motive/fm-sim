# fm_sim_models

Simulation model registry: maps each robot to the MuJoCo (MJCF) model the `mujoco`
backend loads. Single source of truth for those paths.

## Role

```
robot key  →  fm_sim_models.mjcf_path(robot_key)  →  MJCF path
                passed as the `mujoco_model` xacro arg of fm-robot's
                fm_control/urdf/<robot>.sim.urdf.xacro
```

Whoever expands the [`fm-robot`](https://github.com/first-motive/fm-robot) sim
xacro with `sim_backend:=mujoco` looks a path up here and passes it as
`mujoco_model`, so the MJCF path lives in one place instead of being hardcoded in
each robot's `*.sim.urdf.xacro`.

## Usage

```python
from fm_sim_models.models import mjcf_path

mjcf_path("openarm_anvil")  # -> ~/.cache/fm_sim_models/openarm_anvil_<hash>.xml
mjcf_path("openarm")   # -> ~/.cache/fm_sim_models/openarm_loader_<hash>.xml
mjcf_path("openarm_pinch")  # -> /ws/external/openarm_mujoco/v2/openarm_bimanual.xml
mjcf_path("unknown")   # -> RuntimeError listing the registered keys
```

## Registered Models

| Key | MJCF | Status |
|-----|------|--------|
| `openarm_anvil` | Compiled at run time from the installed `openarm_v2_anvil_description` (see below) | The `openarm` catalog robot; matches fm-robot's `openarm_loader.sim.urdf.xacro`. Private: raises naming the package where it is not installed |
| `openarm` | `openarm_mujoco/v2/openarm_bimanual.xml` patched to the loader hand (see below) | Matches fm-robot's `hand:=loader` description; no catalog entry runs it now |
| `openarm_pinch` | `openarm_mujoco/v2/openarm_bimanual.xml` | Stock v2.0 pinch gripper; matches `hand:=pinch` |
| `so101` | `so_arm/Simulation/SO101/so101_new_calib.xml` | Joint names match the system |
| `g1_d` | `unitree_mujoco/unitree_robots/g1/g1_29dof.xml` | Bipedal model — right-arm joints match; legs differ from the wheeled G1-D, so mujoco is wired-not-yet-validated |
| `axol` | `models/axol/axol.xml` (this package's share) | Almond Bot Axol, bimanual 2×7-DOF — mujoco validated; mock validated, real CAN deferred |

The vendored paths are the in-container locations under `/ws/external`
(gitignored on the host, mounted at `/ws` in the dev container). `axol` is the
exception: it ships no upstream MJCF, so First Motive authored one (compiled from
`axol.urdf`) and committed it into this package's share at `models/axol/axol.xml`
(with its meshes). `models.py` resolves it from the package share, not
`/ws/external`.

## OpenArm Anvil Model

fm-sim's `openarm` robot is the OpenArm v2 on fm-rob-01 as Anvil modelled it, under
the loader's names (`follower_l_*`, `follower_r_*`). Its description,
`openarm_v2_anvil_description`, is private: it is installed in the private
fm-sim-controllers image and never in the public fm-sim image, and this package
holds none of its geometry. `mjcf_path("openarm_anvil")`
(`fm_sim_models/openarm_anvil.py`) builds the model where the description is
installed:

1. Reads `urdf/robot.urdf` from the package's share and points every
   `package://` mesh at its installed file, refusing one that leaves its share or
   does not exist.
2. Lets MuJoCo compile the URDF, keeping every link a body (`fusestatic="false"`,
   for the camera mounts and TF), the visual meshes and the full mesh paths.
   MuJoCo turns each finger's `<mimic>` into the equality that drives
   `finger_joint2` from `finger_joint1`; the patch checks it is there.
3. Patches in what a URDF cannot say:

| Part | Patched model |
|------|---------------|
| Arm servos | a position actuator on each of the 14 arm joints over the URDF range, force capped at the URDF effort, with openarm_mujoco v2's gains |
| Arm joints | openarm_mujoco v2's damping, armature and friction loss per motor class |
| Finger servo | a position actuator on each `finger_joint1`, 0..0.05 m, with the loader hand's gains and 30 N cap (`openarm_loader`) |
| Finger mimic | the `finger_joint2` equality made stiff (`solref 0.005 1`, `solimp 0.95 0.99 0.001`, as openarm_mujoco v2's), so the follower finger does not give way when the hand squeezes an object |
| Contacts | excluded between each hand's two fingers and between `link5` and `link7` |
| Option | `noslip_iterations="5"`, as for the loader hand |

MuJoCo writes the compiled model to six significant digits, so the joint ranges are
set again from the URDF at full precision. The result is cached like the loader
hand below, as `openarm_anvil_<hash>.xml`. Without the description, `mjcf_path`
raises naming the package. Closed on the `can_on_table` can, the fingers close on
it at the pads and hold it, still and through a lift. A can pushed deep into the
hand, against the palm end of the fingers, stops them before they reach the pads,
so a grasp must take it at the pads.

## OpenArm Loader Hand

No catalog entry runs this model now. It stays, with fm_sim_bringup's
`test_openarm_hand.py`, until the Anvil model passes a live pick in sim; then the `openarm` and `openarm_pinch` keys, `openarm_loader.py` and that
test are retired together.

fm-robot's `openarm.sim.urdf.xacro` builds the OpenArm fm-rob-01's loader runs by
default (`hand:=loader`): the v2.0 arm with the loader's joint limits and Enactic's
v1.0 parallel-jaw hand. `openarm_mujoco` ships that hand only on its v1 arm, so
`mjcf_path("openarm")` parses the vendored v2 model and patches it to match the
URDF (`fm_sim_models/openarm_loader.py`):

| Part | Patched model |
|------|---------------|
| Limits | the loader's on all seven joints of each arm, the joint `range` and its actuator `ctrlrange`: joint1 ±2.3562 on both arms, joint6 -1.22173..0.785398 (right) and -0.785398..1.22173 (left), the rest v2.0's to rounding |
| Wrist | the pinch base meshes and Enactic's wrist camera on `openarm_<side>_ee_base_link` give way to the loader's link7 mesh and inertial (openarm_mujoco v1 meshes), as fm-robot's description has them |
| Hand | body `openarm_<side>_hand` 100.1 mm out from `ee_base_link`, flipped about x, with site `openarm_<side>_grasp_frame` 0.08 m beyond |
| Fingers | bodies `openarm_<side>_right_finger` / `_left_finger` on slide joints `finger_joint1` / `finger_joint2` over 0..0.05 m; the vendored equality makes joint2 mimic joint1 |
| Finger actuator | position on `finger_joint1`, `ctrlrange` 0..0.05 m, kp 500 N/m, kv 10 N·s/m, ±30 N (the most a finger ever squeezes) |
| Contacts | excluded between the two fingers (their meshes overlap 3.5 mm at 0) and between `link5` and `ee_base_link` (the wrist's convex hull overlaps link5's within the loader's limits: 5.5 mm at the joint6 limit, 36 mm with joint7 at its limit too), so those pairs never collide |
| Option | `noslip_iterations="5"`, so a squeezed can does not creep down the fingers; it costs every contact in the world the extra iterations |

Body names are the URDF link names, so camera mounts and TF line up. Closed on the
`can_on_table` can (66 mm), the fingers stop at about 0.034 m; the real hand reads
about 0.0185 m on the same can, so its jaw geometry differs from the URDF's.

The patched model is re-derived from the vendored file on every call and written
once per content hash (`fm_sim_models/cache.py`) to
`$XDG_CACHE_HOME/fm_sim_models/openarm_loader_<hash>.xml`
(default `~/.cache`), with absolute mesh paths; a cached file that differs is
rewritten. Old hashes stay, about 40 kB per patch or upstream change, so a running
sim never loses its model. An upstream model the patch does not fit (a missing
element, an asset directory other than `meshdir`, no v1 meshes) raises a
`RuntimeError` naming it. The world generator copies the model's `<option>` into
the scenario world, which MuJoCo's attach would otherwise drop.

## Build Type

`ament_python`. A registry module, no nodes and no console scripts.
