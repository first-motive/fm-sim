# fm_sim_models

Simulation model registry: maps each robot to the MuJoCo (MJCF) model the `mujoco`
backend loads. Single source of truth for those paths.

## Role

```
fm_bringup robot registry  →  fm_sim_models.mjcf_path(robot_key)  →  MJCF path
                              injected as the `mujoco_model` xacro arg
```

`fm_bringup` looks a path up here whenever `sim_backend == "mujoco"` and injects it
into the description, so the MJCF path lives in one place instead of being hardcoded
in each robot's `*.sim.urdf.xacro`. Keys match `fm_bringup`'s `RobotSpec.key`.

## Usage

```python
from fm_sim_models.models import mjcf_path

mjcf_path("openarm")   # -> /ws/external/openarm_mujoco/v2/openarm_bimanual.xml
mjcf_path("unknown")   # -> RuntimeError listing the registered keys
```

## Registered Models

| Key | MJCF | Status |
|-----|------|--------|
| `openarm` | `openarm_mujoco/v2/openarm_bimanual.xml` | Joint names match the description |
| `so101` | `so_arm/Simulation/SO101/so101_new_calib.xml` | Joint names match the system |
| `g1_d` | `unitree_mujoco/unitree_robots/g1/g1_29dof.xml` | Bipedal model — right-arm joints match; legs differ from the wheeled G1-D, so mujoco is wired-not-yet-validated |

Paths are the in-container locations under `/ws/external` (gitignored on the host,
mounted at `/ws` in the dev container).

## Build Type

`ament_python`. A registry module, no nodes and no console scripts.
