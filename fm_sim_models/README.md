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

mjcf_path("openarm")   # -> /ws/external/openarm_mujoco/v2/openarm_bimanual.xml
mjcf_path("unknown")   # -> RuntimeError listing the registered keys
```

## Registered Models

| Key | MJCF | Status |
|-----|------|--------|
| `openarm` | `openarm_mujoco/v2/openarm_bimanual.xml` | Joint names match the description |
| `so101` | `so_arm/Simulation/SO101/so101_new_calib.xml` | Joint names match the system |
| `g1_d` | `unitree_mujoco/unitree_robots/g1/g1_29dof.xml` | Bipedal model — right-arm joints match; legs differ from the wheeled G1-D, so mujoco is wired-not-yet-validated |
| `axol` | `models/axol/axol.xml` (this package's share) | Almond Bot Axol, bimanual 2×7-DOF — mujoco validated; mock validated, real CAN deferred |

The first three paths are the in-container locations under `/ws/external`
(gitignored on the host, mounted at `/ws` in the dev container). `axol` is the
exception: it ships no upstream MJCF, so First Motive authored one (compiled from
`axol.urdf`) and committed it into this package's share at `models/axol/axol.xml`
(with its meshes). `models.py` resolves it from the package share, not
`/ws/external`.

## Build Type

`ament_python`. A registry module, no nodes and no console scripts.
