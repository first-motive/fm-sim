# fm_sim

Simulation layer for the fm_ros2 workspace. Groups the headless dev loop, the
per-engine backend hosts, and the MJCF model registry. Split-ready: this whole
group extracts cleanly into its own repo later.

There are two distinct simulation paths, and they do not include one another:

```
headless path   fm_sim_core/sim.launch.py
                  └─ sim_loop node → MuJoCo stepper → /joint_states
                  no controllers, no ros2_control — control / orchestration dev

stack path      fm_bringup/sim.launch.py        (the TUI's full sim)
                  ├─ controllers.launch.py
                  └─ fm_sim_backends/<engine>.launch.py   (mujoco | gazebo | isaac)
                       └─ hosts controller_manager inside the sim
```

The headless path is one node stepping physics — fast, no control stack, runs
native arm64 on the M5. The stack path is the full `ros2_control` graph, where the
chosen engine hosts the `controller_manager`. Both launch files are named
`sim.launch.py`; neither includes the other. See
[docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md#launch-dependency-graph) for the
full launch graph.

## Sub-Packages

| Package | Build | Role |
|---------|-------|------|
| [`fm_sim_core`](fm_sim_core/README.md) | ament_python | ROS-free MuJoCo stepper + headless `sim_loop` node |
| [`fm_sim_backends`](fm_sim_backends/README.md) | ament_python | Per-engine launch hosts (mujoco, gazebo, isaac) for the `ros2_control` stack |
| [`fm_sim_models`](fm_sim_models/README.md) | ament_python | MJCF model registry: robot key → MuJoCo model path |

## How the Pieces Connect

`fm_sim_backends/mujoco.launch.py` runs the `ros2_control` stack against a MuJoCo
model. `fm_bringup` looks the model path up in `fm_sim_models` and injects it into
the robot description as the `mujoco_model` xacro arg, so the path lives in one
place rather than being hardcoded per robot. `fm_sim_core` is independent of both —
it loads its own MJCF (or a built-in fallback) directly.

## Build Type

`ament_cmake` metapackage (exec-depends on the three sub-packages). The package
itself builds nothing; it ties the group together for a single install.
