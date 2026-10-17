# fm_sim_msgs

Interfaces a scenario exposes on the sim host. The scenario node in
[`fm_sim_scenarios`](../fm_sim_scenarios/README.md) serves them; a controller's
ground-truth perception and the desktop viewer read them.

| Interface | Topic or service | Description |
|-----------|------------------|-------------|
| `msg/ObjectStates` | `/sim/objects` | Every scenario object's pose, as a list of `msg/ObjectState` (`name`, `pose`) under one header |
| `msg/Outcome` | `/sim/outcome` | How an episode ended: `scenario`, `seed`, `success`, `reason`, `elapsed_s`. Transient local, one per episode |
| `srv/Reset` | `/sim/reset` | Takes a `seed`, respawns the objects and starts a new episode; answers `ok` and `message` |

The definitions are fixed by the simulation contracts; change them only with the
consumers (fm-skills' ground-truth perception, the desktop decoders).

## Build Type

`ament_cmake` with `rosidl_default_generators`. Depends on `geometry_msgs` and
`std_msgs`.
