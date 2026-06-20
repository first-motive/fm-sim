# Diagrams

Architecture diagrams for the simulation layer, authored in [d2](https://d2lang.com).
Each `.d2` file is the source of truth; the matching `.svg` is a generated
artifact referenced by the docs. Edit the `.d2`, then re-render.

## Render

```bash
./render.sh          # renders every *.d2 to *.svg with the brand font
```

Needs `d2` on `PATH`. The font ships in [`fonts/`](fonts/), so rendering is
self-contained. The script passes the font explicitly:

```bash
d2 --layout elk --font-regular fonts/GeistMono-VF.ttf \
   --font-bold fonts/GeistMono-VF.ttf --font-italic fonts/GeistMono-VF.ttf in.d2 out.svg
```

## Palette + Grammar

Brand palette mirrors firstmotive.ai, defined once in [`styles.d2`](styles.d2),
imported with `...@styles`. Every component is a stacked block — role (plum) /
package (lavender) / artifact (cream); node graphs use `node` (plum box) + `topic`
(cream pill). A block expanded elsewhere uses `class: zoom` (dashed border). Full
token table and block grammar:
[fm-robot/docs/diagrams](https://github.com/first-motive/fm-robot/blob/main/docs/diagrams/README.md).

## Diagrams

```
backends   sim_backend → {mujoco · gazebo · isaac} launch hosts → in-engine controller_manager
sim_loop   fm_sim_core headless dev loop — MujocoStepper → sim_loop node → /joint_states
```

`backends` is included by `fm_bringup/sim.launch.py` (in
[`fm-app`](https://github.com/first-motive/fm-app)); this layer provides the hosts
it selects. See [ARCHITECTURE.md](../ARCHITECTURE.md).
