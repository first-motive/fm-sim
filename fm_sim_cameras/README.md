# fm_sim_cameras

Simulated robot cameras published the way the real ones are. A simulator renders
a pinhole image; the OpenArm cameras are fisheyes. This package works out the
pinhole each camera must render, where on the robot it is mounted, and runs the
node that turns the render into the real camera's topics. `fm_sim_bringup`'s
`run.launch.py` wires it up; see [Cameras](../fm_sim_bringup/README.md#cameras).

## Modules

| Module | Role |
|--------|------|
| `cameras_file` | The one loader and CameraInfo builder every backend uses. `load` / `check` give `{role: camera}` as plain data, refusing roles other than `chest`, `waist`, `wrist_right` and `wrist_left`, bad frame names, a `rate_hz` that is not a positive number, an image above 1920x1080 without `allow_large_image: true`, and distortion other than `plumb_bob` (5 coefficients) or `equidistant` (4). ROS-free (yaml and plain dicts). Backends refuse what they cannot render on top: the MuJoCo warp only `equidistant` (`fisheye.from_intrinsics`), Isaac a `rate_hz` that does not divide its 60 Hz render (`isaac_stage.check_cameras`) |
| `fisheye` | ROS-free. `from_intrinsics` takes a checked camera's intrinsics and refuses any model but `equidistant`. `render_camera` gives the pinhole render, as wide as the fisheye image, whose field reaches the rays through the fisheye's edge-midpoint pixels. `warp_maps` gives the `cv2.remap` maps from that render to the fisheye image, by inverting cv2.fisheye's model |
| `depth` | ROS-free. `nearest_maps` rounds `warp_maps`' maps for nearest-neighbour use; `warp` moves a pinhole z-depth render (`32FC1` metres) into the fisheye image unchanged, as both share one optical frame; `to_millimetres` gives uint16 millimetres, 0 for NaN, infinite, non-positive or past 10 m (MuJoCo writes the far plane, tens of metres, where a ray hits nothing); `encode` / `decode` the 16-bit PNG |
| `mount` | ROS-free. `fixed_mount` walks a URDF frame's fixed joints up to the nearest link the simulator has as a body, or to the root, and returns the frame's pose there; a moving joint on the way is refused |
| `camera_info` | `camera_info(intrinsics)`: the real CameraInfo of a checked camera, shared with Isaac's camera_info node in fm_sim_backends |
| `camera_node` | The `sim_camera` executable |

The render follows the convention of mujoco_ros2_control's CameraPlugin: square
pixels, the principal point at the centre, focal length `height / 2 / tan(fovy / 2)`.
Fisheye pixels whose rays fall outside the render, towards the corners, are black:
about 12 % of OpenArm's frame (asserted in `test_fisheye.py`).

## sim_camera

Reads one role's render and publishes `/cam_<role>/image_raw/compressed`
(`jpeg`, the real image size) and `/cam_<role>/camera_info` (the real K and
distortion), both stamped with the render's stamp and the role's optical frame.
It publishes at most at the role's `rate_hz`, judged on the stamps: a render less
than three quarters of a period after the last one published is skipped. It
skips, with a throttled warning, a render of the wrong size or encoding, one
whose `step` or buffer length does not fit its size, one stamped more than a
period ahead of the sim clock (so a bad stamp cannot hold back thinning), and
any render before the sim clock runs. With no render for 5 periods of sim time
it warns that renders stopped. All topics are reliable with depth 1; the render
subscription has to be, as best effort loses most multi-megabyte renders.

It also reads the role's depth render and publishes `/cam_<role>/depth/compressed`:
a `sensor_msgs/CompressedImage` with `format` `16UC1; png` (image_transport's
`compressed` convention, a plain PNG, not `compressedDepth`'s 12-byte header),
holding z-depth along the optical axis in millimetres at the real image size, 0
where there is no depth (the black corners, nothing hit, or past 10 m). The warp
is nearest neighbour through the colour warp's maps, so no depth is invented
across an edge. Depth follows colour: a depth render is published only once the
colour render with the same stamp has been, with that header (stamp and optical
frame). One whose colour was thinned or skipped is dropped, as is one older than
the last colour published when it arrives; only the newest unpaired one is held,
and a stall warning releases it. A depth render is skipped with a throttled
warning on the same grounds as a colour one: the wrong size or encoding (not
`32FC1`), a buffer that does not fit, a stamp more than a period ahead of the sim
clock, or a sim clock not yet running.

The topics carry no integrity check beyond the host's own DDS graph: anything
that can publish on the sim host's ROS domain can feed a render or replace an
image (as for every sim topic; see the Zenoh trust boundary in the simulation
contracts §3).

| Parameter | Default | Description |
|-----------|---------|-------------|
| `role` | — | `chest`, `waist`, `wrist_right` or `wrist_left`; anything else exits with an error |
| `cameras_file` | — | The robot's cameras file |
| `render_topic` | — | The simulator's `rgb8` render of this camera, at `render_camera`'s size; other sizes or encodings are skipped with a warning |
| `depth_render_topic` | — | The simulator's `32FC1` z-depth render (metres) of the same camera, same size |
| `jpeg_quality` | `90` | 1 to 100 |
| `rate_hz` | `0.0` | Overrides the cameras file's `rate_hz` when above 0, up to 120 |

## Tests

| Test | Checks |
|------|--------|
| `test_cameras_file.py` | OpenArm's cameras file carries the fm-skills chest intrinsics for all four roles, and the CameraInfo `camera_info` builds from it; normalised plain data; `plumb_bob` accepted; the large-image cap; every refusal |
| `test_fisheye.py` | The warp refuses `plumb_bob`; the render covers the edge midpoints and matches the CameraPlugin's focal length; about 12 % of the frame black; the warp inverts `cv2.fisheye.distortPoints` to 0.01 px |
| `test_depth.py` | Millimetres round-trip through the PNG; NaN, infinite, non-positive, past-range and far-plane depth are 0; the warp takes the nearest render pixel, never a blend, and keeps z-depth unchanged; corners carry no depth |
| `test_mount.py` | Mounts on a moving link's body and on the root, frames that are bodies, a moving joint and an unknown frame refused, quaternions |
| `test_camera_node.py` | The node on a fake render: JPEG of the real size with the frame, stamp and orientation kept, black corners, the real camera_info; a wrong-size render, a buffer that does not fit and a future stamp skipped; the stall warning; depth on the colour frame's header as a `16UC1; png` of the real size, waiting for its colour, dropped with a thinned colour, wrong-size or wrong-encoding depth skipped, an empty `depth_render_topic` refused; thinning to `rate_hz`, the `rate_hz` parameter overriding the file and its range; unknown roles refused, also by the executable |

`test/conftest.py` gives the tests a ROS domain of their own and holds the shared
constants (the shipped OpenArm cameras file, the fm-skills K and D).

## Build Type

`ament_python`. Exec-depends on `rclpy`, `sensor_msgs`, `python3-opencv`,
`python3-numpy` and `python3-yaml`.
