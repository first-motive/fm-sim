"""Camera mounts from a URDF: fixed joints up to a simulator body or the root."""

import math

from fm_sim_cameras import cameras_file, mount
import numpy as np
import pytest

URDF = """<robot name="probe">
  <link name="world"/>
  <link name="base"/>
  <link name="hand"/>
  <link name="cam_mount"/>
  <link name="cam_optical"/>
  <link name="chest_optical"/>
  <joint name="base_joint" type="fixed">
    <parent link="world"/><child link="base"/>
    <origin xyz="0 0 0.5"/>
  </joint>
  <joint name="arm_joint" type="revolute">
    <parent link="base"/><child link="hand"/>
    <origin xyz="0.2 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="cam_mount_joint" type="fixed">
    <parent link="hand"/><child link="cam_mount"/>
    <origin xyz="0 0 -0.1" rpy="3.141592653589793 0 0"/>
  </joint>
  <joint name="cam_optical_joint" type="fixed">
    <parent link="cam_mount"/><child link="cam_optical"/>
    <origin xyz="0.05 0 0" rpy="0 0 1.5707963267948966"/>
  </joint>
  <joint name="chest_joint" type="fixed">
    <parent link="base"/><child link="chest_optical"/>
    <origin xyz="0.1 0 0.3" rpy="0 1.5707963267948966 0"/>
  </joint>
</robot>"""


def test_frame_on_a_moving_link_mounts_on_that_body():
    placed = mount.fixed_mount(URDF, 'cam_optical', {'hand', 'base'})
    assert placed.parent == 'hand'
    # Turned pi about x, the mount's +x stays +x: the camera sits 5 cm out along it.
    assert placed.xyz == pytest.approx((0.05, 0.0, -0.1))
    half = math.sqrt(0.5)
    # Rx(pi) then Rz(pi/2): (w, x, y, z) = (0, 1, 0, 0) * (cos 45, 0, 0, sin 45); with
    # w zero, q and -q are both the canonical form.
    assert abs(np.dot(placed.quat_wxyz, (0.0, half, -half, 0.0))) == pytest.approx(1.0)


def test_frame_fixed_to_the_root_mounts_in_the_world():
    placed = mount.fixed_mount(URDF, 'chest_optical', {'hand'})
    assert placed.parent is None
    assert placed.xyz == pytest.approx((0.1, 0.0, 0.8))
    half = math.sqrt(0.5)
    assert placed.quat_wxyz == pytest.approx((half, 0.0, half, 0.0))


def test_frame_that_is_a_body_mounts_on_itself():
    placed = mount.fixed_mount(URDF, 'hand', {'hand'})
    assert (placed.parent, placed.xyz, placed.quat_wxyz) == ('hand', (0, 0, 0), (1, 0, 0, 0))


def test_moving_joint_before_a_body_is_refused():
    with pytest.raises(cameras_file.CameraError, match="moves with joint 'arm_joint'"):
        mount.fixed_mount(URDF, 'cam_optical', {'base'})


def test_unknown_frame_is_refused():
    with pytest.raises(cameras_file.CameraError, match='not a link'):
        mount.fixed_mount(URDF, 'no_frame', {'hand'})


@pytest.mark.parametrize('angles', [(0.3, -1.2, 2.5), (math.pi, 0.0, 0.0), (0.0, math.pi, 1.0)])
def test_quaternion_matches_the_rotation(angles):
    rotation = mount._rpy_matrix(*angles)
    w, x, y, z = mount.quat_wxyz(rotation)
    rebuilt = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    assert rebuilt == pytest.approx(rotation, abs=1e-12)
    assert w >= 0
