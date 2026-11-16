"""The Axol MJCF built from fm_description's ox_cart URDF.

The refusal tests build a small URDF in a temporary directory and run anywhere.
The model tests need the mujoco binding and fm_description installed with the
vendored Axol description (fm-sim.repos imports it), and skip without either.
"""

import importlib.util
import xml.etree.ElementTree as ET

from fm_sim_models import axol, models
import pytest

ARM = [f'{side}{j}_0' for side in axol.SIDES for j in axol.ARM_JOINTS]
FINGERS = [f'{side}finger_1_joint' for side in axol.SIDES]
MIMICS = [f'{side}finger_2_joint' for side in axol.SIDES]


def _installed_urdf():
    try:
        return axol.description_urdf()
    except RuntimeError:
        return None


needs_model = pytest.mark.skipif(
    importlib.util.find_spec('mujoco') is None or _installed_urdf() is None,
    reason='the mujoco binding and fm_description with the vendored Axol description',
)


def test_registry_compiles_the_axol_model():
    assert models._MJCF['axol'] is axol.mjcf_path


@pytest.mark.skipif(_installed_urdf() is not None, reason='fm_description is installed')
def test_registry_names_the_missing_description():
    with pytest.raises(RuntimeError, match='fm_description is not installed'):
        models.mjcf_path('axol')


def _write_urdf(tmp_path, mesh='meshes/a.stl', drive=True, lift=True):
    """Write a one-joint URDF beside a mesh and return its path."""
    (tmp_path / 'meshes').mkdir()
    (tmp_path / 'meshes' / 'a.stl').write_text('solid a\nendsolid a\n')
    gains = '<drive stiffness="130" damping="5"/>' if drive else ''
    lift_joint = ('<joint name="lift" type="prismatic"><parent link="world"/>'
                  '<child link="root"/></joint>') if lift else ''
    joints = ''.join(
        f'<joint name="{name}" type="revolute"><limit lower="-1" upper="1" effort="87"/>'
        f'{gains}</joint>' for name in ARM
    )
    path = tmp_path / 'axol.urdf'
    path.write_text(
        f'<robot name="assembly"><link name="world"/><link name="root"><visual><geometry>'
        f'<mesh filename="{mesh}"/></geometry></visual></link>{lift_joint}{joints}</robot>'
    )
    return str(path)


@pytest.mark.parametrize('mesh, message', [
    ('../outside.stl', 'is not a path inside'),
    ('package://assembly/meshes/a.stl', 'is not a path inside'),
    ('meshes/missing.stl', 'which does not exist'),
])
def test_refuses_a_mesh_it_cannot_resolve(tmp_path, mesh, message):
    with pytest.raises(RuntimeError, match=message):
        axol.mjcf_text(_write_urdf(tmp_path, mesh=mesh))


def test_refuses_an_arm_joint_with_no_drive_gains(tmp_path):
    with pytest.raises(RuntimeError, match="no <drive stiffness damping> on 'left_s1_0'"):
        axol.mjcf_text(_write_urdf(tmp_path, drive=False))


def test_refuses_a_description_with_no_lift(tmp_path):
    with pytest.raises(RuntimeError, match="no joint 'lift'"):
        axol.mjcf_text(_write_urdf(tmp_path, lift=False))


@pytest.fixture
def model(monkeypatch, tmp_path):
    mujoco = pytest.importorskip('mujoco')
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path))
    return mujoco.MjModel.from_xml_path(models.mjcf_path('axol'))


@needs_model
def test_moving_joints_are_the_arms_and_fingers(model):
    # The lift is fixed: no ros2_control system claims it.
    names = {model.joint(i).name for i in range(model.njnt)}
    assert names == set(ARM + FINGERS + MIMICS)


@needs_model
def test_every_controlled_joint_has_a_servo_over_its_urdf_range(model):
    urdf = ET.parse(_installed_urdf()).getroot()
    actuated = {model.joint(model.actuator_trnid[i, 0]).name: model.actuator(i)
                for i in range(model.nu)}
    assert set(actuated) == set(ARM + FINGERS)
    for name, actuator in actuated.items():
        limit = urdf.find(f"joint[@name='{name}']/limit")
        assert list(actuator.ctrlrange) == [float(limit.get('lower')), float(limit.get('upper'))]
        assert actuator.forcerange[1] == float(limit.get('effort'))
    drive = urdf.find("joint[@name='left_e1_0']/drive")
    assert actuated['left_e1_0'].gainprm[0] == float(drive.get('stiffness'))


@needs_model
def test_servos_track_and_the_second_finger_follows(model):
    import mujoco
    data = mujoco.MjData(model)
    targets = {'left_e1_0': 1.0, 'right_e1_0': -1.0, 'left_w1_0': 0.5, 'right_e2_0': -0.8,
               'left_finger_1_joint': -0.03, 'right_finger_1_joint': -0.02}
    for i in range(model.nu):
        data.ctrl[i] = targets.get(model.joint(model.actuator_trnid[i, 0]).name, 0.0)
    for _ in range(1500):
        mujoco.mj_step(model, data)
    for name, target in targets.items():
        assert data.joint(name).qpos[0] == pytest.approx(target, abs=0.01), name
    for side in axol.SIDES:
        follower = data.joint(f'{side}finger_2_joint').qpos[0]
        assert follower == pytest.approx(data.joint(f'{side}finger_1_joint').qpos[0], abs=1e-3)


@needs_model
def test_rest_pose_has_no_contacts(model):
    import mujoco
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.ncon == 0
