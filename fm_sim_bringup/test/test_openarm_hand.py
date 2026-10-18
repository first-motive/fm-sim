"""The OpenArm catalog entry's description and MuJoCo model are the same robot.

run.launch.py expands fm-robot's description with the entry's ``description_args``
and runs the MJCF the entry's ``mjcf`` names inside the generated world. These
tests build both the way the launch does and compare them: joint limits, actuator
ranges, the hand's frames under random arm and finger poses, and a can held in the
loader hand. Needs the openarm_mujoco external (vcs import fm-sim.repos) and the
mujoco Python binding (requirements-sim.txt).
"""

import math
import os
import xml.etree.ElementTree as ET

from fm_sim_bringup import catalog, run
from fm_sim_models.models import mjcf_path
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    not os.path.isfile(mjcf_path('openarm_pinch')),
    reason='openarm_mujoco external (vcs import)',
)

SIDES = ('right', 'left')
# Frames the hand carries, as MuJoCo bodies (URDF links) or sites.
HAND_BODIES = ('ee_base_link', 'hand', 'right_finger', 'left_finger')
HAND_SITES = ('grasp_frame',)
# Where the fingers stop on the 66 mm can: well inside 0..0.05 m (they read about
# 0.034, as the jaw meshes overlap 3.5 mm at 0).
CAN_GRIP_M = (0.025, 0.045)


@pytest.fixture(scope='module')
def mujoco():
    return pytest.importorskip('mujoco', reason='mujoco Python binding (requirements-sim.txt)')


def _build(tmp_path, scenario='empty', description_args=None, mjcf=None):
    """Return ``(urdf root, world path)`` as run.launch.py builds them for openarm."""
    import xacro
    entries = run.resolve('mujoco', 'openarm', scenario, 'idle')
    robot = dict(entries['robot'])
    robot['description_args'] = dict(robot['description_args'], **(description_args or {}))
    robot['mjcf'] = mjcf or robot['mjcf']
    robot_mjcf = mjcf_path(robot['mjcf'])
    world = run.world_model(
        entries['backend'], robot, entries['scenario'], 0, robot_mjcf, str(tmp_path)
    )
    urdf = xacro.process_file(
        catalog.share_file(robot['description_file']),
        mappings=run.description_args(
            entries['backend'], robot, world, catalog.share_file(robot['controllers_file'])
        ),
    ).toxml()
    return ET.fromstring(urdf), world


@pytest.fixture(scope='module')
def built(tmp_path_factory, mujoco):
    urdf, world = _build(tmp_path_factory.mktemp('openarm'))
    return urdf, mujoco.MjModel.from_xml_path(world)


def _axis_angle(axis, angle):
    x, y, z = axis / np.linalg.norm(axis)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def _rpy(roll, pitch, yaw):
    def turn(axis, angle):
        return _axis_angle(np.array(axis, dtype=float), angle)
    return turn((0, 0, 1), yaw) @ turn((0, 1, 0), pitch) @ turn((1, 0, 0), roll)


def _fk(urdf, link, positions):
    """Return the 4x4 pose of ``link`` in the URDF root at joint ``positions``."""
    parent_joint = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    transform = np.eye(4)
    while link in parent_joint:
        joint = parent_joint[link]
        origin = joint.find('origin')
        step = np.eye(4)
        if origin is not None:
            step[:3, :3] = _rpy(*[float(v) for v in origin.get('rpy', '0 0 0').split()])
            step[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        motion = np.eye(4)
        value = positions.get(joint.get('name'), 0.0)
        if joint.get('type') != 'fixed':
            axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
            if joint.get('type') == 'prismatic':
                motion[:3, 3] = axis * value
            else:
                motion[:3, :3] = _axis_angle(axis, value)
        transform = step @ motion @ transform
        link = joint.find('parent').get('link')
    return transform


def _moving_joints(urdf):
    return [j for j in urdf.findall('joint') if j.get('type') in ('revolute', 'prismatic')]


def _actuator(model, joint_id):
    found = [i for i in range(model.nu) if model.actuator_trnid[i, 0] == joint_id]
    return model.actuator(found[0]) if found else None


def test_every_joint_has_the_description_limits(built, mujoco):
    urdf, model = built
    for joint in _moving_joints(urdf):
        name = joint.get('name')
        limit = joint.find('limit')
        expected = [float(limit.get('lower')), float(limit.get('upper'))]
        kind = mujoco.mjtJoint.mjJNT_SLIDE if joint.get('type') == 'prismatic' else (
            mujoco.mjtJoint.mjJNT_HINGE)
        assert model.joint(name).type[0] == kind, name
        assert model.joint(name).range == pytest.approx(expected, abs=1e-6), name
        actuator = _actuator(model, model.joint(name).id)
        if joint.find('mimic') is None:
            assert actuator.ctrlrange == pytest.approx(expected, abs=1e-5), name
        else:
            assert actuator is None, name


def test_hand_frames_follow_the_description(built, mujoco):
    urdf, model = built
    data = mujoco.MjData(model)
    rng = np.random.default_rng(13)
    joints = _moving_joints(urdf)
    for _ in range(20):
        positions = {}
        for joint in joints:
            name = joint.get('name')
            mimic = joint.find('mimic')
            if mimic is not None:
                continue
            low, high = model.joint(name).range
            positions[name] = float(rng.uniform(low, high))
        for joint in joints:
            mimic = joint.find('mimic')
            if mimic is not None:
                positions[joint.get('name')] = positions[mimic.get('joint')]
        for name, value in positions.items():
            data.qpos[model.joint(name).qposadr[0]] = value
        mujoco.mj_kinematics(model, data)
        for side in SIDES:
            prefix = f'openarm_{side}_'
            for body in HAND_BODIES:
                expected = _fk(urdf, prefix + body, positions)
                assert data.body(prefix + body).xpos == pytest.approx(expected[:3, 3], abs=1e-6)
                assert data.body(prefix + body).xmat.reshape(3, 3) == pytest.approx(
                    expected[:3, :3], abs=1e-6)
            for site in HAND_SITES:
                expected = _fk(urdf, prefix + site, positions)
                assert data.site(prefix + site).xpos == pytest.approx(expected[:3, 3], abs=1e-6)
                assert data.site(prefix + site).xmat.reshape(3, 3) == pytest.approx(
                    expected[:3, :3], abs=1e-6)


@pytest.mark.parametrize('hand,mjcf', [('loader', None), ('pinch', 'openarm_pinch')])
def test_hand_and_mjcf_pick_the_same_gripper(tmp_path, mujoco, hand, mjcf):
    # The catalog switches the hand in two fields; each pairing must agree.
    urdf, world = _build(tmp_path, description_args={'hand': hand}, mjcf=mjcf)
    model = mujoco.MjModel.from_xml_path(world)
    finger = urdf.find("joint[@name='openarm_right_finger_joint1']")
    kind = {'prismatic': mujoco.mjtJoint.mjJNT_SLIDE, 'revolute': mujoco.mjtJoint.mjJNT_HINGE}
    assert model.joint('openarm_right_finger_joint1').type[0] == kind[finger.get('type')]
    limit = finger.find('limit')
    assert model.joint('openarm_right_finger_joint1').range == pytest.approx(
        [float(limit.get('lower')), float(limit.get('upper'))], abs=1e-4)


def test_catalog_runs_the_loader_hand(built):
    urdf, _ = built
    entry = catalog.load('robots', 'openarm')
    assert entry['description_args']['hand'] == 'loader'
    assert entry['mjcf'] == 'openarm'
    assert urdf.find("link[@name='openarm_right_hand']") is not None


def _settle(mujoco, model, data, seconds, hold=None):
    for _ in range(int(seconds / model.opt.timestep)):
        if hold:
            hold()
        mujoco.mj_step(model, data)


def test_right_hand_closes_on_the_can_and_holds_it(tmp_path, mujoco):
    # The arm hangs at zero, the hand pointing down. The can is held kinematically
    # between the open fingers while they close, then let go: friction alone must
    # hold it against gravity, along the fingers.
    _, world = _build(tmp_path, scenario='can_on_table')
    model = mujoco.MjModel.from_xml_path(world)
    data = mujoco.MjData(model)
    finger = _actuator(model, model.joint('openarm_right_finger_joint1').id).id
    fingers = [model.joint(f'openarm_right_finger_joint{i}').qposadr[0] for i in (1, 2)]
    can_q = model.joint('can').qposadr[0]
    can_v = model.joint('can').dofadr[0]
    site = model.site('openarm_right_grasp_frame').id
    for adr in fingers:
        data.qpos[adr] = 0.05
    mujoco.mj_forward(model, data)
    for i in range(model.nu):
        data.ctrl[i] = data.qpos[model.jnt_qposadr[model.actuator_trnid[i, 0]]]

    def can_in_hand():
        rotation = data.site_xmat[site].reshape(3, 3)
        return rotation.T @ (data.qpos[can_q:can_q + 3] - data.site_xpos[site])

    def hold_can():
        # 50 mm past the grasp frame, so the can clears the palm; upright.
        rotation = data.site_xmat[site].reshape(3, 3)
        data.qpos[can_q:can_q + 3] = data.site_xpos[site] + 0.05 * rotation[:, 2]
        data.qpos[can_q + 3:can_q + 7] = [1, 0, 0, 0]
        data.qvel[can_v:can_v + 6] = 0

    _settle(mujoco, model, data, 1.0, hold_can)
    data.ctrl[finger] = 0.0
    _settle(mujoco, model, data, 1.0, hold_can)
    gripped = can_in_hand()
    closed = [float(data.qpos[adr]) for adr in fingers]
    # Each finger stops on the can, short of closed; the mimic keeps them together.
    assert CAN_GRIP_M[0] < closed[0] < CAN_GRIP_M[1]
    assert closed[1] == pytest.approx(closed[0], abs=1e-3)
    _settle(mujoco, model, data, 3.0)
    assert can_in_hand() == pytest.approx(gripped, abs=0.002)
    assert [float(data.qpos[adr]) for adr in fingers] == pytest.approx(closed, abs=1e-3)


def test_empty_close_shuts_the_fingers(tmp_path, mujoco):
    _, world = _build(tmp_path)
    model = mujoco.MjModel.from_xml_path(world)
    data = mujoco.MjData(model)
    finger = _actuator(model, model.joint('openarm_left_finger_joint1').id).id
    adr = model.joint('openarm_left_finger_joint1').qposadr[0]
    data.ctrl[finger] = 0.05
    _settle(mujoco, model, data, 1.0)
    assert data.qpos[adr] == pytest.approx(0.05, abs=1e-3)
    data.ctrl[finger] = 0.0
    _settle(mujoco, model, data, 1.0)
    assert data.qpos[adr] == pytest.approx(0.0, abs=1e-3)


def _floats(text):
    return np.array([float(v) for v in text.split()])


def _quat_matrix(wxyz):
    w, x, y, z = np.asarray(wxyz, dtype=float) / np.linalg.norm(wxyz)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


@pytest.mark.parametrize('side', SIDES)
def test_wrist_matches_the_description(built, side):
    # fm-robot puts the loader's link7 wrist on ee_base_link; the MJCF must carry
    # the same mesh pose, mirroring and inertial, so every backend has one wrist.
    urdf, model = built
    prefix = f'openarm_{side}_'
    link = urdf.find(f"link[@name='{prefix}ee_base_link']")
    collision = link.find('collision')
    origin = collision.find('origin')
    urdf_rotation = _rpy(*_floats(origin.get('rpy')))
    urdf_mirror = np.sign(_floats(collision.find('geometry/mesh').get('scale'))[1])

    mjcf = ET.parse(mjcf_path('openarm')).getroot()
    geom = mjcf.find(
        f".//body[@name='{prefix}ee_base_link']/geom[@name='{prefix}link7_collision']")
    mesh = mjcf.find(f"asset/mesh[@name='{geom.get('mesh')}']")
    assert _floats(geom.get('pos')) == pytest.approx(_floats(origin.get('xyz')), abs=1e-9)
    assert _quat_matrix(_floats(geom.get('quat'))) == pytest.approx(urdf_rotation, abs=1e-6)
    assert np.sign(_floats(mesh.get('scale'))[1]) == urdf_mirror

    inertial = link.find('inertial')
    inertia = inertial.find('inertia')
    i = {k: float(inertia.get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')}
    urdf_inertia = np.array([
        [i['ixx'], i['ixy'], i['ixz']],
        [i['ixy'], i['iyy'], i['iyz']],
        [i['ixz'], i['iyz'], i['izz']],
    ])
    body = model.body(prefix + 'ee_base_link')
    rotation = _quat_matrix(body.iquat)
    assert body.mass[0] == pytest.approx(float(inertial.find('mass').get('value')), rel=1e-6)
    assert body.ipos == pytest.approx(_floats(inertial.find('origin').get('xyz')), abs=1e-6)
    assert rotation @ np.diag(body.inertia) @ rotation.T == pytest.approx(urdf_inertia, abs=1e-8)


def _initial_positions(urdf):
    """Return the joints' spawn positions from the description's ros2_control block."""
    positions = {}
    for joint in urdf.findall('ros2_control/joint'):
        value = joint.find("state_interface[@name='position']/param[@name='initial_value']")
        if value is not None:
            positions[joint.get('name')] = float(value.text)
    for joint in _moving_joints(urdf):
        mimic = joint.find('mimic')
        if mimic is not None:
            positions[joint.get('name')] = positions[mimic.get('joint')]
    return positions


@pytest.mark.parametrize('fingers', [None, 0.05])
def test_robot_spawns_clear_of_itself(built, mujoco, fingers):
    # At the description's spawn pose (and with the fingers fully open) no two
    # robot geoms touch: the loader hands are wide enough to meet at the pinch pose.
    urdf, model = built
    positions = _initial_positions(urdf)
    assert {f'openarm_{side}_joint{i}' for side in SIDES for i in range(1, 8)} <= set(positions)
    if fingers is not None:
        for name in positions:
            if 'finger_joint' in name:
                positions[name] = fingers
    data = mujoco.MjData(model)
    for name, value in positions.items():
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(model, data)
    touching = [(model.geom(c.geom1).name, model.geom(c.geom2).name)
                for c in data.contact[:data.ncon]]
    assert touching == []
