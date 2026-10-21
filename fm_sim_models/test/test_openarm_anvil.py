"""The MJCF built from the OpenArm's Anvil description — no ROS.

The synthetic tests build a small URDF under the loader's names (two seven-joint
arms with prismatic fingers, one mesh behind ``package://``) and check the patch
on it, so they run anywhere the mujoco binding is installed. The tests on the real
description need the private openarm_v2_anvil_description and skip without it.
"""

import os
import subprocess
import sys
import xml.etree.ElementTree as ET

from fm_sim_models import cache, openarm_anvil
from fm_sim_models.models import mjcf_path
import pytest

mujoco = pytest.importorskip('mujoco', reason='mujoco Python binding (requirements-sim.txt)')

# A tetrahedron, 1 cm on a side: the one mesh the synthetic arm uses.
TETRAHEDRON = """v 0 0 0
v 0.01 0 0
v 0 0.01 0
v 0 0 0.01
f 1 3 2
f 1 2 4
f 1 4 3
f 2 3 4
"""
# Arbitrary limits; joint3 keeps a seventh significant digit MuJoCo's writer drops.
ARM_LIMIT = (-1.570796, 1.570796)
EFFORT = 12.0
FINGER_LIMIT = (0.0, 0.05)


def _inertial(link):
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass', value='0.1')
    ET.SubElement(inertial, 'inertia', ixx='1e-4', iyy='1e-4', izz='1e-4',
                  ixy='0', ixz='0', iyz='0')


def _link(robot, name, mesh=None):
    link = ET.SubElement(robot, 'link', name=name)
    _inertial(link)
    collision = ET.SubElement(ET.SubElement(link, 'collision'), 'geometry')
    if mesh:
        ET.SubElement(collision, 'mesh', filename=mesh)
    else:
        ET.SubElement(collision, 'box', size='0.01 0.01 0.01')


def _joint(robot, name, kind, parent, child, xyz, limit=None, mimic=None):
    joint = ET.SubElement(robot, 'joint', name=name, type=kind)
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'origin', xyz=xyz, rpy='0 0 0')
    if kind != 'fixed':
        ET.SubElement(joint, 'axis', xyz='0 1 0' if kind == 'prismatic' else '0 0 1')
        ET.SubElement(joint, 'limit', lower=str(limit[0]), upper=str(limit[1]),
                      effort=str(EFFORT), velocity='1')
    if mimic:
        ET.SubElement(joint, 'mimic', joint=mimic)


def synthetic_urdf(mimic=True, mesh='package://probe/meshes/t.obj'):
    """Return a URDF under the loader's names; link1 carries ``mesh``."""
    robot = ET.Element('robot', name='probe')
    ET.SubElement(robot, 'link', name='world')
    _link(robot, 'follower_body_link0')
    _joint(robot, 'body_joint', 'fixed', 'world', 'follower_body_link0', '0 0 0')
    for side, y in (('follower_l_', 0.3), ('follower_r_', -0.3)):
        _link(robot, f'{side}link0')
        _joint(robot, f'{side}base_joint', 'fixed', 'follower_body_link0', f'{side}link0',
               f'0 {y} 0.5')
        for index in range(1, 8):
            _link(robot, f'{side}link{index}', mesh if index == 1 else None)
            _joint(robot, f'{side}joint{index}', 'revolute', f'{side}link{index - 1}',
                   f'{side}link{index}', '0 0 0.1', ARM_LIMIT)
        _link(robot, f'{side}hand')
        _joint(robot, f'{side}hand_joint', 'fixed', f'{side}link7', f'{side}hand', '0 0 0.1')
        for finger, joint, sign in (('right_finger', 'finger_joint1', -1),
                                    ('left_finger', 'finger_joint2', 1)):
            _link(robot, f'{side}{finger}')
            _joint(robot, f'{side}{joint}', 'prismatic', f'{side}hand', f'{side}{finger}',
                   f'0 {0.02 * sign} 0.05', FINGER_LIMIT,
                   mimic=f'{side}finger_joint1' if mimic and sign > 0 else None)
    return robot


@pytest.fixture
def probe(tmp_path, monkeypatch):
    """Return ``build(urdf root) -> MJCF text`` with the probe share in ``tmp_path``."""
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path / 'cache'))
    share = tmp_path / 'share' / 'probe'
    (share / 'meshes').mkdir(parents=True)
    (share / 'meshes' / 't.obj').write_text(TETRAHEDRON)

    def build(robot, path=False):
        urdf = tmp_path / 'probe.urdf'
        ET.ElementTree(robot).write(urdf)
        shares = {'probe': str(share)}
        if path:
            return openarm_anvil.mjcf_path(str(urdf), shares.__getitem__)
        return openarm_anvil.mjcf_text(str(urdf), shares.__getitem__)
    build.share = share
    return build


def _actuator(model, joint):
    found = [i for i in range(model.nu) if model.actuator_trnid[i, 0] == model.joint(joint).id]
    return model.actuator(found[0]) if found else None


def test_every_arm_joint_and_finger_gets_a_servo_over_its_urdf_range(probe):
    model = mujoco.MjModel.from_xml_string(probe(synthetic_urdf()))
    assert model.nu == 16
    for side in openarm_anvil.SIDES:
        for index, motor in openarm_anvil.ARM_MOTORS.items():
            name = f'{side}joint{index}'
            # The full-precision URDF range, not the six digits MuJoCo writes.
            assert list(model.joint(name).range) == list(ARM_LIMIT)
            actuator = _actuator(model, name)
            assert list(actuator.ctrlrange) == list(ARM_LIMIT)
            assert list(actuator.forcerange) == [-EFFORT, EFFORT]
            assert actuator.gainprm[0] == motor.kp
            assert model.dof_damping[model.joint(name).dofadr[0]] == motor.damping
        finger = _actuator(model, f'{side}finger_joint1')
        assert model.joint(f'{side}finger_joint1').type[0] == mujoco.mjtJoint.mjJNT_SLIDE
        assert list(finger.ctrlrange) == list(FINGER_LIMIT)
        assert list(finger.forcerange) == [-openarm_anvil.FINGER_FORCE_N,
                                           openarm_anvil.FINGER_FORCE_N]
        assert _actuator(model, f'{side}finger_joint2') is None
    assert model.opt.noslip_iterations == openarm_anvil.NOSLIP_ITERATIONS


def test_the_mimic_finger_follows_the_driven_one(probe):
    model = mujoco.MjModel.from_xml_string(probe(synthetic_urdf()))
    data = mujoco.MjData(model)
    finger = _actuator(model, 'follower_r_finger_joint1').id
    data.ctrl[finger] = 0.04
    for _ in range(500):
        mujoco.mj_step(model, data)
    leader = data.qpos[model.joint('follower_r_finger_joint1').qposadr[0]]
    follower = data.qpos[model.joint('follower_r_finger_joint2').qposadr[0]]
    assert leader == pytest.approx(0.04, abs=1e-3)
    assert follower == pytest.approx(leader, abs=1e-3)
    (mimic,) = [model.equality(i) for i in range(model.neq)
                if model.eq_obj2id[i] == model.joint('follower_r_finger_joint1').id]
    assert list(mimic.solref) == [float(v) for v in openarm_anvil.MIMIC_SOLREF.split()]


def test_fingers_and_wrist_never_collide_with_each_other(probe):
    model = mujoco.MjModel.from_xml_string(probe(synthetic_urdf()))
    excluded = {
        frozenset((model.body(signature >> 16).name, model.body(signature & 0xFFFF).name))
        for signature in model.exclude_signature
    }
    for side in openarm_anvil.SIDES:
        assert frozenset((f'{side}right_finger', f'{side}left_finger')) in excluded
        assert frozenset((f'{side}link5', f'{side}link7')) in excluded


def test_meshes_resolve_to_absolute_files_in_their_share(probe):
    text = probe(synthetic_urdf())
    assert f'file="{probe.share}/meshes/t.obj"' in text
    assert 'package://' not in text


@pytest.mark.parametrize('mesh,message', [
    ('package://probe/../escape.obj', 'outside'),
    ('../escape.obj', 'outside'),
    ('package://probe/meshes/missing.obj', 'does not exist'),
])
def test_a_mesh_that_does_not_resolve_is_named(probe, mesh, message):
    with pytest.raises(RuntimeError, match=message):
        probe(synthetic_urdf(mesh=mesh))


def test_a_finger_without_its_mimic_is_refused(probe):
    with pytest.raises(RuntimeError, match='no equality making follower_l_finger_joint2'):
        probe(synthetic_urdf(mimic=False))


def test_a_missing_arm_joint_is_named(probe):
    robot = synthetic_urdf()
    joint = robot.find("joint[@name='follower_r_joint4']")
    joint.set('name', 'follower_r_elbow')
    with pytest.raises(RuntimeError, match='follower_r_joint4'):
        probe(robot)


def test_cached_by_content(probe, tmp_path):
    path = probe(synthetic_urdf(), path=True)
    assert os.path.dirname(path) == cache.cache_dir()
    assert os.path.basename(path).startswith('openarm_anvil_')
    assert probe(synthetic_urdf(), path=True) == path


def _installed_urdf():
    try:
        return openarm_anvil.description_urdf()
    except RuntimeError:
        return None


needs_anvil = pytest.mark.skipif(
    _installed_urdf() is None,
    reason='openarm_v2_anvil_description (private; in the fm-sim-controllers image)',
)


@pytest.mark.skipif(_installed_urdf() is not None, reason='the description is installed')
def test_registry_names_the_missing_description():
    with pytest.raises(RuntimeError, match='openarm_v2_anvil_description is not installed'):
        mjcf_path('openarm_anvil')


@needs_anvil
def test_joints_and_finger_units_match_the_description(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path))
    model = mujoco.MjModel.from_xml_path(mjcf_path('openarm_anvil'))
    kinds = {'revolute': mujoco.mjtJoint.mjJNT_HINGE, 'prismatic': mujoco.mjtJoint.mjJNT_SLIDE}
    urdf = ET.parse(_installed_urdf()).getroot()
    moving = [j for j in urdf.iter('joint') if j.get('type') in kinds]
    assert {model.joint(i).name for i in range(model.njnt)} == {j.get('name') for j in moving}
    for joint in moving:
        name, limit = joint.get('name'), joint.find('limit')
        expected = [float(limit.get('lower')), float(limit.get('upper'))]
        assert model.joint(name).type[0] == kinds[joint.get('type')], name
        assert list(model.joint(name).range) == expected, name
        actuator = _actuator(model, name)
        if joint.find('mimic') is None:
            assert list(actuator.ctrlrange) == expected, name
        else:
            assert actuator is None, name


def test_compiles_whatever_gl_backend_the_run_names():
    # A run under MUJOCO_GL=osmesa without the OSMesa library failed at `import mujoco`.
    code = 'from fm_sim_models import openarm_anvil; openarm_anvil._mujoco().MjSpec()'
    env = {**os.environ, 'MUJOCO_GL': 'osmesa'}
    subprocess.run([sys.executable, '-c', code], env=env, check=True)
