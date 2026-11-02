"""The fer_* rename of the vendored mujoco_menagerie Franka Panda — no ROS."""

import os
import re

from fm_sim_models import panda
from fm_sim_models.models import _vendored, mjcf_path
import pytest

MENAGERIE = _vendored('mujoco_menagerie')
needs_menagerie = pytest.mark.skipif(
    not os.path.isdir(MENAGERIE), reason='mujoco_menagerie external (vcs import)'
)
FER_ARM = [f'fer_joint{i}' for i in range(1, 8)]


@pytest.fixture
def vendored(tmp_path):
    """Return a copy of the Menagerie panda.xml a test may edit, and its text."""
    root = tmp_path / 'mujoco_menagerie' / 'franka_emika_panda'
    root.mkdir(parents=True)
    os.symlink(os.path.join(MENAGERIE, 'franka_emika_panda', 'assets'), root / 'assets')
    with open(os.path.join(MENAGERIE, 'franka_emika_panda', 'panda.xml')) as source:
        text = source.read()
    model = root / 'panda.xml'
    model.write_text(text)
    return root.parent, model, text


@needs_menagerie
@pytest.mark.parametrize('change,message', [
    (lambda t: t.replace('name="joint4"', 'name="elbow"'), r"no joint named \['joint4'\]"),
    (lambda t: t.replace(' meshdir="assets"', ''), 'no <compiler meshdir>'),
    (lambda t: t.replace('meshdir="assets"', 'meshdir="assets" texturedir="tex"'),
     'texturedir'),
    (lambda t: re.sub(r'<actuator>.*</actuator>', '', t, flags=re.DOTALL), 'no <actuator>'),
    (lambda t: t.replace('<general class="panda" name="actuator3" joint="joint3"',
                         '<motor name="actuator3" joint="joint3"'), 'unexpected <motor>'),
])
def test_upstream_drift_is_named(vendored, change, message):
    root, model, text = vendored
    model.write_text(change(text))
    with pytest.raises(RuntimeError, match=message):
        panda.mjcf_text(str(root))


@pytest.fixture(scope='module')
def model():
    if not os.path.isdir(MENAGERIE):
        pytest.skip('mujoco_menagerie external (vcs import)')
    mujoco = pytest.importorskip('mujoco', reason='mujoco Python binding (requirements-sim.txt)')
    return mujoco.MjModel.from_xml_path(mjcf_path('panda'))


def _actuator(model, joint):
    (index,) = [i for i in range(model.nu) if model.actuator_trnid[i, 0] == model.joint(joint).id]
    return model.actuator(index)


def test_keyframes_are_dropped(model):
    # Menagerie's keyframe ctrl vector fits its 8 general actuators, not the servos.
    assert model.nkey == 0


def test_meshes_resolve_from_any_directory(monkeypatch, tmp_path):
    if not os.path.isdir(MENAGERIE):
        pytest.skip('mujoco_menagerie external (vcs import)')
    mujoco = pytest.importorskip('mujoco', reason='mujoco Python binding (requirements-sim.txt)')
    monkeypatch.chdir(tmp_path)
    assert mujoco.MjModel.from_xml_path(mjcf_path('panda')).nmesh > 0


def test_joints_carry_fm_robot_names(model):
    names = {model.joint(i).name for i in range(model.njnt)}
    assert names == set(FER_ARM) | {'fer_finger_joint1', 'fer_finger_joint2'}


def test_every_commanded_joint_has_one_position_servo(model):
    # MujocoSystemInterface commands a joint through the actuator on it; a position
    # servo has gain kp and bias (0, -kp, -kv).
    assert model.nu == len(FER_ARM) + 1
    for joint in FER_ARM + ['fer_finger_joint1']:
        actuator = _actuator(model, joint)
        kp = actuator.gainprm[0]
        assert kp > 0, joint
        assert actuator.biasprm[1] == pytest.approx(-kp), joint
        assert actuator.biasprm[2] < 0, joint


def test_arm_servo_ranges_follow_the_joints(model):
    for joint in FER_ARM:
        assert _actuator(model, joint).ctrlrange == pytest.approx(model.joint(joint).range)


def test_hand_servo_works_in_metres(model):
    actuator = _actuator(model, 'fer_finger_joint1')
    assert actuator.ctrlrange == pytest.approx(panda.FINGER_RANGE_M)
    assert actuator.gainprm[0] == pytest.approx(panda.FINGER_KP)
    assert actuator.forcerange == pytest.approx([-panda.FINGER_FORCE_N, panda.FINGER_FORCE_N])
    assert model.joint('fer_finger_joint1').range == pytest.approx(panda.FINGER_RANGE_M)


def test_second_finger_follows_the_first(model):
    finger1, finger2 = (model.joint(n).id for n in ('fer_finger_joint1', 'fer_finger_joint2'))
    assert any(
        {model.eq_obj1id[i], model.eq_obj2id[i]} == {finger1, finger2} for i in range(model.neq)
    )
