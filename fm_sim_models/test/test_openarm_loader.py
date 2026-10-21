"""The OpenArm loader-hand patch of the vendored openarm_mujoco v2 model — no ROS."""

import os

from fm_sim_models import cache, openarm_loader
from fm_sim_models.models import mjcf_path
import pytest

OPENARM_MUJOCO = os.path.dirname(os.path.dirname(mjcf_path('openarm_pinch')))
needs_openarm = pytest.mark.skipif(
    not os.path.isdir(OPENARM_MUJOCO), reason='openarm_mujoco external (vcs import)'
)


@pytest.fixture
def vendored(tmp_path):
    """Return a copy of openarm_mujoco whose v2 model a test may edit, and its text."""
    root = tmp_path / 'openarm_mujoco'
    (root / 'v2').mkdir(parents=True)
    os.symlink(os.path.join(OPENARM_MUJOCO, 'v1'), root / 'v1')
    os.symlink(os.path.join(OPENARM_MUJOCO, 'v2', 'assets'), root / 'v2' / 'assets')
    with open(os.path.join(OPENARM_MUJOCO, 'v2', 'openarm_bimanual.xml')) as source:
        text = source.read()
    model = root / 'v2' / 'openarm_bimanual.xml'
    model.write_text(text)
    return root, model, text


@needs_openarm
def test_cached_by_content(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path))
    path = mjcf_path('openarm')
    assert os.path.dirname(path) == str(tmp_path / 'fm_sim_models')
    assert mjcf_path('openarm') == path
    with open(path) as model_file:
        assert model_file.read() == openarm_loader.mjcf_text(OPENARM_MUJOCO)
    assert not [n for n in os.listdir(tmp_path / 'fm_sim_models') if n.endswith('.tmp')]
    # A different patch is a different file; the old one is left as it was.
    monkeypatch.setattr(openarm_loader, 'FINGER_KP', openarm_loader.FINGER_KP + 1)
    assert mjcf_path('openarm') != path
    assert os.path.isfile(path)


@needs_openarm
def test_a_damaged_cache_file_is_rewritten(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path))
    path = mjcf_path('openarm')
    with open(path, 'w') as model_file:
        model_file.write('<mujoco/>')
    assert mjcf_path('openarm') == path
    with open(path) as model_file:
        assert model_file.read() == openarm_loader.mjcf_text(OPENARM_MUJOCO)


def test_cache_defaults_to_home(monkeypatch, tmp_path):
    monkeypatch.delenv('XDG_CACHE_HOME', raising=False)
    monkeypatch.setenv('HOME', str(tmp_path))
    assert cache.cache_dir() == str(tmp_path / '.cache' / 'fm_sim_models')


@needs_openarm
def test_noslip_added_where_the_model_has_no_option(vendored):
    root, model, text = vendored
    assert '<option' not in text
    patched = openarm_loader.mjcf_text(str(root))
    assert f'<option noslip_iterations="{openarm_loader.NOSLIP_ITERATIONS}"' in patched
    # An <option> the model has keeps its settings and gains noslip.
    model.write_text(text.replace('<compiler', '<option timestep="0.001" />\n  <compiler', 1))
    patched = openarm_loader.mjcf_text(str(root))
    assert (f'<option timestep="0.001" noslip_iterations="{openarm_loader.NOSLIP_ITERATIONS}"'
            in patched)


@needs_openarm
@pytest.mark.parametrize('change,message', [
    (lambda t: t.replace(' meshdir="assets"', ''), 'no <compiler meshdir>'),
    (lambda t: t.replace('meshdir="assets"', 'meshdir="assets" texturedir="tex"'),
     'texturedir'),
    (lambda t: t.replace('<contact>', '<!--').replace('</contact>', '-->'), 'no <contact>'),
    (lambda t: t.replace('joint="openarm_right_joint4"', 'joint="openarm_right_joint3"'),
     "no actuator on 'openarm_right_joint4'"),
    (lambda t: t.replace('<body name="openarm_left_ee_base_link"', '<body name="x"'),
     "no body 'openarm_left_ee_base_link'"),
    (lambda t: t.replace('joint2="openarm_right_finger_joint2"', 'joint2="x"'),
     'no equality making openarm_right_finger_joint2'),
])
def test_upstream_drift_is_named(vendored, change, message):
    root, model, text = vendored
    model.write_text(change(text))
    with pytest.raises(RuntimeError, match=message):
        openarm_loader.mjcf_text(str(root))


@needs_openarm
def test_missing_v1_meshes_are_named(vendored):
    root, _, _ = vendored
    os.unlink(root / 'v1')
    with pytest.raises(RuntimeError, match='no v1 mesh directory'):
        openarm_loader.mjcf_text(str(root))


@pytest.fixture(scope='module')
def loader():
    if not os.path.isdir(OPENARM_MUJOCO):
        pytest.skip('openarm_mujoco external (vcs import)')
    mujoco = pytest.importorskip('mujoco', reason='mujoco Python binding (requirements-sim.txt)')
    return mujoco, mujoco.MjModel.from_xml_path(mjcf_path('openarm'))


def _actuator(model, joint):
    (index,) = [i for i in range(model.nu) if model.actuator_trnid[i, 0] == model.joint(joint).id]
    return model.actuator(index)


@pytest.mark.parametrize('side', ['right', 'left'])
def test_loader_limits_reach_joints_and_actuators(loader, side):
    _, model = loader
    for index, expected in openarm_loader.JOINT_RANGES[side].items():
        name = f'openarm_{side}_joint{index}'
        assert model.joint(name).range == pytest.approx(expected), name
        assert _actuator(model, name).ctrlrange == pytest.approx(expected), name


@pytest.mark.parametrize('side', ['right', 'left'])
def test_fingers_slide_over_the_loader_range(loader, side):
    mujoco, model = loader
    prefix = f'openarm_{side}_'
    for joint, link in (('finger_joint1', 'right_finger'), ('finger_joint2', 'left_finger')):
        finger = model.joint(prefix + joint)
        assert finger.type[0] == mujoco.mjtJoint.mjJNT_SLIDE
        assert finger.range == pytest.approx(openarm_loader.FINGER_RANGE_M)
        assert model.body(finger.bodyid[0]).name == prefix + link
        assert model.body(model.body(finger.bodyid[0]).parentid[0]).name == prefix + 'hand'
    actuator = _actuator(model, prefix + 'finger_joint1')
    assert actuator.ctrlrange == pytest.approx(openarm_loader.FINGER_RANGE_M)
    assert actuator.gainprm[0] == pytest.approx(openarm_loader.FINGER_KP)
    assert actuator.forcerange == pytest.approx(
        [-openarm_loader.FINGER_FORCE_N, openarm_loader.FINGER_FORCE_N]
    )
    # finger_joint2 has no actuator; the equality drives it 1:1 from finger_joint1.
    driven = {model.actuator_trnid[i, 0] for i in range(model.nu)}
    assert model.joint(prefix + 'finger_joint2').id not in driven
    (mimic,) = [model.equality(i) for i in range(model.neq)
                if model.eq_obj1id[i] == model.joint(prefix + 'finger_joint1').id]
    assert model.eq_obj2id[mimic.id] == model.joint(prefix + 'finger_joint2').id
    assert mimic.data[:5] == pytest.approx([0, 1, 0, 0, 0])


@pytest.mark.parametrize('side', ['right', 'left'])
def test_pinch_gripper_is_gone(loader, side):
    _, model = loader
    prefix = f'openarm_{side}_'
    bodies = {model.body(i).name for i in range(model.nbody)}
    assert not {prefix + 'ee_inner_finger', prefix + 'ee_outer_finger'} & bodies
    flange = model.body(prefix + 'ee_base_link').id
    children = [model.body(i).name for i in range(model.nbody)
                if model.body_parentid[i] == flange and i != flange]
    assert children == [prefix + 'hand']
    assert model.ncam == 0
    assert model.opt.noslip_iterations == openarm_loader.NOSLIP_ITERATIONS
