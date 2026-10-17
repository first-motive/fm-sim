"""The MuJoCo generator: deterministic per seed, and a world MuJoCo loads.

Two loaders check the output: ``compile`` from mujoco_vendor, the MuJoCo build
mujoco_ros2_control runs, and the mujoco Python binding (requirements-sim.txt),
which also steps the world. The world is written away from the robot model, so
the robot's relative meshdir must still resolve.
"""

import math
import os
import subprocess

from ament_index_python.packages import get_package_prefix
from conftest import ARM_MJCF
from fm_sim_scenarios import mujoco_world, scenario as sc
import pytest

MOUNT = [0.0, 0.0, 0.698]


def _compile_binary():
    return os.path.join(
        get_package_prefix('mujoco_vendor'), 'opt', 'mujoco_vendor', 'bin', 'compile'
    )


def test_same_seed_gives_the_same_bytes(can_on_table):
    first = mujoco_world.generate(ARM_MJCF, MOUNT, can_on_table, 5)
    assert first == mujoco_world.generate(ARM_MJCF, MOUNT, can_on_table, 5)
    assert first != mujoco_world.generate(ARM_MJCF, MOUNT, can_on_table, 6)


def test_robot_path_must_be_absolute(can_on_table):
    with pytest.raises(sc.ScenarioError, match='absolute'):
        mujoco_world.generate('arm.xml', MOUNT, can_on_table, 0)


def test_write_names_the_file_by_scenario_and_seed(can_on_table, tmp_path):
    path = mujoco_world.write(ARM_MJCF, MOUNT, can_on_table, 9, str(tmp_path))
    assert path == str(tmp_path / 'can_on_table_9.xml')
    with open(path) as world_file:
        assert world_file.read() == mujoco_world.generate(
            ARM_MJCF, MOUNT, can_on_table, 9, 'fm_sim_can_on_table'
        )


def test_write_refuses_a_name_that_is_not_a_frame_name(can_on_table, tmp_path):
    can_on_table['name'] = '../escape'
    with pytest.raises(sc.ScenarioError, match='not a valid name'):
        mujoco_world.write(ARM_MJCF, MOUNT, can_on_table, 0, str(tmp_path))
    assert list(tmp_path.iterdir()) == []


def test_engine_build_compiles_the_world(can_on_table, tmp_path):
    path = mujoco_world.write(ARM_MJCF, MOUNT, can_on_table, 1, str(tmp_path))
    result = subprocess.run(
        [_compile_binary(), path, str(tmp_path / 'world.mjb')],
        capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert (tmp_path / 'world.mjb').stat().st_size > 0


@pytest.fixture
def mujoco():
    return pytest.importorskip('mujoco', reason='mujoco Python binding (requirements-sim.txt)')


def _load(mujoco, can_on_table, seed, tmp_path):
    path = mujoco_world.write(ARM_MJCF, MOUNT, can_on_table, seed, str(tmp_path))
    model = mujoco.MjModel.from_xml_path(path)
    return model, mujoco.MjData(model)


def test_world_holds_the_robot_and_objects(mujoco, can_on_table, tmp_path):
    model, data = _load(mujoco, can_on_table, 11, tmp_path)
    mujoco.mj_forward(model, data)
    # The robot is mounted, unchanged.
    assert data.body('arm_base').xpos == pytest.approx(MOUNT)
    assert model.joint('arm_joint').range == pytest.approx([-1.5, 1.5])
    # Static objects are world geoms; the others free bodies at their seeded spawn.
    table = model.geom('table')
    assert table.bodyid[0] == 0
    assert table.size == pytest.approx([0.35, 0.6, 0.02])
    free = [model.body(model.jnt_bodyid[j]).name for j in range(model.njnt)
            if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    assert free == ['can', 'ball']
    for name, (xyz, quat) in sc.spawn_poses(can_on_table, 11).items():
        x, y, z, w = quat
        assert data.body(name).xpos == pytest.approx(xyz)
        assert data.body(name).xquat == pytest.approx((w, x, y, z))
    can = model.geom('can')
    assert can.size[:2] == pytest.approx([0.033, 0.0675])
    assert model.body('can').mass[0] == pytest.approx(0.35)
    assert can.friction == pytest.approx([1.0, 0.005, 0.0001])
    assert can.priority[0] == mujoco_world.OBJECT_PRIORITY
    assert can.condim[0] == mujoco_world.FREE_OBJECT_CONDIM


def test_objects_settle_on_the_table(mujoco, can_on_table, tmp_path):
    model, data = _load(mujoco, can_on_table, 2, tmp_path)
    spawn = sc.spawn_poses(can_on_table, 2)
    mujoco.mj_step(model, data, nstep=int(1.0 / model.opt.timestep))
    table_top = 0.207
    assert data.body('can').xpos[2] == pytest.approx(table_top + 0.135 / 2, abs=1e-3)
    assert data.body('can').xpos[:2] == pytest.approx(spawn['can'][0][:2], abs=2e-3)
    assert data.body('ball').xpos[2] == pytest.approx(table_top + 0.02, abs=1e-3)


# A camera on the arm's moving link, looking along the link's +x (optical z), and
# one fixed in the world looking straight down. Optical frame: z forward, y down.
LINK_CAMERA = mujoco_world.Camera(
    'cam_wrist_right', 'arm_link', (0.01, 0.02, 0.03), (0.5, -0.5, 0.5, -0.5), 60.0, (64, 48)
)
WORLD_CAMERA = mujoco_world.Camera(
    'cam_chest', None, (0.1, 0.0, 1.5), (0.0, 1.0, 0.0, 0.0), 45.0, (32, 24)
)


def test_write_with_cameras_attaches_a_robot_copy(can_on_table, tmp_path):
    path = mujoco_world.write(
        ARM_MJCF, MOUNT, can_on_table, 2, str(tmp_path), [LINK_CAMERA, WORLD_CAMERA]
    )
    copy = tmp_path / 'can_on_table_2_robot.xml'
    with open(path) as world_file:
        world = world_file.read()
    assert f'file="{copy}"' in world
    assert '<camera name="cam_chest"' in world
    robot = copy.read_text()
    assert 'own_camera' not in robot
    assert '<camera name="cam_wrist_right"' in robot
    assert 'cam_chest' not in robot
    arm_dir = os.path.dirname(ARM_MJCF)
    assert f'meshdir="{os.path.join(arm_dir, "assets")}"' in robot


def test_robot_copy_refuses_a_missing_body(can_on_table, tmp_path):
    lost = mujoco_world.Camera('cam_chest', 'no_such_body', (0, 0, 0), (1, 0, 0, 0), 45.0, (8, 8))
    with pytest.raises(sc.ScenarioError, match="0 bodies named 'no_such_body'"):
        mujoco_world.write(ARM_MJCF, MOUNT, can_on_table, 0, str(tmp_path), [lost])


def test_robot_copy_refuses_includes(tmp_path):
    model = tmp_path / 'with_include.xml'
    model.write_text('<mujoco><include file="other.xml"/><worldbody/></mujoco>')
    with pytest.raises(sc.ScenarioError, match='<include>'):
        mujoco_world.robot_with_cameras(str(model), [])


def test_generate_refuses_a_body_camera(can_on_table):
    with pytest.raises(sc.ScenarioError, match='rides on a body'):
        mujoco_world.generate(ARM_MJCF, MOUNT, can_on_table, 0, world_cameras=[LINK_CAMERA])


def test_engine_build_compiles_a_world_with_cameras(can_on_table, tmp_path):
    path = mujoco_world.write(
        ARM_MJCF, MOUNT, can_on_table, 1, str(tmp_path), [LINK_CAMERA, WORLD_CAMERA]
    )
    result = subprocess.run(
        [_compile_binary(), path, str(tmp_path / 'world.mjb')],
        capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_cameras_look_along_their_optical_axis(mujoco, can_on_table, tmp_path):
    path = mujoco_world.write(
        ARM_MJCF, MOUNT, can_on_table, 3, str(tmp_path), [LINK_CAMERA, WORLD_CAMERA]
    )
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    assert model.ncam == 2  # the model's own camera is gone
    data.qpos[model.joint('arm_joint').qposadr[0]] = 0.7
    mujoco.mj_forward(model, data)

    def forward(name):
        # MuJoCo cameras look down their -z; the optical z is the view direction.
        return -data.cam(name).xmat.reshape(3, 3)[:, 2], data.cam(name).xmat.reshape(3, 3)[:, 1]

    view, up = forward('cam_chest')
    assert view == pytest.approx([0.0, 0.0, -1.0], abs=1e-9)
    assert data.cam('cam_chest').xpos == pytest.approx(WORLD_CAMERA.pos)
    # The link camera's optical z is the link's +x, its optical y (down) the
    # link's -z, so MuJoCo's up is +z; the link has turned 0.7 rad about z.
    view, up = forward('cam_wrist_right')
    assert view == pytest.approx([math.cos(0.7), math.sin(0.7), 0.0], abs=1e-9)
    assert up == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)
    assert model.cam('cam_wrist_right').fovy[0] == pytest.approx(60.0)
    assert list(model.cam_resolution[model.cam('cam_wrist_right').id]) == [64, 48]


def test_robot_copy_resolves_every_asset_path_as_mujoco_would(tmp_path):
    import xml.etree.ElementTree as ET
    model = tmp_path / 'robot.xml'
    # meshdir falls back to assetdir; a later compiler overrides texturedir; a
    # nested model file is relative to the model's directory.
    model.write_text(
        '<mujoco><compiler assetdir="assets" texturedir="old"/><compiler texturedir="tex"/>'
        '<asset><model name="hand" file="parts/hand.xml"/></asset>'
        '<worldbody><body name="b"/></worldbody></mujoco>'
    )
    root = ET.fromstring(mujoco_world.robot_with_cameras(str(model), []))
    (compiler, *rest) = root.findall('compiler')
    assert compiler.get('assetdir') == str(tmp_path / 'assets')
    assert compiler.get('meshdir') == str(tmp_path / 'assets')
    assert compiler.get('texturedir') == str(tmp_path / 'tex')
    assert all('texturedir' not in other.attrib for other in rest)
    assert root.find('asset/model').get('file') == str(tmp_path / 'parts' / 'hand.xml')


def test_robot_copy_without_compiler_dirs_resolves_from_the_model_directory(tmp_path):
    import xml.etree.ElementTree as ET
    model = tmp_path / 'bare.xml'
    model.write_text('<mujoco><worldbody><body name="b"/></worldbody></mujoco>')
    root = ET.fromstring(mujoco_world.robot_with_cameras(str(model), []))
    compiler = root.find('compiler')
    for key in ('assetdir', 'meshdir', 'texturedir'):
        assert os.path.normpath(compiler.get(key)) == str(tmp_path)
