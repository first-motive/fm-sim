"""The MuJoCo generator: deterministic per seed, and a world MuJoCo loads.

Two loaders check the output: ``compile`` from mujoco_vendor, the MuJoCo build
mujoco_ros2_control runs, and the mujoco Python binding (requirements-sim.txt),
which also steps the world. The world is written away from the robot model, so
the robot's relative meshdir must still resolve.
"""

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
