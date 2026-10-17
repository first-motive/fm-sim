"""The rules run.launch.py applies to the four catalog entries."""

import json
import os

from fm_sim_bringup import catalog, run
from fm_sim_models.models import mjcf_path
from fm_sim_scenarios import scenario as sc
import pytest

ROBOT = {
    'name': 'arm',
    'description_args': {'robot_preset': 'right_arm'},
    'controller_roles': {
        'broadcasters': ['joint_state_broadcaster'],
        'arm_trajectory': ['arm_controller'],
        'arm_forward_position': ['forward_position_controller'],
    },
}


def _controller(*roles):
    return {'name': 'test', 'activate': list(roles)}


def test_backend_args_fill_placeholders_over_robot_args():
    backend = {'description_args': {
        'sim_backend': 'mujoco',
        'mujoco_model': '{mjcf_path}',
        'gazebo_controllers_file': '{controllers_file}',
    }}
    assert run.description_args(backend, ROBOT, '/m.xml', '/c.yaml') == {
        'robot_preset': 'right_arm',
        'sim_backend': 'mujoco',
        'mujoco_model': '/m.xml',
        'gazebo_controllers_file': '/c.yaml',
    }


def test_idle_activates_broadcasters_only():
    active, inactive = run.controller_sets(ROBOT, _controller())
    assert active == ['joint_state_broadcaster']
    assert inactive == ['arm_controller', 'forward_position_controller']


def test_controller_activates_its_roles():
    active, inactive = run.controller_sets(ROBOT, _controller('arm_forward_position'))
    assert active == ['joint_state_broadcaster', 'forward_position_controller']
    assert inactive == ['arm_controller']


def test_unknown_role_names_the_robot_roles():
    with pytest.raises(catalog.CatalogError, match='gripper.*arm_forward_position'):
        run.controller_sets(ROBOT, _controller('gripper'))


def test_resolve_refuses_an_unknown_backend():
    with pytest.raises(catalog.CatalogError, match="unknown backend 'bullet'"):
        run.resolve('bullet', 'openarm', 'empty', 'idle')


@pytest.mark.parametrize('robot,controllers', [('openarm', 6), ('so101', 2), ('axol', 2)])
def test_idle_loads_every_controller_once(robot, controllers):
    entries = run.resolve('mujoco', robot, 'empty', 'idle')
    active, inactive = run.controller_sets(entries['robot'], entries['controller'])
    assert active == ['joint_state_broadcaster']
    assert len(inactive) == len(set(inactive)) == controllers


@pytest.mark.parametrize('robot,arm_controllers', [
    ('openarm', ['openarm_left_arm_controller', 'openarm_right_arm_controller']),
    ('so101', ['so101_arm_controller']),
    ('axol', ['axol_left_arm_controller', 'axol_right_arm_controller']),
])
def test_teleop_activates_the_arm_trajectory_controllers(robot, arm_controllers):
    entries = run.resolve('mujoco', robot, 'empty', 'teleop')
    active, inactive = run.controller_sets(entries['robot'], entries['controller'])
    assert active == ['joint_state_broadcaster', *arm_controllers]
    assert not set(active) & set(inactive)


def test_resolve_refuses_a_launch_controller_on_a_robot_without_profile(monkeypatch):
    load = catalog.require_available

    def idle_with_a_launch(kind, name):
        entry = load(kind, name)
        if kind == 'controllers':
            entry['launch'] = {'package': 'fake_pkg', 'path': 'launch/run.launch.py'}
        return entry

    monkeypatch.setattr(catalog, 'require_available', idle_with_a_launch)
    with pytest.raises(
        catalog.CatalogError, match="'idle' needs a robot profile; robot 'so101' has none"
    ):
        run.resolve('mujoco', 'so101', 'empty', 'idle')
    assert run.resolve('mujoco', 'openarm', 'empty', 'idle')['robot']['robot_profile']


def test_resolve_refuses_a_backend_the_robot_does_not_support(monkeypatch):
    load = catalog.require_available

    def mujoco_only_robot(kind, name):
        entry = load(kind, name)
        if kind == 'robots':
            entry['supported_backends'] = ['mujoco']
        return entry

    monkeypatch.setattr(catalog, 'require_available', mujoco_only_robot)
    with pytest.raises(
        catalog.CatalogError, match="not supported on backend 'gazebo' yet; supported: mujoco$"
    ):
        run.resolve('gazebo', 'openarm', 'empty', 'idle')


@pytest.mark.parametrize('text, seed', [('0', 0), ('42', 42), ('4294967295', 2 ** 32 - 1)])
def test_seed_parses(text, seed):
    assert run.parse_seed(text) == seed


@pytest.mark.parametrize('text', ['', '-1', '4294967296', '1.5', 'x'])
def test_seed_refuses_non_uint32(text):
    with pytest.raises(catalog.CatalogError, match='seed must be an integer'):
        run.parse_seed(text)


def test_timeout_empty_keeps_the_scenario_one():
    assert run.parse_timeout('') == 0.0
    assert run.parse_timeout('12.5') == 12.5


@pytest.mark.parametrize('text', ['0', '-3', 'x', 'nan', 'inf', '1e400'])
def test_timeout_refuses_non_positive(text):
    with pytest.raises(catalog.CatalogError, match='timeout_s'):
        run.parse_timeout(text)


def _entries(backend, scenario):
    return run.resolve(backend, 'openarm', scenario, 'idle')


def test_backend_without_generator_runs_the_robot_model_for_empty():
    entries = _entries('gazebo', 'empty')
    world = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 0, '/robot.xml', '/unused'
    )
    assert world is None


def test_backend_without_generator_refuses_objects():
    entries = _entries('gazebo', 'can_on_table')
    with pytest.raises(catalog.CatalogError, match="backend 'gazebo' has no scenario generator"):
        run.world_model(
            entries['backend'], entries['robot'], entries['scenario'], 0, '/robot.xml', '/unused'
        )


@pytest.mark.skipif(
    not os.path.isfile(mjcf_path('openarm')), reason='openarm_mujoco external (vcs import)'
)
def test_mujoco_builds_the_openarm_world(tmp_path):
    entries = _entries('mujoco', 'can_on_table')
    world = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 3, mjcf_path('openarm'),
        str(tmp_path),
    )
    assert world == str(tmp_path / 'can_on_table_3.xml')
    with open(world) as world_file:
        text = world_file.read()
    assert f'file="{mjcf_path("openarm")}"' in text
    assert '<frame name="robot_mount" pos="0 0 0.698">' in text
    (x, y, z), _ = sc.spawn_poses(entries['scenario'], 3)['can']
    assert f'<body name="can" pos="{x:.9g} {y:.9g} {z:.9g}"' in text


def test_scenario_node_runs_only_with_objects():
    empty = _entries('mujoco', 'empty')
    assert run.scenario_node_parameters(empty['backend'], empty['scenario'], 0, 0.0) is None
    entries = _entries('mujoco', 'can_on_table')
    parameters = run.scenario_node_parameters(entries['backend'], entries['scenario'], 7, 30.0)
    assert parameters['backend'] == 'mujoco'
    assert parameters['seed'] == 7
    assert parameters['timeout_s'] == 30.0
    assert parameters['use_sim_time'] is True
    assert json.loads(parameters['scenario']) == entries['scenario']
