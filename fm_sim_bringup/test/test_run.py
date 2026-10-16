"""The rules run.launch.py applies to the four catalog entries."""

from fm_sim_bringup import catalog, run
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


def test_openarm_idle_loads_every_controller_once():
    entries = run.resolve('mujoco', 'openarm', 'empty', 'idle')
    active, inactive = run.controller_sets(entries['robot'], entries['controller'])
    assert active == ['joint_state_broadcaster']
    assert len(inactive) == len(set(inactive)) == 6


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
