"""run.launch.py's launch roles, through its own setup function, without an engine.

Resolves the real OpenArm entry with a controller that activates ``servo`` and
checks what the launch would start: Servo waits on the active spawner, gets the
run's robot description and ``sim_only:=true``, and a failed spawner shuts the
run down instead.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from fm_sim_bringup import catalog, run
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest

SERVO_CONTROLLER = {
    'name': 'servo_probe',
    'description': 'Test controller that activates the servo role',
    'available': True,
    'activate': ['servo'],
    'launch': None,
}


def _run_launch():
    path = os.path.join(
        get_package_share_directory('fm_sim_bringup'), 'launch', 'run.launch.py'
    )
    spec = importlib.util.spec_from_file_location('run_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def setup_actions(monkeypatch):
    """Return run.launch.py's actions for openarm on mujoco with ``controller``."""
    resolve = run.resolve

    def with_controller(controller):
        def resolve_with(**choice):
            entries = resolve(**dict(choice, controller='idle'))
            entries['controller'] = controller
            return entries
        monkeypatch.setattr(run, 'resolve', resolve_with)
        context = LaunchContext()
        context.launch_configurations.update({
            'backend': 'mujoco', 'robot': 'openarm', 'scenario': 'empty',
            'controller': controller['name'], 'use_sim': 'true',
        })
        return _run_launch()._launch_setup(context)

    return with_controller


def _process_exited(action, returncode=0):
    return ProcessExited(
        action=action, name='spawner', cmd=['spawner'], cwd=None, env=None, pid=1,
        returncode=returncode,
    )


def _launch_args(include):
    def text(value):
        return perform_substitutions(
            LaunchContext(), normalize_to_list_of_substitutions(value)
        )
    return {text(key): text(value) for key, value in include.launch_arguments}


def _exit_handler(actions):
    (handler,) = [a for a in actions if isinstance(a, RegisterEventHandler)]
    assert isinstance(handler.event_handler, OnProcessExit)
    return handler


def test_servo_starts_after_the_active_spawner(setup_actions):
    actions = setup_actions(SERVO_CONTROLLER)
    backend, active_spawner = actions[0], actions[1]
    assert isinstance(active_spawner, Node)
    handler = _exit_handler(actions).event_handler
    assert handler.matches(_process_exited(active_spawner))
    (servo,) = handler.handle(_process_exited(active_spawner, 0), LaunchContext())
    assert isinstance(servo, IncludeLaunchDescription)
    source = servo.launch_description_source
    source.get_launch_description(LaunchContext())
    assert source.location == catalog.share_file(
        {'package': 'fm_control', 'path': 'launch/openarm_sim_servo.launch.py'}
    )
    args = _launch_args(servo)
    assert args['sim_only'] == 'true'
    assert args['command_type'] == 'joint_jog'
    assert args['robot_description'] == _launch_args(backend)['robot_description']


def test_failed_spawner_shuts_the_run_down(setup_actions):
    actions = setup_actions(SERVO_CONTROLLER)
    handler = _exit_handler(actions).event_handler
    (shutdown,) = handler.handle(_process_exited(actions[1], 1), LaunchContext())
    assert isinstance(shutdown, Shutdown)


def test_no_launch_role_registers_no_handler(setup_actions):
    actions = setup_actions(dict(SERVO_CONTROLLER, activate=['arm_trajectory']))
    assert not [a for a in actions if isinstance(a, RegisterEventHandler)]


def test_missing_launch_file_fails_at_setup(setup_actions, monkeypatch):
    role_launches = run.role_launches

    def missing_package(robot, controller):
        return [
            dict(role, launch={'package': 'no_such_package', 'path': 'x.launch.py'})
            for role in role_launches(robot, controller)
        ]
    monkeypatch.setattr(run, 'role_launches', missing_package)
    with pytest.raises(Exception, match='no_such_package'):
        setup_actions(SERVO_CONTROLLER)


def test_unknown_role_fails_the_launch(setup_actions):
    with pytest.raises(catalog.CatalogError, match='activates teleop'):
        setup_actions(dict(SERVO_CONTROLLER, activate=['teleop']))
