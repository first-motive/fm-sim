"""run.launch.py's launch roles, through its own setup function, without an engine.

Resolves the real SO-101 entry, given OpenArm's ``servo`` launch role, with a
controller that activates it, and checks what the launch would start: Servo waits on
the active spawner, gets the run's robot description and its launch args, and a
failed spawner shuts the run down instead. SO-101 stands in for the OpenArm, whose
description is private, so this runs in every image; the setup only has to find
fm-robot's sim Servo launch, not start it.
"""

import importlib.util
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from fm_sim_bringup import catalog, run
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events.process import ProcessExited
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
import pytest

SERVO_ROLE = catalog.load('robots', 'openarm')['launch_roles']['servo']
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
    """Return run.launch.py's actions for so101 with ``SERVO_ROLE`` and ``controller``."""
    resolve = run.resolve
    shutdown_handlers = []

    def with_controller(controller):
        def resolve_with(**choice):
            entries = resolve(**dict(choice, controller='idle'))
            entries['robot']['launch_roles'] = {'servo': SERVO_ROLE}
            entries['robot']['robot_profile'] = 'probe_profile'
            entries['controller'] = controller
            return entries
        monkeypatch.setattr(run, 'resolve', resolve_with)
        context = LaunchContext()
        context.launch_configurations.update({
            'backend': 'mujoco', 'robot': 'so101', 'scenario': 'empty',
            'controller': controller['name'], 'use_sim': 'true',
            'seed': '0', 'timeout_s': '',
        })
        actions = _run_launch()._launch_setup(context)
        shutdown_handlers.extend(_handlers(actions, OnShutdown))
        return actions

    yield with_controller
    # Removes the generated world, as the launch does when it shuts down.
    for handler in shutdown_handlers:
        handler.handle(None, LaunchContext())


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


def _handlers(actions, kind):
    return [
        a.event_handler for a in actions
        if isinstance(a, RegisterEventHandler) and isinstance(a.event_handler, kind)
    ]


def _exit_handler(actions):
    (handler,) = _handlers(actions, OnProcessExit)
    return handler


def _backend_and_active_spawner(actions):
    backend = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    active_spawner = next(a for a in actions if isinstance(a, Node))
    return backend, active_spawner


def test_servo_starts_after_the_active_spawner(setup_actions):
    actions = setup_actions(SERVO_CONTROLLER)
    backend, active_spawner = _backend_and_active_spawner(actions)
    handler = _exit_handler(actions)
    assert handler.matches(_process_exited(active_spawner))
    (servo,) = handler.handle(_process_exited(active_spawner, 0), LaunchContext())
    assert isinstance(servo, IncludeLaunchDescription)
    source = servo.launch_description_source
    source.get_launch_description(LaunchContext())
    assert source.location == catalog.share_file(SERVO_ROLE['launch'])
    args = _launch_args(servo)
    assert args['sim_only'] == 'true'
    assert args['command_type'] == 'joint_jog'
    assert args['names'] == 'loader'
    assert args['robot_description'] == _launch_args(backend)['robot_description']


def test_failed_spawner_shuts_the_run_down(setup_actions):
    actions = setup_actions(SERVO_CONTROLLER)
    _, active_spawner = _backend_and_active_spawner(actions)
    (shutdown,) = _exit_handler(actions).handle(
        _process_exited(active_spawner, 1), LaunchContext()
    )
    assert isinstance(shutdown, Shutdown)


def test_no_launch_role_registers_no_handler(setup_actions):
    actions = setup_actions(dict(SERVO_CONTROLLER, activate=['arm_trajectory']))
    assert not _handlers(actions, OnProcessExit)


def test_missing_launch_file_fails_at_setup(setup_actions, monkeypatch, tmp_path):
    role_launches = run.role_launches

    def missing_package(robot, controller):
        return [
            dict(role, launch={'package': 'no_such_package', 'path': 'x.launch.py'})
            for role in role_launches(robot, controller)
        ]
    monkeypatch.setattr(run, 'role_launches', missing_package)
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    with pytest.raises(Exception, match='no_such_package'):
        setup_actions(SERVO_CONTROLLER)
    # It fails before the generated world exists, so nothing is left behind.
    assert not list(tmp_path.iterdir())


def test_controller_launch_gets_its_launch_args(setup_actions):
    actions = setup_actions(dict(
        SERVO_CONTROLLER, activate=[], launch=SERVO_ROLE['launch'],
        launch_args={'perception': 'model', 'pack': 'can_paper_seg'},
    ))
    (controller,) = [
        a for a in actions if isinstance(a, IncludeLaunchDescription)
        and 'robot_profile' in _launch_args(a)
    ]
    assert _launch_args(controller) == {
        'perception': 'model',
        'pack': 'can_paper_seg',
        'use_sim': 'true',
        'robot_profile': 'probe_profile',
        'scenario': 'empty',
    }


def test_launch_args_refuse_exactly_the_names_the_run_sets():
    declared = [arg.name for arg in _run_launch().generate_launch_description()
                .get_launch_arguments()]
    entries = run.resolve('gazebo', 'so101', 'empty', 'idle')
    # Without its launch_args: those are the backend's own, not the run's.
    backend = dict(entries['backend'], launch_args={})
    sets = {
        *declared,
        *run.backend_launch_arguments(
            backend, entries['robot'], robot_description='', controllers_file='', world=None
        ),
        *run.controller_launch_arguments(entries['controller'], entries['robot'], 'empty', 'true'),
    }
    assert sets == catalog.CONTROLLER_LAUNCH_OWNED


def test_unknown_role_fails_the_launch(setup_actions):
    with pytest.raises(catalog.CatalogError, match='activates teleop'):
        setup_actions(dict(SERVO_CONTROLLER, activate=['teleop']))
