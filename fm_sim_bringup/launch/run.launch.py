r"""Run one simulation: backend x robot x scenario x controller.

Every dimension is a catalog entry registered in the ament index (see
``fm_sim_bringup.catalog``); an unknown or unavailable value fails before anything
starts, naming the registered ones. ``ros2 run fm_sim_bringup catalog`` lists them.

    ros2 launch fm_sim_bringup run.launch.py \
        backend:=mujoco robot:=openarm scenario:=empty controller:=idle

What comes up:

1. the robot description: the robot entry's xacro, expanded with the robot's args
   plus the backend's (which select the hardware plugin);
2. the backend launch (given the backend's ``launch_args``), which publishes the
   description and hosts the controller_manager;
3. the controller spawners: broadcasters and the roles the controller activates
   start active, the robot's other controllers load inactive;
4. each launch role the controller activates (the robot's ``launch_roles``, such
   as OpenArm's ``servo``), once the active spawner has exited cleanly, given the
   role's ``launch_args`` and the run's ``robot_description``; a failed spawner
   shuts the run down instead;
5. the controller entry's launch, if it has one, given ``use_sim``,
   ``robot_profile`` and ``scenario``;
6. for a scenario with objects, the scenario node (fm_sim_scenarios), serving
   ``/sim/objects``, ``/sim/markers``, ``/sim/reset`` and ``/sim/outcome``;
7. on a backend that renders cameras (mujoco), one ``sim_camera`` node
   (fm_sim_cameras) per role of the robot's ``cameras_file``, publishing
   ``/cam_<role>/image_raw/compressed`` and ``/cam_<role>/camera_info``.

On a backend with a scenario generator (mujoco, isaac), the scenario's objects are
first spawned from ``seed`` into a generated world: on mujoco the robot's model is
built into it with the robot's cameras, and that world is the model the backend
loads; on isaac it is the scenario half of the stage, which the backend launch
(given ``world`` and the robot's ``cameras_file``) completes from the robot
description. ``timeout_s`` overrides the scenario's episode timeout; empty keeps
it. The world is written to a temporary directory that is removed when the launch
shuts down.
"""

import shutil
import tempfile

from fm_sim_bringup import catalog, run
from fm_sim_models.models import mjcf_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.logging
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


# Spawners wait this long for the controller_manager; a software-GL engine on a
# small host can take most of it to come up.
_CONTROLLER_MANAGER_TIMEOUT_S = '120'

_logger = launch.logging.get_logger('run')


def _spawner(controllers, controllers_file, inactive=False):
    arguments = [
        *controllers,
        '--param-file', controllers_file,
        '--controller-manager-timeout', _CONTROLLER_MANAGER_TIMEOUT_S,
    ]
    if inactive:
        arguments.append('--inactive')
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=arguments,
        output='screen',
    )


def _after_active(launches, robot_description):
    """Return the active spawner's exit handler: start ``launches`` or shut down.

    ``launches`` holds ``(launch file path, launch_args)`` pairs, resolved at setup.
    A launch role (Servo, say) streams to controllers the spawner activates, so it
    starts only once they are active; a spawner that failed leaves them down.
    """
    def handler(event, context):
        if event.returncode != 0:
            reason = (
                f'controller spawner exited {event.returncode}; not starting the '
                'launch roles that need its controllers'
            )
            _logger.error(reason)
            return [Shutdown(reason=reason)]
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(path),
                launch_arguments={**args, 'robot_description': robot_description}.items(),
            )
            for path, args in launches
        ]
    return handler


def _launch_setup(context):
    choice = {
        dimension: LaunchConfiguration(dimension).perform(context)
        for dimension in ('backend', 'robot', 'scenario', 'controller')
    }
    use_sim = LaunchConfiguration('use_sim').perform(context)
    if use_sim not in ('true', 'false'):
        raise catalog.CatalogError(f"use_sim must be 'true' or 'false', got {use_sim!r}")
    seed = run.parse_seed(LaunchConfiguration('seed').perform(context))
    timeout_s = run.parse_timeout(LaunchConfiguration('timeout_s').perform(context))
    entries = run.resolve(**choice)
    backend, robot, controller = entries['backend'], entries['robot'], entries['controller']
    scenario = entries['scenario']

    active, inactive = run.controller_sets(robot, controller)
    # Resolved now, so a missing launch file fails the run before anything starts.
    launches = [
        (catalog.share_file(role['launch']), role.get('launch_args', {}))
        for role in run.role_launches(robot, controller)
    ]

    robot_mjcf = mjcf_path(robot['mjcf'])
    # The generated world lives for this launch only: the engine has read it by the
    # time the launch shuts down, and the handler removes it then.
    world_dir = tempfile.mkdtemp(prefix='fm_sim_world_')
    controllers_file = catalog.share_file(robot['controllers_file'])
    try:
        # Where the world carries cameras, the description names it before it
        # exists: the cameras are placed from the description.
        world = run.world_file(backend, scenario, seed, world_dir)
        robot_description = xacro.process_file(
            catalog.share_file(robot['description_file']),
            mappings=run.description_args(backend, robot, world or robot_mjcf, controllers_file),
        ).toxml()
        cameras = run.sim_cameras(backend, robot, robot_description, robot_mjcf)
        world = run.world_model(backend, robot, scenario, seed, robot_mjcf, world_dir, cameras)
    except Exception:
        # The launch fails before the shutdown handler exists; remove the world here.
        shutil.rmtree(world_dir, ignore_errors=True)
        raise
    active_spawner = _spawner(active, controllers_file)
    actions = [
        RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: shutil.rmtree(world_dir, ignore_errors=True)
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(catalog.share_file(backend['launch'])),
            launch_arguments=run.backend_launch_arguments(
                backend, robot, robot_description, controllers_file, world
            ).items(),
        ),
        active_spawner,
    ]
    if launches:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=active_spawner,
            on_exit=_after_active(launches, robot_description),
        )))
    if inactive:
        actions.append(_spawner(inactive, controllers_file, inactive=True))
    if controller['launch']:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(catalog.share_file(controller['launch'])),
                launch_arguments={
                    'use_sim': use_sim,
                    'robot_profile': robot['robot_profile'],
                    'scenario': choice['scenario'],
                }.items(),
            )
        )
    for camera in cameras:
        actions.append(Node(
            package='fm_sim_cameras',
            executable='sim_camera',
            name=f'sim_camera_{camera.role}',
            parameters=[run.camera_node_parameters(camera)],
            output='screen',
        ))
    scenario_parameters = run.scenario_node_parameters(backend, scenario, seed, timeout_s)
    if scenario_parameters:
        actions.append(Node(
            package='fm_sim_scenarios',
            executable='scenario_node',
            parameters=[scenario_parameters],
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'backend', default_value='mujoco',
                description='Sim backend (catalog fm_sim.backends).',
            ),
            DeclareLaunchArgument(
                'robot', default_value='openarm',
                description='Robot (catalog fm_sim.robots).',
            ),
            DeclareLaunchArgument(
                'scenario', default_value='empty',
                description='Scenario (catalog fm_sim.scenarios).',
            ),
            DeclareLaunchArgument(
                'controller', default_value='idle',
                description='Controller under test (catalog fm_sim.controllers).',
            ),
            DeclareLaunchArgument(
                'use_sim', default_value='true',
                description="'true' or 'false'; passed to the controller's launch.",
            ),
            DeclareLaunchArgument(
                'seed', default_value='0',
                description='Spawn seed, an integer in [0, 2^32 - 1].',
            ),
            DeclareLaunchArgument(
                'timeout_s', default_value='',
                description="Episode timeout in seconds; empty keeps the scenario's.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
