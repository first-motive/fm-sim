"""What the isaac launch starts beside the controller_manager for a stage.

Kept apart from the launch file so the wiring unit-tests without a launch run
or Isaac Sim: the stage spec written for the run, the stage runner command, and
per camera the compression and camera_info relays.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from fm_sim_scenarios import isaac_stage
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
import yaml

RUNNER = os.path.join('isaac', 'fm_sim_isaac.py')
# Isaac Sim's own ROS 2 node discovers only on this host, as the agent's run
# containers do (agent/src/fm_sim_agent/runner.py DEFAULT_DDS_RANGE).
RUNNER_ENV = {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
# Shown in the logged command when no isaac_python is set.
ISAAC_PYTHON_PLACEHOLDER = '$ISAAC_SIM/python.sh'


def load_cameras(cameras_file):
    """Return the checked cameras of a robot's cameras file; ``{}`` for no file."""
    if not cameras_file:
        return {}
    with open(cameras_file) as camera_yaml:
        return isaac_stage.check_cameras(yaml.safe_load(camera_yaml))


def runner_command(stage_path, isaac_python, gui):
    """Return the command that runs the stage runner in Isaac Sim's Python."""
    runner = os.path.join(get_package_share_directory('fm_sim_backends'), RUNNER)
    command = [isaac_python, runner, '--spec', stage_path]
    if gui:
        command.append('--gui')
    return command


def camera_relays(cameras):
    """Return the nodes that turn each raw camera image into the contract's topics.

    image_transport's republisher compresses ``/cam_<role>/image_raw`` onto
    ``/cam_<role>/image_raw/compressed``; one camera_info node answers every image
    with the camera's real intrinsics on ``/cam_<role>/camera_info``.
    """
    if not cameras:
        return []
    relays = [
        Node(
            package='image_transport',
            executable='republish',
            name=f'cam_{role}_compress',
            parameters=[{'in_transport': 'raw', 'out_transport': 'compressed'}],
            # The compressed publisher's own name is remapped: on Jazzy a remap of
            # the base name 'out' leaves it on /out/compressed (checked live).
            remappings=[
                ('in', f'/cam_{role}/image_raw'),
                ('out/compressed', f'/cam_{role}/image_raw/compressed'),
            ],
            # Host-local like the runner, so no LAN peer can feed fake frames.
            additional_env=RUNNER_ENV,
            output='screen',
        )
        for role in sorted(cameras)
    ]
    relays.append(Node(
        package='fm_sim_backends',
        executable='camera_info',
        parameters=[{'cameras': json.dumps(cameras, sort_keys=True)}],
        additional_env=RUNNER_ENV,
        output='screen',
    ))
    return relays


def load_mesh_roots(text):
    """Return the launch's JSON ``mesh_roots`` as ``[(prefix, directory)]``."""
    try:
        roots = json.loads(text or '[]')
    except json.JSONDecodeError as error:
        raise isaac_stage.StageError(f'mesh_roots is not JSON: {text!r}') from error
    return isaac_stage.parse_mesh_roots(roots)


def stage_actions(world, robot_description, cameras_file, mesh_roots, isaac_python, gui):
    """Return the actions for a run's stage, or only a notice when there is none.

    ``world`` is the scenario half ``isaac_stage.write`` produced (empty when the
    launch is used without ``run.launch.py``). With ``isaac_python`` (Isaac Sim's
    ``python.sh``) the stage runner starts here; without it the command to run
    is logged. ``mesh_roots`` is the launch's JSON ``[[prefix, directory]]``.
    """
    if not world:
        return [LogInfo(
            msg='Isaac backend: no world given, so no stage; Isaac Sim must publish '
            "and subscribe the description's joint topics itself."
        )]
    cameras = load_cameras(cameras_file)
    stage_path = isaac_stage.write_stage(
        world, robot_description, cameras, load_mesh_roots(mesh_roots),
        get_package_share_directory,
    )
    actions = camera_relays(cameras)
    if isaac_python:
        command = runner_command(stage_path, isaac_python, gui)
        actions += [
            LogInfo(msg=f'Isaac backend: starting the stage runner: {" ".join(command)}'),
            ExecuteProcess(cmd=command, additional_env=RUNNER_ENV, output='screen'),
        ]
    else:
        command = ' '.join(runner_command(stage_path, ISAAC_PYTHON_PLACEHOLDER, gui))
        actions.append(LogInfo(
            msg=f'Isaac backend: stage {stage_path}; isaac_python is empty, so Isaac Sim '
            f'is not started here (set FM_ISAAC_PYTHON or isaac_python:=). Run, with '
            f'$ISAAC_SIM your Isaac Sim install: ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST '
            f'{command}'
        ))
    return actions
