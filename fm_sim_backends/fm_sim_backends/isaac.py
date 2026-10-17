"""What the isaac launch starts beside the controller_manager for a stage.

Kept apart from the launch file so the wiring unit-tests without a launch run
or Isaac Sim: the stage spec written for the run, the stage runner command, and
per camera the compression and camera_info relays (``fm_sim_backends.cameras``).
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from fm_sim_backends.cameras import camera_relays, HOST_LOCAL_ENV, load_cameras
from fm_sim_cameras import cameras_file as cameras_file_module
from fm_sim_scenarios import isaac_stage
from launch.actions import ExecuteProcess, LogInfo

RUNNER = os.path.join('isaac', 'fm_sim_isaac.py')
# Isaac Sim's own ROS 2 node discovers only on this host, as the agent's run
# containers do (agent/src/fm_sim_agent/runner.py DEFAULT_DDS_RANGE).
RUNNER_ENV = HOST_LOCAL_ENV
# Shown in the logged command when no isaac_python is set.
ISAAC_PYTHON_PLACEHOLDER = '$ISAAC_SIM/python.sh'


def runner_command(stage_path, isaac_python, gui):
    """Return the command that runs the stage runner in Isaac Sim's Python."""
    runner = os.path.join(get_package_share_directory('fm_sim_backends'), RUNNER)
    command = [isaac_python, runner, '--spec', stage_path]
    if gui:
        command.append('--gui')
    return command


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
    # Isaac's own rules on top of the shared check (its render rate, its models).
    try:
        loaded = load_cameras(cameras_file)
    except cameras_file_module.CameraError as error:
        raise isaac_stage.StageError(str(error)) from error
    cameras = isaac_stage.check_cameras({'cameras': loaded})
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
