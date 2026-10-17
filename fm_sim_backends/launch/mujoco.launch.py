"""MuJoCo sim backend for the OpenArm — the Mac daily driver (CPU, no GPU).

mujoco_ros2_control's ros2_control_node hosts the controller_manager *inside* the
MuJoCo simulation: it loads the robot_description (whose <ros2_control> System plugin
is mujoco_ros2_control/MujocoSystemInterface), steps the MJCF physics, and exposes
the hardware interfaces the controllers drive. The launch that includes this one
spawns the controllers against the controller_manager it brings up.

On Jazzy the controller_manager reads the description from the
``/robot_description`` topic, not a node parameter, so this launch runs the
robot_state_publisher that publishes it. The description comes from fm-robot's
``fm_control/urdf/openarm.sim.urdf.xacro`` with ``sim_backend:=mujoco`` and the
``mujoco_model`` arg set to ``fm_sim_models.mjcf_path('openarm')``: the vendored
openarm_mujoco v2 model, whose joint names (openarm_left_*, openarm_right_*) match
the description exactly. The model is bimanual; pair it with the default_bimanual
preset for a full joint map (a single-arm preset leaves the other arm passive).

``config/mujoco_plugins.yaml`` loads mujoco_ros2_control's free-joint state
publisher, so every loose object's pose is on ``/mujoco/free_joint_states``.
"""

import os
import platform
from shutil import which

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

from fm_sim_backends.description import (
    robot_description_parameter,
    robot_state_publisher_node,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _display_prefix():
    """Return the launch prefix that gives MuJoCo's GLFW viewer a display.

    X11 only. On Linux the viewer needs a display that a headless container/host has
    no session for, so it runs under a virtual one (``xvfb-run``). macOS has no X11:
    GLFW talks to Cocoa directly and a real window opens on the logged-in session, so
    the prefix is empty there — asking for xvfb-run would just fail to find it.

    xvfb-run runs the node in the foreground of a ``/bin/sh`` that defers launch's
    SIGINT and, killed by the SIGTERM that follows, leaves the node and Xvfb running:
    the controller_manager outlived shutdown, and the next launch's spawners found
    it. The bash wrapper runs xvfb-run as a job (``set -m``, so the node does not
    start with SIGINT ignored) and passes each shutdown signal to xvfb-run's children
    except Xvfb. The node stops first; xvfb-run then stops Xvfb, as on a normal exit,
    and returns the node's exit status. Signalling Xvfb in the same instant would cut
    the node's X connection in the middle of its shutdown.
    """
    if platform.system() == 'Darwin':
        return ''
    return (
        "bash -c '"
        'set -m; xvfb-run -a "$@" & job=$!; '
        'forward() { for pid in $(pgrep -P $job); do '
        '[ "$(cat /proc/$pid/comm)" = Xvfb ] || kill -$1 $pid; done; }; '
        'trap "forward INT" INT; trap "forward TERM" TERM; '
        "wait $job; wait $job' xvfb-run"
    )


def _require_backend():
    """Fail early with a clear pointer when the MuJoCo backend can't run here.

    The backend hosts the controller_manager inside ``mujoco_ros2_control``. The
    fm-sim image installs it from the Jazzy apt mirror; a native host needs the
    same package. Absent, the launch would die deep with a cryptic "package not
    found"; say so up front.

    xvfb-run is required only where it is actually used — see ``_display_prefix``.
    """
    missing = []
    try:
        get_package_share_directory('mujoco_ros2_control')
    except PackageNotFoundError:
        missing.append(
            'the mujoco_ros2_control package (apt install ros-jazzy-mujoco-ros2-control)'
        )
    if _display_prefix() and which('xvfb-run') is None:
        missing.append('xvfb-run (the virtual display)')
    if missing:
        raise RuntimeError(
            'The MuJoCo sim backend needs ' + ' and '.join(missing) + '.'
        )


def generate_launch_description():
    # Guard before building the description, so a missing backend stops with an
    # actionable message rather than a cryptic one deep in the launch.
    _require_backend()

    robot_description = robot_description_parameter()
    controllers_file = LaunchConfiguration('controllers_file')
    plugins_file = os.path.join(
        get_package_share_directory('fm_sim_backends'), 'config', 'mujoco_plugins.yaml'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_description',
                description='Robot description XML with the MujocoSystemInterface '
                '<ros2_control> system (carries the mujoco_model param).',
            ),
            DeclareLaunchArgument(
                'controllers_file',
                description='Path to the controllers.yaml for the active preset.',
            ),
            # Publishes /robot_description, which the controller_manager below
            # subscribes to on Jazzy. The caller must not run its own (see the
            # package README).
            robot_state_publisher_node(robot_description),
            # controller_manager hosted inside MuJoCo. Steps physics + serves the
            # hardware interfaces; the caller spawns the controllers.
            # The viewer's display comes from _display_prefix (xvfb-run on Linux, the
            # Cocoa session on macOS). MJCF path comes from the mujoco_model param.
            Node(
                package='mujoco_ros2_control',
                executable='ros2_control_node',
                prefix=_display_prefix(),
                # Jazzy reads the description from the topic and ignores this
                # parameter; fm-app's Humble fm_bringup still includes this
                # backend and needs it until that launch moves to Jazzy.
                parameters=[
                    {'robot_description': robot_description}, controllers_file, plugins_file,
                ],
                output='screen',
            ),
        ]
    )
