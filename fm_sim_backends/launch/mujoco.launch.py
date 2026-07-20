"""MuJoCo sim backend for the OpenArm — the Mac daily driver (CPU, no GPU).

mujoco_ros2_control's ros2_control_node hosts the controller_manager *inside* the
MuJoCo simulation: it loads the robot_description (whose <ros2_control> System plugin
is mujoco_ros2_control/MujocoSystemInterface), steps the MJCF physics, and exposes
the hardware interfaces the controllers drive. ``sim.launch.py`` starts this, then
spawns the controllers against the controller_manager it brings up.

The MJCF is the vendored openarm_mujoco v2 model, loaded in place so its relative
``meshdir="assets"`` resolves. Its joint names (openarm_left_*, openarm_right_*)
match the description exactly, so MujocoSystemInterface maps interfaces by name. The
model is bimanual; pair it with the default_bimanual variant for a full joint map
(a single-arm preset leaves the other arm passive in the scene).
"""

import platform
from shutil import which

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _display_prefix():
    """Return the launch prefix that gives MuJoCo's GLFW viewer a display.

    X11 only. On Linux the viewer needs a display that a headless container/host has
    no session for, so it runs under a virtual one (``xvfb-run``). macOS has no X11:
    GLFW talks to Cocoa directly and a real window opens on the logged-in session, so
    the prefix is empty there — asking for xvfb-run would just fail to find it.
    """
    if platform.system() == "Darwin":
        return ""
    return "xvfb-run -a"


def _require_backend():
    """Fail early with a clear pointer when the MuJoCo backend can't run here.

    The backend hosts the controller_manager inside ``mujoco_ros2_control``, which is
    not on RoboStack — the native path builds it from source (``external.repos`` +
    ``patch-mujoco-ros2-control.sh``), the container gets it from apt. Absent, the
    launch would die deep with a cryptic "package not found"; say so up front.

    xvfb-run is required only where it is actually used — see ``_display_prefix``.
    """
    missing = []
    try:
        get_package_share_directory("mujoco_ros2_control")
    except PackageNotFoundError:
        missing.append("the mujoco_ros2_control package (rebuild: `pixi run build`)")
    if _display_prefix() and which("xvfb-run") is None:
        missing.append("xvfb-run (the virtual display)")
    if missing:
        raise RuntimeError(
            "The MuJoCo sim backend needs " + " and ".join(missing) + "."
        )


def generate_launch_description():
    # Guard before building the description, so a missing backend stops with an
    # actionable message rather than a cryptic one deep in the launch.
    _require_backend()

    # value_type=str: the description is XML, not yaml — stop the param loader
    # from trying to parse it.
    robot_description = ParameterValue(
        LaunchConfiguration("robot_description"), value_type=str
    )
    controllers_file = LaunchConfiguration("controllers_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_description",
                description="Robot description XML with the MujocoSystemInterface "
                "<ros2_control> system (carries the mujoco_model param).",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                description="Path to the controllers.yaml for the active preset.",
            ),
            # controller_manager hosted inside MuJoCo. Steps physics + serves the
            # hardware interfaces; controllers are spawned separately by sim.launch.py.
            # The viewer's display comes from _display_prefix (xvfb-run on Linux, the
            # Cocoa session on macOS). MJCF path comes from the mujoco_model param.
            Node(
                package="mujoco_ros2_control",
                executable="ros2_control_node",
                prefix=_display_prefix(),
                parameters=[
                    {"robot_description": robot_description},
                    controllers_file,
                ],
                output="screen",
            ),
        ]
    )
