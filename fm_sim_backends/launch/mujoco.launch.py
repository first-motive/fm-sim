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


def _require_container_backend():
    """Fail early with a clear pointer when the MuJoCo backend can't run here.

    The MuJoCo daily driver hosts the controller_manager inside
    ``mujoco_ros2_control`` and renders under ``xvfb-run`` — both ship only in the
    Linux container image, not the native (pixi/RoboStack) env. Launched natively
    (e.g. the macOS default path) they surface as a cryptic "package not found" or
    "xvfb-run: command not found" deep in the launch. Detect the absence up front
    and point at the container path instead.
    """
    missing = []
    try:
        get_package_share_directory("mujoco_ros2_control")
    except PackageNotFoundError:
        missing.append("the mujoco_ros2_control package")
    if which("xvfb-run") is None:
        missing.append("xvfb-run (the virtual display)")
    if missing:
        raise RuntimeError(
            "The MuJoCo sim backend needs "
            + " and ".join(missing)
            + ", which ship only in the Linux container image, not the native "
            "(pixi/RoboStack) env. Run the container path instead:\n"
            "    ./run.sh --container"
        )


def generate_launch_description():
    # Guard before building the description: native hosts lack the container-only
    # MuJoCo ros2_control host, so stop with an actionable message, not a cryptic one.
    _require_container_backend()

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
            # MuJoCo's GLFW viewer needs a display, so run it under a virtual one
            # (xvfb-run) — this is what makes the headless Mac container the daily
            # driver. The MJCF path comes from the description's mujoco_model param.
            Node(
                package="mujoco_ros2_control",
                executable="ros2_control_node",
                prefix="xvfb-run -a",
                parameters=[
                    {"robot_description": robot_description},
                    controllers_file,
                ],
                output="screen",
            ),
        ]
    )
