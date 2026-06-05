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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
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
