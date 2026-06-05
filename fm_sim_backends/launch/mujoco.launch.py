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

# Vendored MJCF path inside the container (src/external is gitignored, mounted at
# /ws). Override with mujoco_model:= for an out-of-container layout.
_DEFAULT_MJCF = "/ws/src/external/openarm_mujoco/v2/openarm_bimanual.xml"


def generate_launch_description():
    robot_description = LaunchConfiguration("robot_description")
    controllers_file = LaunchConfiguration("controllers_file")
    mujoco_model = LaunchConfiguration("mujoco_model")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_description",
                description="Robot description XML with the MujocoSystemInterface "
                "<ros2_control> system.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                description="Path to the controllers.yaml for the active preset.",
            ),
            DeclareLaunchArgument(
                "mujoco_model",
                default_value=_DEFAULT_MJCF,
                description="MJCF scene to simulate. Defaults to the vendored "
                "openarm_mujoco v2 bimanual model.",
            ),
            # controller_manager hosted inside MuJoCo. Steps physics + serves the
            # hardware interfaces; controllers are spawned separately by sim.launch.py.
            Node(
                package="mujoco_ros2_control",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description},
                    {"mujoco_model": mujoco_model},
                    controllers_file,
                ],
                output="screen",
            ),
        ]
    )
