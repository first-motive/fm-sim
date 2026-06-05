"""Gazebo (gz-sim) sim backend for the OpenArm — Linux/GPU via the linux overlay.

The robot_description carries two gazebo-specific pieces (emitted by the xacro for
this backend): the gz_ros2_control/GazeboSimSystem in each <ros2_control> block, and
a <gazebo> world plugin (gz_ros2_control-system) that hosts the controller_manager
inside the sim and loads the controllers.yaml. This launch starts gz-sim, spawns the
robot from /robot_description, and bridges the sim clock. sim.launch.py then spawns
the controllers against the controller_manager the plugin brings up.

Not verifiable on the Mac (no GPU); runs on the Linux/GPU host via scripts/sim.sh
--backend gazebo, which selects the linux compose overlay.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_args = LaunchConfiguration("gz_args")
    entity_name = LaunchConfiguration("entity_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_args",
                default_value="-r -v 3 empty.sdf",
                description="Arguments passed to gz-sim (world file + run/verbosity).",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="openarm",
                description="Name for the spawned robot entity.",
            ),
            # gz-sim server + GUI (GUI shown only with a display; headless on a
            # server host).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                    )
                ),
                launch_arguments={"gz_args": gz_args}.items(),
            ),
            # Spawn the robot from the description robot_state_publisher publishes.
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", entity_name, "-topic", "robot_description"],
                output="screen",
            ),
            # Bridge the sim clock so ROS controllers run on sim time.
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            ),
        ]
    )
