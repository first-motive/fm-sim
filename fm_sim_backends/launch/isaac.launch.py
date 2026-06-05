"""Isaac Sim backend for the OpenArm — Linux + NVIDIA, bridged over ROS topics.

Isaac Sim runs as its own process/container (on the GPU host) and is the physics
authority. The connection is purely topic-based: the description's <ros2_control>
system uses topic_based_ros2_control/TopicBasedSystem, and a standalone
ros2_control_node here hosts the controller_manager and bridges through it.

Topic contract (Isaac Sim side must match):
    /isaac_joint_states    sensor_msgs/JointState   Isaac  -> ros2_control (state)
    /isaac_joint_commands  sensor_msgs/JointState   ros2_control -> Isaac (command)

Override the topic names with the matching xacro args when building the description
(isaac_joint_commands_topic / isaac_joint_states_topic). Not verifiable on the Mac;
runs on the Linux/NVIDIA host via scripts/sim.sh --backend isaac.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
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
                description="Robot description XML with the TopicBasedSystem "
                "<ros2_control> system.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                description="Path to the controllers.yaml for the active preset.",
            ),
            LogInfo(
                msg="Isaac backend: ensure Isaac Sim publishes /isaac_joint_states "
                "and subscribes /isaac_joint_commands (sensor_msgs/JointState)."
            ),
            # Standalone controller_manager; the TopicBasedSystem inside bridges to
            # the externally-running Isaac Sim over the topics above.
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description},
                    controllers_file,
                ],
                output="screen",
            ),
        ]
    )
