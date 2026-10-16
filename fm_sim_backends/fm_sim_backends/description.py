"""The robot description pieces every backend launch shares."""

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def robot_description_parameter():
    """Return the ``robot_description`` launch argument as a string parameter.

    value_type=str: the description is XML, not yaml, so the parameter loader
    must not try to parse it.
    """
    return ParameterValue(LaunchConfiguration('robot_description'), value_type=str)


def robot_state_publisher_node(robot_description):
    """Return the robot_state_publisher that publishes ``/robot_description``.

    The Jazzy controller_manager reads the description from that topic and never
    from a node parameter, so every backend runs one.
    """
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
