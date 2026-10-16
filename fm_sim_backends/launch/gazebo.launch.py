"""Gazebo Harmonic sim backend for the OpenArm — Linux/GPU, or headless anywhere.

The robot_description carries two gazebo-specific pieces, emitted by fm-robot's
``fm_control/urdf/openarm.sim.urdf.xacro`` for ``sim_backend:=gz`` (or ``gazebo``):
the gz_ros2_control/GazeboSimSystem in each <ros2_control> block, and a <gazebo>
world plugin (gz_ros2_control-system) that hosts the controller_manager inside the
sim and loads the ``gazebo_controllers_file`` xacro arg. This launch starts gz-sim
through ros_gz_sim, spawns the robot from /robot_description, and bridges the sim
clock. The launch that includes this one spawns the controllers against the
controller_manager the plugin brings up.
"""

from fm_sim_backends.description import (
    robot_description_parameter,
    robot_state_publisher_node,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_args = LaunchConfiguration('gz_args')
    entity_name = LaunchConfiguration('entity_name')
    robot_description = robot_description_parameter()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'gz_args',
                default_value='-r -v 3 empty.sdf',
                description='Arguments passed to gz-sim (world file + run/verbosity).',
            ),
            DeclareLaunchArgument(
                'entity_name',
                default_value='openarm',
                description='Name for the spawned robot entity.',
            ),
            # robot_description is published below so the entity spawn resolves
            # it; gazebo hosts its controller_manager via the gz_ros2_control
            # plugin embedded in the description, so it loads the controllers.yaml
            # itself — controllers_file is accepted for a uniform backend contract
            # and reserved for an explicit override.
            DeclareLaunchArgument(
                'robot_description',
                description='Robot description XML with the GazeboSimSystem '
                '<ros2_control> system and the gz_ros2_control world plugin.',
            ),
            DeclareLaunchArgument(
                'controllers_file',
                default_value='',
                description='controllers.yaml for the preset (gazebo loads '
                'controllers via the description plugin; reserved for override).',
            ),
            # Publish the description so `create -topic robot_description` resolves
            # it. The caller must not run its own (see the package README).
            robot_state_publisher_node(robot_description),
            # gz-sim server + GUI (GUI shown only with a display; headless on a
            # server host).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
                    )
                ),
                launch_arguments={'gz_args': gz_args}.items(),
            ),
            # Spawn the robot from the description robot_state_publisher publishes.
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', entity_name, '-topic', 'robot_description'],
                output='screen',
            ),
            # Bridge the sim clock so ROS controllers run on sim time.
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
            ),
        ]
    )
