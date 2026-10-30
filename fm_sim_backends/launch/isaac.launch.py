"""Isaac Sim backend for the OpenArm — Linux + NVIDIA, bridged over ROS topics.

Isaac Sim runs as its own process/container (on the GPU host) and is the physics
authority. The connection is purely topic-based: the description's <ros2_control>
system uses topic_based_ros2_control/TopicBasedSystem, and a standalone
ros2_control_node here hosts the controller_manager and bridges through it. On
Jazzy that controller_manager reads the description from the
``/robot_description`` topic, so this launch also runs the robot_state_publisher.

Topic contract (Isaac Sim side must match):
    /isaac_joint_states    sensor_msgs/JointState   Isaac  -> ros2_control (state)
    /isaac_joint_commands  sensor_msgs/JointState   ros2_control -> Isaac (command)

The description comes from fm-robot's ``fm_control/urdf/openarm.sim.urdf.xacro``
with ``sim_backend:=isaac``; override the topic names with its
``isaac_joint_commands_topic`` / ``isaac_joint_states_topic`` args. Not verifiable
on the Mac; runs on the Linux/NVIDIA host.

Under ``run.launch.py`` the ``world`` argument is the scenario half of the stage
(``fm_sim_scenarios.isaac_stage.write``). The launch then writes the full stage
spec beside it (URDF import, scenario objects, cameras from ``cameras_file``,
the ROS 2 bridge graph), starts the camera relays (compressed images and
camera_info), and starts Isaac Sim's stage runner when ``isaac_python`` (default
``$FM_ISAAC_PYTHON``) names Isaac Sim's ``python.sh``; otherwise it logs the
command to run. Isaac Sim must see the same filesystem as this launch.
"""

from fm_sim_backends import isaac
from fm_sim_backends.description import (
    robot_description_parameter,
    robot_state_publisher_node,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def _stage_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    gui = arg('isaac_gui')
    if gui not in ('true', 'false'):
        raise ValueError(f"isaac_gui must be 'true' or 'false', got {gui!r}")
    return isaac.stage_actions(
        arg('world'), arg('robot_description'), arg('cameras_file'), arg('mesh_roots'),
        arg('isaac_python'), gui == 'true',
    )


def generate_launch_description():
    robot_description = robot_description_parameter()
    controllers_file = LaunchConfiguration('controllers_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_description',
                description='Robot description XML with the TopicBasedSystem '
                '<ros2_control> system.',
            ),
            DeclareLaunchArgument(
                'controllers_file',
                description='Path to the controllers.yaml for the active preset.',
            ),
            DeclareLaunchArgument(
                'world', default_value='',
                description='Scenario half of the Isaac stage (isaac_stage.write); '
                'empty builds no stage.',
            ),
            DeclareLaunchArgument(
                'cameras_file', default_value='',
                description="The robot's cameras file; empty renders no cameras.",
            ),
            DeclareLaunchArgument(
                'mesh_roots', default_value='[]',
                description='JSON [[prefix, directory]] for mesh URIs no package '
                'resolves (the robot entry\'s mesh_roots).',
            ),
            DeclareLaunchArgument(
                'isaac_python',
                default_value=EnvironmentVariable('FM_ISAAC_PYTHON', default_value=''),
                description="Isaac Sim's python.sh; empty logs the runner command instead.",
            ),
            DeclareLaunchArgument(
                'isaac_gui', default_value='false',
                description="'true' opens the Isaac Sim window.",
            ),
            LogInfo(
                msg='Isaac backend: ensure Isaac Sim publishes /isaac_joint_states '
                'and subscribes /isaac_joint_commands (sensor_msgs/JointState).'
            ),
            # Publishes /robot_description, which the controller_manager below
            # subscribes to on Jazzy. The caller must not run its own (see the
            # package README).
            robot_state_publisher_node(robot_description),
            # Standalone controller_manager; the TopicBasedSystem inside bridges to
            # the externally-running Isaac Sim over the topics above.
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                # Jazzy reads the description from the topic and ignores this
                # parameter; fm-app's Humble fm_bringup still includes this
                # backend and needs it until that launch moves to Jazzy.
                # use_sim_time: the Isaac stage, a process beside this launch,
                # publishes /clock from simulation time; a manager on wall time
                # stamps /joint_states off it. Until the stage plays, this
                # manager's clock stands still.
                parameters=[
                    {'robot_description': robot_description, 'use_sim_time': True},
                    controllers_file,
                ],
                output='screen',
            ),
            OpaqueFunction(function=_stage_setup),
        ]
    )
