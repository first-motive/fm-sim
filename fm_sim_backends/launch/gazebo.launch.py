"""Gazebo Harmonic sim backend — Linux/GPU, or headless anywhere.

The robot_description carries two gazebo-specific pieces, emitted by fm-robot's
``fm_control/urdf/<robot>.sim.urdf.xacro`` for ``sim_backend:=gz`` (or ``gazebo``):
the gz_ros2_control/GazeboSimSystem in the <ros2_control> block, and a <gazebo>
plugin (gz_ros2_control-system) that hosts the controller_manager inside the sim
and loads the ``gazebo_controllers_file`` xacro arg. The launch that includes this
one spawns the controllers against the controller_manager the plugin brings up.

Under ``run.launch.py`` the ``world`` argument is the scenario's world SDF
(``fm_sim_scenarios.gazebo_world.write``). This launch writes the robot half
beside it (``gazebo_world.write_robot``: the URDF with resolved meshes and the
camera sensors from ``cameras_file``, and the bridge config), starts the gz-sim
server on it in a process group of its own, spawns the robot as ``robot``, and
runs the bridge and the camera relays (compressed images and camera_info).
Without a ``world`` it runs gz-sim's ``empty.sdf``, spawns /robot_description and
bridges /clock only. See ``fm_sim_backends.gazebo``.
"""

from fm_sim_backends import gazebo
from fm_sim_backends.description import (
    robot_description_parameter,
    robot_state_publisher_node,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _run(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    return gazebo.run_actions(
        arg('gz_args'), arg('world'), arg('robot_description'), arg('robot'),
        arg('cameras_file'), gazebo.parse_mesh_roots(arg('mesh_roots')),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'gz_args',
                default_value='-r -v 3',
                description='gz-sim flags (run, verbosity, server-only); the world follows.',
            ),
            DeclareLaunchArgument(
                'world', default_value='',
                description="The scenario's generated world SDF; empty runs empty.sdf.",
            ),
            DeclareLaunchArgument(
                'robot', default_value='robot',
                description='Name of the spawned robot model (the robot entry name).',
            ),
            DeclareLaunchArgument(
                'cameras_file', default_value='',
                description="The robot's cameras file; empty renders no cameras.",
            ),
            DeclareLaunchArgument(
                'mesh_roots', default_value='[]',
                description='JSON [[prefix, directory]] for mesh URIs Gazebo cannot resolve.',
            ),
            DeclareLaunchArgument(
                'robot_description',
                description='Robot description XML with the GazeboSimSystem '
                '<ros2_control> system and the gz_ros2_control plugin.',
            ),
            # Gazebo hosts its controller_manager in the gz_ros2_control plugin
            # embedded in the description, which loads the controllers.yaml itself;
            # controllers_file is accepted for a uniform backend contract.
            DeclareLaunchArgument(
                'controllers_file',
                default_value='',
                description='controllers.yaml for the preset (gazebo loads '
                'controllers via the description plugin; reserved for override).',
            ),
            # The gz_ros2_control plugin reads the description from
            # robot_state_publisher. The caller must not run its own (see the README).
            robot_state_publisher_node(robot_description_parameter()),
            OpaqueFunction(function=_run),
        ]
    )
