"""Launch sim_loop with its params loaded from a config file.

Defaults come from this package's config/sim.yaml. Override the whole file with
``params_file:=/path/to/my.yaml`` rather than editing the packaged default.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "fm_sim_core"
DEFAULT_PARAMS = os.path.join(get_package_share_directory(PKG), "config", "sim.yaml")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=DEFAULT_PARAMS,
                description="YAML params for sim_loop (default: packaged config/sim.yaml).",
            ),
            Node(
                package=PKG,
                executable="sim_loop",
                name="sim_loop",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
