"""Launch a sim backend: the in-process MuJoCo dev loop, or an external engine.

``backend`` selects what comes up:

- ``mujoco`` (default) — the in-process ``sim_loop`` node, the Mac dev driver.
  Steps a MuJoCo model and publishes JointState; params come from ``params_file``
  (default: this package's ``config/sim.yaml``).
- ``gazebo`` / ``isaac`` — delegate to ``fm_sim_backends/<backend>.launch.py``,
  the external-engine hosts. These need a ``robot_description`` and a
  ``controllers_file``, which fm-sim does not own: in the assembled stack
  ``fm_bringup`` supplies them. Run standalone with neither and the launch fails
  with a clear, actionable error rather than a cryptic include failure.

Examples::

    ros2 launch fm_sim_core sim.launch.py backend:=mujoco
    ros2 launch fm_sim_core sim.launch.py backend:=mujoco params_file:=/path/my.yaml
    ros2 launch fm_sim_core sim.launch.py backend:=gazebo \\
        robot_description:="$(xacro ...)" controllers_file:=/path/controllers.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PKG = "fm_sim_core"
DEFAULT_PARAMS = os.path.join(get_package_share_directory(PKG), "config", "sim.yaml")

# Backends that delegate to fm_sim_backends; each needs a caller-supplied
# robot_description + controllers_file. mujoco is handled in-process (sim_loop).
_INCLUDE_BACKENDS = ("gazebo", "isaac")
_VALID_BACKENDS = ("mujoco", *_INCLUDE_BACKENDS)


def _dispatch(context, *args, **kwargs):
    """Resolve ``backend`` at launch time and return the actions for it."""
    backend = LaunchConfiguration("backend").perform(context)

    if backend == "mujoco":
        # In-process dev loop. params_file carries model_path + rate_hz.
        return [
            Node(
                package=PKG,
                executable="sim_loop",
                name="sim_loop",
                output="screen",
                parameters=[LaunchConfiguration("params_file").perform(context)],
            )
        ]

    if backend in _INCLUDE_BACKENDS:
        robot_description = LaunchConfiguration("robot_description").perform(context)
        controllers_file = LaunchConfiguration("controllers_file").perform(context)
        # fm-sim owns no robot description; the assembled stack (fm_bringup)
        # supplies it. Standalone, fail with a clear pointer instead of a cryptic
        # include error deep inside the backend launch.
        if not robot_description or not controllers_file:
            raise RuntimeError(
                f"backend:={backend} needs robot_description and controllers_file, "
                "which fm-sim does not own. Launch it from the assembled stack "
                "(fm_bringup supplies both), or pass them explicitly: "
                f"ros2 launch fm_sim_core sim.launch.py backend:={backend} "
                "robot_description:=... controllers_file:=..."
            )
        backend_launch = PathJoinSubstitution(
            [FindPackageShare("fm_sim_backends"), "launch", f"{backend}.launch.py"]
        )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(backend_launch),
                launch_arguments={
                    "robot_description": robot_description,
                    "controllers_file": controllers_file,
                }.items(),
            )
        ]

    raise RuntimeError(
        f"unknown backend {backend!r} — valid: {', '.join(_VALID_BACKENDS)}"
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="mujoco",
                description=f"Sim backend: {', '.join(_VALID_BACKENDS)} "
                "(mujoco runs the in-process sim_loop).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=DEFAULT_PARAMS,
                description="YAML params for sim_loop (mujoco backend; "
                "default: packaged config/sim.yaml).",
            ),
            DeclareLaunchArgument(
                "robot_description",
                default_value="",
                description="Robot description XML for the gazebo/isaac backends "
                "(caller-supplied; fm_bringup provides it in the assembled stack).",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value="",
                description="controllers.yaml for the gazebo/isaac backends "
                "(caller-supplied; fm_bringup provides it in the assembled stack).",
            ),
            OpaqueFunction(function=_dispatch),
        ]
    )
