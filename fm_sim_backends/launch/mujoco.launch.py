"""MuJoCo sim backend for the OpenArm — the Mac daily driver (CPU, no GPU).

mujoco_ros2_control's ros2_control_node hosts the controller_manager *inside* the
MuJoCo simulation: it loads the robot_description (whose <ros2_control> System plugin
is mujoco_ros2_control/MujocoSystemInterface), steps the MJCF physics, and exposes
the hardware interfaces the controllers drive. The launch that includes this one
spawns the controllers against the controller_manager it brings up.

On Jazzy the controller_manager reads the description from the
``/robot_description`` topic, not a node parameter, so this launch runs the
robot_state_publisher that publishes it. The description comes from fm-robot's
``fm_control/urdf/<robot>.sim.urdf.xacro`` with ``sim_backend:=mujoco`` and the
``mujoco_model`` arg set to ``fm_sim_models.mjcf_path(<key>)``, a model whose
joint names match the description's exactly (for OpenArm,
``openarm_loader.sim.urdf.xacro`` and ``openarm_anvil``).

``config/mujoco_plugins.yaml`` loads mujoco_ros2_control's free-joint state
publisher, so every loose object's pose is on ``/mujoco/free_joint_states``, and
its CameraPlugin, which renders the world's cameras. ``MUJOCO_GL`` picks how:
``egl`` (the default; the GPU where the host has one), ``osmesa`` (Mesa's
software rasteriser through EGL, for a host with no GPU such as the Mac under
OrbStack) or ``glfw`` (a hidden window under xvfb-run; the only choice on macOS).

``params_file`` (``fm_sim_backends.params_file``) is appended last to the
engine node, then a guard file keeping it on sim time with this description.
"""

import os
import platform
import shutil
from shutil import which
import tempfile

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

from fm_sim_backends import process_group
from fm_sim_backends.description import (
    robot_description_parameter,
    robot_state_publisher_node,
)
from fm_sim_backends.params_file import declare_params_file, with_params_file, write_guard
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


# The engine process's two nodes, by fully qualified name: the controller_manager,
# and the hardware interface's own node that reads mujoco_plugins
# (config/mujoco_plugins.yaml keys it). The guard file keys both.
CONTROLLER_MANAGER = '/controller_manager'
HARDWARE_NODE = '/mujoco_ros2_control_node'
# MUJOCO_GL values, as MuJoCo's own Python bindings name them.
RENDERERS = ('egl', 'osmesa', 'glfw')
# glvnd's vendor file for Mesa: pins EGL to Mesa when a GPU vendor is installed too.
MESA_EGL_VENDOR = '/usr/share/glvnd/egl_vendor.d/50_mesa.json'


def _renderer():
    """Return the renderer ``MUJOCO_GL`` asks for: egl by default, glfw on macOS.

    mujoco_ros2_control's CameraPlugin (0.1.2) renders through a hidden GLFW
    window when the engine process has a display, and through a surfaceless EGL
    context when it has none; fm_sim_bringup's test_camera_launch.py checks the
    EGL path live. macOS has no EGL, so there the only choice is glfw.
    """
    default = 'glfw' if platform.system() == 'Darwin' else 'egl'
    # Set but empty is refused like any other unknown value, not taken as unset.
    renderer = os.environ['MUJOCO_GL'] if 'MUJOCO_GL' in os.environ else default
    if renderer not in RENDERERS:
        raise RuntimeError(
            f'MUJOCO_GL={renderer!r} is not a renderer; choose one of {", ".join(RENDERERS)}'
        )
    if platform.system() == 'Darwin' and renderer != 'glfw':
        raise RuntimeError(
            f'MUJOCO_GL={renderer!r} needs EGL, which macOS does not have; use glfw'
        )
    return renderer


def _render_env(renderer):
    """Return the environment that makes the CameraPlugin render with ``renderer``.

    egl and osmesa take the display away, so the plugin's GLFW init fails and it
    falls back to EGL: glvnd then picks the GPU's EGL vendor where one is
    installed. osmesa also pins EGL to Mesa and forces its software rasteriser
    (llvmpipe, the renderer OSMesa wraps): the CameraPlugin has no OSMesa path of
    its own, so this is the GPU-less fallback.
    """
    if renderer == 'glfw':
        return {}
    env = {'DISPLAY': '', 'WAYLAND_DISPLAY': ''}
    if renderer == 'osmesa':
        env.update({
            '__EGL_VENDOR_LIBRARY_FILENAMES': MESA_EGL_VENDOR,
            'LIBGL_ALWAYS_SOFTWARE': '1',
        })
    return env


def _display_prefix(renderer):
    """Return the launch prefix that gives a glfw renderer a display.

    Only glfw on Linux needs one: the engine itself runs headless (the
    description's ``headless`` parameter), and egl and osmesa render without a
    display. On Linux the hidden window needs a display that a headless
    container/host has no session for, so it runs under a virtual one
    (``xvfb-run``). macOS has no X11: GLFW talks to Cocoa directly, so the prefix
    is empty there.

    xvfb-run runs the node in the foreground of a ``/bin/sh`` that defers launch's
    SIGINT and, killed by the SIGTERM that follows, leaves the node and Xvfb running:
    the controller_manager outlived shutdown, and the next launch's spawners found
    it. ``process_group`` runs xvfb-run as a job (``set -m``, so the node does not
    start with SIGINT ignored) and passes each shutdown signal to xvfb-run's children
    except Xvfb. The node stops first; xvfb-run then stops Xvfb, as on a normal exit,
    and returns the node's exit status. Signalling Xvfb in the same instant would cut
    the node's X connection in the middle of its shutdown.
    """
    if renderer != 'glfw' or platform.system() == 'Darwin':
        return ''
    return process_group.prefix('xvfb-run', '-a', spare='Xvfb')


def _require_backend(renderer):
    """Fail early with a clear pointer when the MuJoCo backend can't run here.

    The backend hosts the controller_manager inside ``mujoco_ros2_control``. The
    fm-sim image installs it from the Jazzy apt mirror; a native host needs the
    same package. Absent, the launch would die deep with a cryptic "package not
    found"; say so up front.

    xvfb-run is required only where it is actually used — see ``_display_prefix``.
    """
    missing = []
    try:
        get_package_share_directory('mujoco_ros2_control')
    except PackageNotFoundError:
        missing.append(
            'the mujoco_ros2_control package (apt install ros-jazzy-mujoco-ros2-control)'
        )
    if _display_prefix(renderer) and which('xvfb-run') is None:
        missing.append('xvfb-run (the virtual display)')
    if missing:
        raise RuntimeError(
            'The MuJoCo sim backend needs ' + ' and '.join(missing) + '.'
        )


def _guards(robot_description, plugins_file):
    """Return the engine process's launch-owned params, keyed by its two nodes.

    The ``/**`` dict reaches both: the controller_manager and the hardware
    interface's own node, which also reads the plugins file. Every plugin key
    but its rate is the launch's: a plugin's ``type`` names code to load.
    """
    with open(plugins_file) as plugins:
        loaded = yaml.safe_load(plugins)[HARDWARE_NODE.lstrip('/')]['ros__parameters']
    owned = {'use_sim_time': True, 'robot_description': robot_description}
    return {
        CONTROLLER_MANAGER: owned,
        HARDWARE_NODE: {**owned, 'mujoco_plugins': {
            name: {key: value for key, value in plugin.items() if not key.endswith('rate')}
            for name, plugin in loaded['mujoco_plugins'].items()
        }},
    }


def _engine(context, renderer, plugins_file):
    """Return the engine node, with the run's params file and its guard when one is given.

    The guard re-asserts what this launch owns on both nodes the process
    starts (``_guards``), so a params file cannot take the engine off sim time,
    swap its description or load another plugin. The robot_state_publisher gets
    no params file: it owns no tunables, and its robot_description is the robot.
    """
    robot_description = LaunchConfiguration('robot_description').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    actions, guard = [], None
    if params_file:
        guard_dir = tempfile.mkdtemp(prefix='fm_sim_guard_')
        guard = write_guard(guard_dir, _guards(robot_description, plugins_file))
        actions.append(RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: shutil.rmtree(guard_dir, ignore_errors=True)
        )))
    # controller_manager hosted inside MuJoCo. Steps physics + serves the
    # hardware interfaces; the caller spawns the controllers.
    # The cameras' GL context comes from MUJOCO_GL (_render_env, and
    # _display_prefix for glfw). MJCF path comes from the mujoco_model param.
    actions.append(Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        prefix=_display_prefix(renderer),
        additional_env=_render_env(renderer),
        # Jazzy reads the description from the topic and ignores this
        # parameter; fm-app's Humble fm_bringup still includes this
        # backend and needs it until that launch moves to Jazzy.
        # use_sim_time: the engine publishes /clock, and the controllers
        # stamp /joint_states (so the robot's /tf) from this node's clock,
        # which must match the sim-time stamps of /sim/objects and cameras.
        # The clock comes from this same process, so it runs whenever the
        # controllers do. The params file reaches the hardware interface's own
        # node (mujoco_ros2_control_node) too, as the plugins file does.
        parameters=with_params_file(
            [
                {'robot_description': robot_description_parameter(), 'use_sim_time': True},
                LaunchConfiguration('controllers_file'), plugins_file,
            ],
            params_file, guard,
        ),
        output='screen',
    ))
    return actions


def generate_launch_description():
    # Guard before building the description, so a missing backend stops with an
    # actionable message rather than a cryptic one deep in the launch.
    renderer = _renderer()
    _require_backend(renderer)

    plugins_file = os.path.join(
        get_package_share_directory('fm_sim_backends'), 'config', 'mujoco_plugins.yaml'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_description',
                description='Robot description XML with the MujocoSystemInterface '
                '<ros2_control> system (carries the mujoco_model param).',
            ),
            DeclareLaunchArgument(
                'controllers_file',
                description='Path to the controllers.yaml for the active preset.',
            ),
            declare_params_file(),
            # Publishes /robot_description, which the controller_manager below
            # subscribes to on Jazzy. The caller must not run its own (see the
            # package README).
            robot_state_publisher_node(robot_description_parameter()),
            OpaqueFunction(function=_engine, args=[renderer, plugins_file]),
        ]
    )
