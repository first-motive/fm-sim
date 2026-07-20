"""MJCF model registry: maps each robot to the MuJoCo model the sim loads.

Single source of truth for the MJCF the ``mujoco`` sim backend feeds to
MujocoSystemInterface. The fm_bringup robot registry looks a path up here and
injects it into the description as the ``mujoco_model`` xacro arg whenever
``sim_backend == "mujoco"``, so the path lives in one place instead of being
hardcoded in each robot's ``*.sim.urdf.xacro``.

Vendored models live under ``<workspace>/external`` (gitignored, imported by vcs).
The container mounts that workspace at ``/ws``; natively it stays at the checkout
root. So paths are stored relative to ``external/`` and resolved against the
workspace root at call time — ``/ws/external/...`` in the container,
``<checkout>/external/...`` on the Mac — instead of hardcoding the container mount.
Axol is the exception: Almond Bot ships no upstream MJCF, so its model is authored
and committed into this package's share (see ``models/axol/``), resolved from the
share dir. Keys match ``fm_bringup``'s ``RobotSpec.key``.
"""

import os


def _workspace_root():
    """Absolute path of the colcon workspace root (the dir that holds ``external/``).

    ``FM_WS`` is the workspace the container mounts at /ws and the run scripts export
    on the native path, so honour it first. Otherwise walk up from this file until a
    directory containing ``external/`` is found. This file may be imported from the
    source checkout (``<root>/src/fm_sim/.../models.py``) OR a colcon symlink-install
    build tree (``<root>/build/fm_sim_models/.../models.py``) — different depths — so
    SEARCH upward rather than count a fixed number of levels (a fixed count matched
    the source layout but overshot the build layout to outside the workspace, landing
    on ``/ws``). Fall back to ``/ws`` for an installed-only container tree.
    """
    env = os.environ.get("FM_WS")
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isdir(os.path.join(here, "external")):
            return here
        parent = os.path.dirname(here)
        if parent == here:  # reached the filesystem root
            return "/ws"
        here = parent


def _vendored(*parts):
    """Absolute path to a vendored MJCF under ``<workspace>/external``."""
    return os.path.join(_workspace_root(), "external", *parts)


def _axol_mjcf():
    """Resolve Axol's committed MJCF (unlike the vendored models, it ships in-repo).

    Called lazily (only when Axol is requested), not at import, so this module imports
    with no ROS dependency. In a built workspace the model lives in the ament share;
    in a ROS-free context (the native macOS sim-core smoke runs the sources straight
    off ``PYTHONPATH``, with no ament index) it lives beside this package's source. Try
    the share first, fall back to the source tree.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        base = get_package_share_directory("fm_sim_models")
    except Exception:
        # No ament index (or package not installed): resolve relative to this file,
        # where models/ sits one level up from the inner package dir.
        base = os.path.join(os.path.dirname(__file__), "..")
    return os.path.join(base, "models", "axol", "axol.xml")


# robot key -> zero-arg callable returning the MJCF path the mujoco backend loads.
# Callables (not strings) so the workspace root is resolved at call time — the same
# registry works in the container (/ws) and natively (the checkout root).
_MJCF = {
    # Vendored OpenArm v2 bimanual model; joint names match the description exactly.
    "openarm": lambda: _vendored("openarm_mujoco", "v2", "openarm_bimanual.xml"),
    # Vendored official SO101 model; joint names match the system exactly.
    "so101": lambda: _vendored("so_arm", "Simulation", "SO101", "so101_new_calib.xml"),
    # Bipedal g1_29dof: right-arm joints match, but its legs differ from the wheeled
    # G1-D, so mujoco is wired-not-yet-validated (pending a wheeled-G1 MJCF).
    "g1_d": lambda: _vendored("unitree_mujoco", "unitree_robots", "g1", "g1_29dof.xml"),
    # Authored + committed into this package's share; joint names match the
    # description + ros2_control exactly. Resolved lazily (see _axol_mjcf).
    "axol": _axol_mjcf,
}


def mjcf_path(robot_key):
    """Return the MJCF path for ``robot_key`` or raise a clear error."""
    try:
        entry = _MJCF[robot_key]
    except KeyError:
        raise RuntimeError(
            f"No MJCF registered for robot '{robot_key}'. "
            f"Registered: {', '.join(sorted(_MJCF))}."
        )
    return entry()
