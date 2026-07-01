"""MJCF model registry: maps each robot to the MuJoCo model the sim loads.

Single source of truth for the MJCF the ``mujoco`` sim backend feeds to
MujocoSystemInterface. The fm_bringup robot registry looks a path up here and
injects it into the description as the ``mujoco_model`` xacro arg whenever
``sim_backend == "mujoco"``, so the path lives in one place instead of being
hardcoded in each robot's ``*.sim.urdf.xacro``.

Most paths are the in-container locations under ``/ws/external`` (gitignored on
the host, mounted at ``/ws`` in the dev container). Axol is the exception: Almond
Bot ships no upstream MJCF, so its model is authored and committed into this
package's share (see ``models/axol/``), resolved here from the share dir. Keys
match ``fm_bringup``'s ``RobotSpec.key``.
"""

import os


def _axol_mjcf():
    """Resolve Axol's committed MJCF from this package's share.

    Called lazily (only when Axol is requested), not at import, so neither the ament
    lookup nor a failed package resolution can break ``mjcf_path`` for the vendored
    robots. The import lives here too, so this module imports with no ROS dependency.
    Unlike the vendored models, Axol's MJCF is authored and committed into this
    package (Almond Bot ships no upstream MJCF).
    """
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory("fm_sim_models"), "models", "axol", "axol.xml"
    )


# robot key -> MJCF path the mujoco backend loads. Values are either an absolute
# path string (vendored models under the /ws mount) or a zero-arg callable that
# resolves the path on demand (Axol's committed model — resolved from the share).
_MJCF = {
    # Vendored OpenArm v2 bimanual model; joint names match the description exactly.
    "openarm": "/ws/external/openarm_mujoco/v2/openarm_bimanual.xml",
    # Vendored official SO101 model; joint names match the system exactly.
    "so101": "/ws/external/so_arm/Simulation/SO101/so101_new_calib.xml",
    # Bipedal g1_29dof: right-arm joints match, but its legs differ from the wheeled
    # G1-D, so mujoco is wired-not-yet-validated (pending a wheeled-G1 MJCF).
    "g1_d": "/ws/external/unitree_mujoco/unitree_robots/g1/g1_29dof.xml",
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
    return entry() if callable(entry) else entry
