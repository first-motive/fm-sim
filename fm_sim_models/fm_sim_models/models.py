"""MJCF model registry: maps each robot to the MuJoCo model the sim loads.

Single source of truth for the MJCF the ``mujoco`` sim backend feeds to
MujocoSystemInterface. The fm_bringup robot registry looks a path up here and
injects it into the description as the ``mujoco_model`` xacro arg whenever
``sim_backend == "mujoco"``, so the path lives in one place instead of being
hardcoded in each robot's ``*.sim.urdf.xacro``.

Paths are the in-container locations under ``/ws/external`` (gitignored on the
host, mounted at ``/ws`` in the dev container). Keys match ``fm_bringup``'s
``RobotSpec.key``.
"""

# robot key -> MJCF path the mujoco backend loads.
_MJCF = {
    # Vendored OpenArm v2 bimanual model; joint names match the description exactly.
    "openarm": "/ws/external/openarm_mujoco/v2/openarm_bimanual.xml",
    # Vendored official SO101 model; joint names match the system exactly.
    "so101": "/ws/external/so_arm/Simulation/SO101/so101_new_calib.xml",
    # Bipedal g1_29dof: right-arm joints match, but its legs differ from the wheeled
    # G1-D, so mujoco is wired-not-yet-validated (pending a wheeled-G1 MJCF).
    "g1_d": "/ws/external/unitree_mujoco/unitree_robots/g1/g1_29dof.xml",
}


def mjcf_path(robot_key):
    """Return the MJCF path for ``robot_key`` or raise a clear error."""
    try:
        return _MJCF[robot_key]
    except KeyError:
        raise RuntimeError(
            f"No MJCF registered for robot '{robot_key}'. "
            f"Registered: {', '.join(sorted(_MJCF))}."
        )
