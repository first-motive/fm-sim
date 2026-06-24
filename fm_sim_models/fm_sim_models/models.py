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

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory

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

_SO101_TASK_ENVS = {
    "table_reach": "table_reach.xml",
    "pick_place": "pick_place.xml",
    "bin_sort": "bin_sort.xml",
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


def task_env_template_path(task_env: str, workspace_root: str = "/ws") -> str:
    filename = _SO101_TASK_ENVS[task_env]
    candidates = [
        os.path.join(workspace_root, "fm_sim_models", "assets", "mujoco", "so101", filename),
        os.path.join(workspace_root, "assets", "mujoco", "so101", filename),
        os.path.join(
            get_package_share_directory("fm_sim_models"),
            "assets",
            "mujoco",
            "so101",
            filename,
        ),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[-1]


def task_env_runtime_path(task_env: str, workspace_root: str = "/ws") -> str:
    return os.path.join(
        workspace_root,
        "external",
        "so_arm",
        "Simulation",
        "SO101",
        f"fm_task_env_{task_env}.xml",
    )


def materialize_task_env_model(task_env: str, workspace_root: str = "/ws") -> str:
    template_path = task_env_template_path(task_env, workspace_root=workspace_root)
    if not os.path.exists(template_path):
        raise RuntimeError(
            f"SO101 task-env template not found: {template_path}. "
            "Expected a tracked template under fm_sim_models/assets/mujoco/so101/."
        )

    runtime_path = task_env_runtime_path(task_env, workspace_root=workspace_root)
    os.makedirs(os.path.dirname(runtime_path), exist_ok=True)

    with open(template_path, "r", encoding="utf-8") as handle:
        xml = handle.read()
    with open(runtime_path, "w", encoding="utf-8") as handle:
        handle.write(xml)
    return runtime_path
