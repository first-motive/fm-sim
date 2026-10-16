"""Damped-least-squares IK step: one bounded joint update toward a target pose.

ROS-free and solver-free. The caller passes forward kinematics and a geometric Jacobian as
functions, so this runs against KDL in the node and against a numpy toy arm in tests. The
quaternion and rotation-vector helpers are written here rather than taken from PyKDL for
the same reason: this module, and its tests, need numpy only.

The step solves

    dq = J^T (J J^T + damping^2 I)^-1 (gain * e)

where ``e`` stacks the position error and the rotation-vector error, both in the base
frame. Damping keeps ``dq`` finite near a singularity, where the plain pseudo-inverse
blows up. The step is then scaled down uniformly so no joint moves more than its per-step
bound, which keeps the direction of motion, and the result is held inside the joint limits.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

#: (position (3,), rotation matrix (3, 3)) of the tool frame in the base frame.
Pose = tuple[np.ndarray, np.ndarray]


@dataclass(frozen=True)
class StepLimits:
    """Bounds on one IK step.

    Attributes:
        lower: Lower joint limits, ``-inf`` for a continuous joint.
        upper: Upper joint limits, ``inf`` for a continuous joint.
        max_step: Largest change any one joint may make in one step (rad or m), positive.
    """

    lower: np.ndarray
    upper: np.ndarray
    max_step: np.ndarray


def rotation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Return the rotation vector that turns ``current`` into ``target``, in the base frame.

    Goes through the quaternion of ``target @ current.T`` rather than ``arccos`` of the
    trace, which loses precision near 0 and pi.
    """
    w, x, y, z = _quaternion_wxyz(target @ current.T)
    if w < 0.0:  # q and -q are the same rotation; take the short way round.
        w, x, y, z = -w, -x, -y, -z
    vector = np.array([x, y, z])
    sin_half = float(np.linalg.norm(vector))
    if sin_half < 1e-12:
        return np.zeros(3)
    return vector * (2.0 * np.arctan2(sin_half, w) / sin_half)


def pose_error(current: Pose, target: Pose) -> np.ndarray:
    """Stack position error and rotation-vector error into one 6-vector."""
    return np.concatenate([target[0] - current[0], rotation_error(current[1], target[1])])


def dls_step(
    q: np.ndarray,
    target: Pose,
    fk: Callable[[np.ndarray], Pose],
    jacobian: Callable[[np.ndarray], np.ndarray],
    damping: float,
    gain: float,
    limits: StepLimits,
) -> np.ndarray | None:
    """Return the next joint positions toward ``target``, or None on a non-finite input.

    With finite inputs the damped matrix is positive definite, so the step is finite too.

    Args:
        q: Current joint positions.
        target: Target tool pose in the base frame.
        fk: Joint positions to tool pose.
        jacobian: Joint positions to the 6 x n geometric Jacobian (linear rows first), in
            the base frame with the tool frame as reference point.
        damping: Damping factor lambda, positive.
        gain: Fraction of the pose error to close in this step, in (0, 1].
        limits: Joint limits and per-step bound.
    """
    error = gain * pose_error(fk(q), target)
    jac = jacobian(q)
    if not all(np.all(np.isfinite(a)) for a in (q, error, jac)):
        return None
    dq = jac.T @ np.linalg.solve(jac @ jac.T + damping**2 * np.eye(jac.shape[0]), error)
    largest = np.max(np.abs(dq) / limits.max_step)
    if largest > 1.0:
        dq = dq / largest
    within_limits = np.clip(q + dq, limits.lower, limits.upper)
    # A joint that starts outside its limits is walked back in at the step bound, not
    # snapped there.
    step = np.clip(within_limits - q, -limits.max_step, limits.max_step)
    return q + step


def _quaternion_wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Return the unit quaternion of a rotation matrix (Shepperd's method)."""
    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        return 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        return (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    if m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        return (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
    return (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
