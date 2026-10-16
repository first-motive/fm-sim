"""The DLS step against a numpy toy arm: no ROS, no KDL."""

from fm_sim_ee_follower.dls import (
    dls_step,
    pose_error,
    rotation_error,
    StepLimits,
)
import numpy as np
import pytest

UNIT = {'x': np.array([1.0, 0, 0]), 'y': np.array([0, 1.0, 0]), 'z': np.array([0, 0, 1.0])}


def _axis_rotation(axis, angle):
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


class ToyArm:
    """Serial revolute arm: each joint is an offset from the previous one, then an axis."""

    def __init__(self, joints, tool):
        self.joints = [(np.array(offset, dtype=float), UNIT[axis]) for offset, axis in joints]
        self.tool = np.array(tool, dtype=float)

    def _frames(self, q):
        position, rotation = np.zeros(3), np.eye(3)
        frames = []
        for (offset, axis), angle in zip(self.joints, q):
            position = position + rotation @ offset
            frames.append((position, rotation @ axis))
            rotation = rotation @ _axis_rotation(axis, angle)
        return frames, (position + rotation @ self.tool, rotation)

    def fk(self, q):
        return self._frames(q)[1]

    def jacobian(self, q):
        frames, (tip, _) = self._frames(q)
        return np.array([np.concatenate([np.cross(z, tip - p), z]) for p, z in frames]).T


ARM6 = ToyArm(
    [([0, 0, 0.1], 'z'), ([0, 0, 0.2], 'y'), ([0, 0, 0.3], 'y'),
     ([0, 0, 0.25], 'x'), ([0.05, 0, 0], 'y'), ([0.05, 0, 0], 'x')],
    tool=[0.1, 0, 0],
)
PLANAR2 = ToyArm([([0, 0, 0], 'z'), ([0.5, 0, 0], 'z')], tool=[0.5, 0, 0])
OPEN = StepLimits(lower=np.full(6, -np.inf), upper=np.full(6, np.inf), max_step=np.full(6, 0.05))


def _run(arm, q, target, limits, steps, damping=0.05, gain=0.5):
    for _ in range(steps):
        q = dls_step(q, target, arm.fk, arm.jacobian, damping, gain, limits)
    return q


def test_converges_on_reachable_target():
    q_goal = np.array([0.4, -0.3, 0.8, 0.2, -0.5, 0.3])
    target = ARM6.fk(q_goal)
    q = _run(ARM6, q_goal + np.array([0.3, -0.3, 0.3, -0.3, 0.3, -0.3]), target, OPEN, 400)
    error = pose_error(ARM6.fk(q), target)
    assert np.linalg.norm(error[:3]) < 1e-4
    assert np.linalg.norm(error[3:]) < 1e-3


def test_step_is_scaled_to_the_per_step_bound_keeping_direction():
    q = np.zeros(6)
    target = (np.array([0.3, 0.3, 0.4]), _axis_rotation(UNIT['x'], 1.0))
    free = StepLimits(OPEN.lower, OPEN.upper, np.full(6, 1e9))
    unclamped = dls_step(q, target, ARM6.fk, ARM6.jacobian, 0.05, 1.0, free) - q
    step = dls_step(q, target, ARM6.fk, ARM6.jacobian, 0.05, 1.0, OPEN) - q
    assert np.max(np.abs(unclamped)) > 0.05
    assert np.max(np.abs(step)) == pytest.approx(0.05)
    np.testing.assert_allclose(step, unclamped * (0.05 / np.max(np.abs(unclamped))))


def test_joint_limits_hold_when_target_needs_more():
    upper = np.full(6, np.inf)
    upper[0] = 0.2
    limits = StepLimits(OPEN.lower, upper, OPEN.max_step)
    target = ARM6.fk(np.array([1.0, 0.2, 0.3, 0.0, 0.0, 0.0]))
    q = np.zeros(6)
    for _ in range(200):
        q = dls_step(q, target, ARM6.fk, ARM6.jacobian, 0.05, 0.5, limits)
        assert q[0] <= 0.2
    assert q[0] == pytest.approx(0.2)


def test_joint_outside_limits_walks_back_at_step_bound():
    upper = np.full(6, np.inf)
    upper[0] = 0.0
    limits = StepLimits(OPEN.lower, upper, OPEN.max_step)
    q = np.array([0.5, 0, 0, 0, 0, 0])
    q_next = dls_step(q, ARM6.fk(q), ARM6.fk, ARM6.jacobian, 0.05, 0.5, limits)
    assert q_next[0] == pytest.approx(0.45)


def test_damping_bounds_the_step_at_a_singularity():
    # Stretched straight along x with a target beyond reach: J J^T is singular.
    q = np.zeros(2)
    target = (np.array([1.5, 0.0, 0.0]), np.eye(3))
    damping = 0.05
    free = StepLimits(np.full(2, -np.inf), np.full(2, np.inf), np.full(2, 1e9))
    step = dls_step(q, target, PLANAR2.fk, PLANAR2.jacobian, damping, 1.0, free) - q
    error = np.linalg.norm(pose_error(PLANAR2.fk(q), target))
    assert np.all(np.isfinite(step))
    # The DLS gain J^T (J J^T + l^2 I)^-1 never exceeds 1 / (2 l).
    assert np.linalg.norm(step) <= error / (2 * damping) + 1e-9


def _nan_fk(_q):
    return np.full(3, np.nan), np.eye(3)


def _nan_jacobian(_q):
    return np.full((6, 6), np.nan)


@pytest.mark.parametrize('q, target, fk, jacobian', [
    (np.zeros(6), ARM6.fk(np.zeros(6)), _nan_fk, ARM6.jacobian),
    (np.zeros(6), ARM6.fk(np.zeros(6)), ARM6.fk, _nan_jacobian),
    (np.zeros(6), (np.array([np.nan, 0, 0]), np.eye(3)), ARM6.fk, ARM6.jacobian),
    (np.array([0, 0, np.inf, 0, 0, 0]), ARM6.fk(np.zeros(6)), ARM6.fk, ARM6.jacobian),
])
def test_non_finite_input_gives_no_step(q, target, fk, jacobian):
    assert dls_step(q, target, fk, jacobian, 0.05, 0.5, OPEN) is None


def test_rotation_error_is_axis_times_angle():
    current = _axis_rotation(UNIT['x'], 0.2)
    target = _axis_rotation(UNIT['z'], 0.5) @ current
    np.testing.assert_allclose(rotation_error(current, target), [0, 0, 0.5], atol=1e-12)


def test_rotation_error_near_pi_keeps_its_magnitude():
    angle = np.pi - 1e-9
    error = rotation_error(np.eye(3), _axis_rotation(UNIT['y'], angle))
    assert np.linalg.norm(error) == pytest.approx(angle)


@pytest.mark.parametrize('axis', ['x', 'y', 'z'])
@pytest.mark.parametrize('angle', [3.0, -3.0])
def test_rotation_error_takes_the_short_way_round(axis, angle):
    error = rotation_error(np.eye(3), _axis_rotation(UNIT[axis], angle))
    np.testing.assert_allclose(error, UNIT[axis] * angle, atol=1e-9)


@pytest.mark.parametrize('axis', ['x', 'y', 'z'])
@pytest.mark.parametrize('angle', [4.0, -4.0, np.pi + 0.5, -np.pi - 0.5])
def test_rotation_past_pi_is_reached_the_other_way(axis, angle):
    error = rotation_error(np.eye(3), _axis_rotation(UNIT[axis], angle))
    short = angle - np.sign(angle) * 2 * np.pi
    np.testing.assert_allclose(error, UNIT[axis] * short, atol=1e-9)
