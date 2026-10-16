"""The KDL chain built from a URDF: joints, limits, FK and Jacobian."""

from pathlib import Path

from fm_sim_ee_follower import kinematics
from fm_sim_ee_follower.dls import rotation_error
import numpy as np
import pytest


URDF = (Path(__file__).resolve().parent / 'fixtures' / 'arm3.urdf').read_text()


def _rx(a):
    return np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])


def _ry(a):
    return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])


@pytest.fixture
def arm():
    return kinematics.ArmKinematics(URDF, 'base_link', 'tool')


def test_chain_holds_only_the_movable_joints_base_to_tip(arm):
    assert arm.joint_names == ('arm_joint1', 'arm_joint2', 'arm_joint3')


def test_limits_come_from_the_urdf(arm):
    np.testing.assert_array_equal(arm.lower, [-2.0, -0.5, -np.inf])
    np.testing.assert_array_equal(arm.upper, [2.0, 1.5, np.inf])
    np.testing.assert_array_equal(arm.velocity, [3.0, 3.0, 2.0])


def test_fk_at_zero_matches_the_urdf_origins(arm):
    position, rotation = arm.fk(np.zeros(3))
    np.testing.assert_allclose(position, [0.4, 0.0, 0.6], atol=1e-12)
    np.testing.assert_allclose(rotation, _rx(0.1) @ _ry(0.2), atol=1e-12)


def test_jacobian_matches_finite_differences(arm):
    q = np.array([0.3, -0.2, 0.7])
    jac = arm.jacobian(q)
    position, rotation = arm.fk(q)
    eps = 1e-6
    for i in range(3):
        dq = np.zeros(3)
        dq[i] = eps
        moved_position, moved_rotation = arm.fk(q + dq)
        np.testing.assert_allclose(jac[:3, i], (moved_position - position) / eps, atol=1e-5)
        np.testing.assert_allclose(
            jac[3:, i], rotation_error(rotation, moved_rotation) / eps, atol=1e-5)


@pytest.mark.parametrize('base, tip', [
    ('base_link', 'no_such_link'),
    ('no_such_link', 'tool'),
    ('other_link', 'tool'),
    ('link3', 'tool'),
])
def test_frames_without_a_movable_chain_are_rejected(base, tip):
    with pytest.raises(ValueError):
        kinematics.ArmKinematics(URDF, base, tip)


def test_unsupported_joint_type_is_rejected():
    planar = URDF.replace('name="arm_joint3" type="continuous"', 'name="arm_joint3" type="planar"')
    with pytest.raises(ValueError, match='planar'):
        kinematics.ArmKinematics(planar, 'base_link', 'tool')


JOINT1_LIMIT = '<limit lower="-2.0" upper="2.0" effort="10" velocity="3.0"/>'
JOINT3_END = '<limit effort="10" velocity="2.0"/>\n  </joint>'


def test_mimic_joint_is_rejected():
    mimic = URDF.replace(JOINT3_END, '<limit effort="10" velocity="2.0"/>\n'
                         '    <mimic joint="arm_joint2"/>\n  </joint>')
    assert mimic != URDF
    with pytest.raises(ValueError, match='mimic'):
        kinematics.ArmKinematics(mimic, 'base_link', 'tool')


def test_prismatic_joint_moves_along_its_axis():
    prismatic = URDF.replace('name="arm_joint3" type="continuous"',
                             'name="arm_joint3" type="prismatic"').replace(
        JOINT3_END, '<limit lower="0" upper="0.2" effort="1" velocity="0.5"/>\n  </joint>')
    arm = kinematics.ArmKinematics(prismatic, 'base_link', 'tool')
    assert arm.upper[2] == 0.2
    moved = arm.fk(np.array([0.0, 0.0, 0.1]))[0] - arm.fk(np.zeros(3))[0]
    np.testing.assert_allclose(moved, _rx(0.1) @ [0, 0.1, 0], atol=1e-12)


def test_joint_without_origin_sits_at_its_parent():
    bare = URDF.replace('<origin xyz="0 0 0.1" rpy="0 0 0"/>\n', '')
    assert bare != URDF
    position, _ = kinematics.ArmKinematics(bare, 'base_link', 'tool').fk(np.zeros(3))
    np.testing.assert_allclose(position, [0.4, 0.0, 0.5], atol=1e-12)


def test_revolute_joint_without_a_limit_element_is_rejected():
    open_ended = URDF.replace(JOINT1_LIMIT, '')
    assert open_ended != URDF
    with pytest.raises(ValueError, match='arm_joint1'):
        kinematics.ArmKinematics(open_ended, 'base_link', 'tool')


def test_step_bound_is_the_smaller_of_max_step_and_speed_times_period(arm):
    limits = arm.step_limits(max_joint_step=0.05, period=0.01)
    np.testing.assert_allclose(limits.max_step, [0.03, 0.03, 0.02])
    np.testing.assert_array_equal(limits.lower, arm.lower)
    capped = arm.step_limits(max_joint_step=0.05, period=0.01, speed_caps=(1.0, 5.0, 5.0))
    np.testing.assert_allclose(capped.max_step, [0.01, 0.03, 0.02])


@pytest.mark.parametrize('velocity', ['0', '0.0'])
def test_missing_velocity_limit_refuses_the_step_without_a_cap(velocity):
    unset = URDF.replace('effort="10" velocity="2.0"', f'effort="10" velocity="{velocity}"')
    arm = kinematics.ArmKinematics(unset, 'base_link', 'tool')
    assert np.isnan(arm.velocity[2])
    with pytest.raises(ValueError, match='arm_joint3'):
        arm.step_limits(max_joint_step=0.05, period=0.01)
    limits = arm.step_limits(max_joint_step=0.05, period=0.01, speed_caps=(9.0, 9.0, 1.0))
    np.testing.assert_allclose(limits.max_step, [0.03, 0.03, 0.01])


def test_speed_caps_must_match_the_joints(arm):
    with pytest.raises(ValueError, match='speed caps'):
        arm.step_limits(max_joint_step=0.05, period=0.01, speed_caps=(1.0,))


@pytest.mark.parametrize('limit', [
    '<limit effort="10" velocity="3.0"/>',
    '<limit lower="1.0" upper="1.0" effort="10" velocity="3.0"/>',
    '<limit lower="1.0" upper="-1.0" effort="10" velocity="3.0"/>',
], ids=['omitted_reads_as_0_0', 'equal', 'inverted'])
def test_revolute_joint_without_a_range_is_rejected(limit):
    locked = URDF.replace(JOINT1_LIMIT, limit)
    with pytest.raises(ValueError, match='lower must be below upper'):
        kinematics.ArmKinematics(locked, 'base_link', 'tool')


def test_empty_speed_cap_means_none():
    assert kinematics.speed_caps([], 3) is None
    assert kinematics.speed_caps(None, 3) is None
    assert kinematics.speed_caps([1, 2.5, 3], 3) == (1.0, 2.5, 3.0)


@pytest.mark.parametrize('values', [[1.0], [1.0, 2.0, 0.0], [1.0, 2.0, -1.0],
                                    [1.0, 2.0, float('inf')], [1.0, 2.0, float('nan')]])
def test_invalid_speed_cap_is_rejected(values):
    with pytest.raises(ValueError, match='joint_speed_cap'):
        kinematics.speed_caps(values, 3)
