"""The motion gate on its own: ages and flags to a blocked reason."""

from fm_sim_ee_follower.gate import blocked_reason
import pytest

NOW = 100.0
FRESH = {
    'has_kinematics': True,
    'joint_state_time': NOW - 0.1,
    'command_time': NOW - 0.1,
    'max_joint_state_age': 0.25,
    'max_command_age': 0.5,
}


def test_fresh_inputs_may_move():
    assert blocked_reason(NOW, **FRESH) is None


@pytest.mark.parametrize('change, reason', [
    ({'has_kinematics': False}, 'no usable /robot_description'),
    ({'joint_state_time': None}, 'no joint state yet'),
    ({'joint_state_time': NOW - 0.26}, 'joint state stale'),
    ({'joint_state_time': NOW + 0.01}, 'joint state stale'),
    ({'command_time': None}, 'no command yet'),
    ({'command_time': NOW - 0.51}, 'command stale'),
    ({'command_time': NOW + 0.01}, 'command stale'),
])
def test_missing_stale_or_future_input_blocks(change, reason):
    assert blocked_reason(NOW, **{**FRESH, **change}) == reason


def test_ages_at_the_bounds_still_move():
    at_bounds = {**FRESH, 'joint_state_time': NOW - 0.25, 'command_time': NOW - 0.5}
    assert blocked_reason(NOW, **at_bounds) is None
