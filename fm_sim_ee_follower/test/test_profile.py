"""Profile loading: the shipped profile, and name validation."""

from pathlib import Path

from fm_sim_ee_follower.profile import load_arm, parse_arm
import pytest

FIXTURES = Path(__file__).resolve().parent / 'fixtures'


def test_shipped_profile_names_the_sim_controller():
    arm = load_arm('fm_openarm_v2', 'right')
    assert arm.base_frame == 'openarm_body_link0'
    assert arm.joints == tuple(f'openarm_right_joint{i}' for i in range(1, 8))
    assert arm.ee_frame == 'openarm_right_grasp_frame'
    assert arm.command_topic == '/openarm_right_forward_position_controller/commands'


def test_shipped_loader_profile_names_the_sim_controller():
    arm = load_arm('loader_openarm_v2', 'right')
    assert arm.base_frame == 'follower_body_link0'
    assert arm.joints == tuple(f'follower_r_joint{i}' for i in range(1, 8))
    assert arm.ee_frame == 'follower_r_hand_tcp'
    assert arm.command_topic == '/follower_r_forward_position_controller/commands'


# A yaml path is a name with separators: shipped names only.
@pytest.mark.parametrize('name', [
    '../fm_openarm_v2', 'a/b', '', 'fm openarm', str(FIXTURES / 'arm3.yaml'),
])
def test_name_that_is_not_a_shipped_profile_name_is_rejected(name):
    with pytest.raises(ValueError):
        load_arm(name, 'right')


def test_unknown_profile_is_not_found():
    with pytest.raises(FileNotFoundError):
        load_arm('no_such_robot', 'right')


def _profile(**arm):
    names = {'joints': ['j1'], 'arm_controller': 'ctrl', 'ee_frame': 'tool', **arm}
    return {'base_frame': 'base', 'arms': {'right': names}}


@pytest.mark.parametrize('arm', [
    {'joints': []},
    {'joints': ['ok', '../bad']},
    {'ee_frame': 'tool frame'},
    {'arm_controller': None},
])
def test_invalid_names_are_rejected(arm):
    with pytest.raises(ValueError):
        parse_arm(_profile(**arm), 'right')


def test_missing_arm_is_rejected():
    with pytest.raises(ValueError, match='arms.left'):
        parse_arm(_profile(), 'left')
