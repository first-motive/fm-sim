"""The shipped ee_follower node over real topics: it moves on fresh, valid input only.

These are wiring checks; the gate's decision itself is unit-tested in test_gate.py. Each
gate test has the positive case as its control: the same inputs with the gate's condition
fixed do produce commands, so a gate test passes only because the gate held.
"""

import math
from pathlib import Path
import shutil
import time

from builtin_interfaces.msg import Time as TimeMsg
from fm_sim_ee_follower import profile as profile_module
from fm_sim_ee_follower.follower_node import (
    EeFollower,
    MAX_DESCRIPTION_BYTES,
    rotation_from_quaternion,
)
from geometry_msgs.msg import PoseStamped
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

FIXTURES = Path(__file__).resolve().parent / 'fixtures'
URDF = (FIXTURES / 'arm3.urdf').read_text()
JOINTS = ['arm_joint1', 'arm_joint2', 'arm_joint3']
COMMANDS = '/arm3_forward_position_controller/commands'
MAX_AGE_S = 0.2
WINDOW_S = 0.6


def _stamp(seconds):
    whole = int(seconds)
    return TimeMsg(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


class Rig:
    """The follower plus a driver node that feeds it and records its commands.

    With ``sim_time`` the follower runs on /clock, which the driver publishes from ``t``.
    """

    def __init__(self, max_age_s=MAX_AGE_S, sim_time=False, speed_cap=None):
        overrides = [
            Parameter('robot_profile', value='arm3'),
            Parameter('rate_hz', value=50.0),
            Parameter('max_joint_state_age', value=max_age_s),
            Parameter('max_command_age', value=max_age_s),
            Parameter('use_sim_time', value=sim_time),
        ]
        if speed_cap is not None:
            overrides.append(Parameter('joint_speed_cap', Parameter.Type.DOUBLE_ARRAY, speed_cap))
        self.follower = EeFollower(parameter_overrides=overrides)
        self.driver = rclpy.create_node('ee_follower_driver')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._description = self.driver.create_publisher(String, '/robot_description', latched)
        self._joints = self.driver.create_publisher(JointState, '/joint_states', 10)
        self._command = self.driver.create_publisher(PoseStamped, '/commanded_ee_right', 10)
        self._clock = self.driver.create_publisher(Clock, '/clock', 10)
        self.t = 100.0
        self.received = []
        self.driver.create_subscription(
            Float64MultiArray, COMMANDS, lambda m: self.received.append(list(m.data)), 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.follower)
        self.executor.add_node(self.driver)

    def describe(self, urdf=URDF):
        self._description.publish(String(data=urdf))

    def joint_states(self, position=(0.1, 0.2, 0.3)):
        self.publish_joint_state(JointState(name=JOINTS, position=list(position)))

    def publish_joint_state(self, message):
        self._joints.publish(message)

    def command(self, frame='base_link', position=(0.35, 0.1, 0.55), quat=(0.0, 0.0, 0.0, 1.0),
                stamp=None):
        """Publish a command stamped now on the driver's clock, or at ``stamp`` seconds."""
        message = PoseStamped()
        message.header.frame_id = frame
        message.header.stamp = (self.driver.get_clock().now().to_msg() if stamp is None
                                else _stamp(stamp))
        p, o = message.pose.position, message.pose.orientation
        p.x, p.y, p.z = position
        o.x, o.y, o.z, o.w = quat
        self._command.publish(message)

    def tick_clock(self, step=0.005):
        self.t += step
        self._clock.publish(Clock(clock=_stamp(self.t)))

    def spin(self, seconds, *feeds):
        """Spin both nodes, calling every feed each 20 ms."""
        deadline = time.monotonic() + seconds
        next_feed = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_feed:
                for feed in feeds:
                    feed()
                next_feed = time.monotonic() + 0.02
            self.executor.spin_once(timeout_sec=0.005)

    def close(self):
        self.executor.shutdown()
        self.follower.destroy_node()
        self.driver.destroy_node()


@pytest.fixture
def profiles(tmp_path, monkeypatch):
    """Ship the arm3 fixture profile, and only it, for the node under test."""
    shutil.copy(FIXTURES / 'arm3.yaml', tmp_path / 'arm3.yaml')
    monkeypatch.setattr(profile_module, 'profiles_dir', lambda: tmp_path)
    return tmp_path


@pytest.fixture
def rig(request, profiles):
    rclpy.init()
    rig = Rig(getattr(request, 'param', MAX_AGE_S))
    try:
        yield rig
    finally:
        rig.close()
        rclpy.shutdown()


def test_fresh_valid_input_moves_the_arm_within_limits(rig):
    rig.describe()
    rig.spin(WINDOW_S, rig.joint_states, rig.command)
    assert rig.received, 'no command for fresh, valid input'
    for command in rig.received:
        assert len(command) == 3
        assert all(math.isfinite(v) for v in command)
        assert -2.0 <= command[0] <= 2.0 and -0.5 <= command[1] <= 1.5
        assert max(abs(c - s) for c, s in zip(command, (0.1, 0.2, 0.3))) <= 0.02 + 1e-9


def test_no_motion_without_robot_description(rig):
    rig.spin(WINDOW_S, rig.joint_states, rig.command)
    assert rig.received == []


def test_no_motion_without_joint_states(rig):
    rig.describe()
    rig.spin(WINDOW_S, rig.command)
    assert rig.received == []


def test_no_motion_on_stale_joint_states(rig):
    rig.describe()
    rig.joint_states()
    rig.spin(0.05)
    rig.spin(2 * MAX_AGE_S)
    rig.received.clear()
    rig.spin(WINDOW_S, rig.command)
    assert rig.received == []


def test_no_motion_without_command(rig):
    rig.describe()
    rig.spin(WINDOW_S, rig.joint_states)
    assert rig.received == []


def test_no_motion_on_stale_command(rig):
    rig.describe()
    rig.spin(0.1, rig.joint_states, rig.command)
    rig.spin(2 * MAX_AGE_S, rig.joint_states)
    rig.received.clear()
    rig.spin(WINDOW_S, rig.joint_states)
    assert rig.received == []


def test_command_in_another_frame_is_dropped(rig):
    rig.describe()
    rig.spin(WINDOW_S, rig.joint_states, lambda: rig.command(frame='world'))
    assert rig.received == []


NAN_JOINT_STATES = (0.1, math.nan, 0.3)
NON_FINITE_POSES = [
    {'position': (math.nan, 0.1, 0.55)},
    {'position': (0.35, math.inf, 0.55)},
    {'quat': (math.nan, 0.0, 0.0, 1.0)},
    {'quat': (0.0, 0.0, 0.0, 0.0)},
]


@pytest.mark.parametrize('pose', NON_FINITE_POSES)
def test_non_finite_command_alone_does_not_move(rig, pose):
    rig.describe()
    rig.spin(WINDOW_S, rig.joint_states, lambda: rig.command(**pose))
    assert rig.received == []


def test_non_finite_joint_states_alone_do_not_move(rig):
    rig.describe()
    rig.spin(WINDOW_S, lambda: rig.joint_states(NAN_JOINT_STATES), rig.command)
    assert rig.received == []


# Bad input after good input stops the arm: the bad message replaces the good one, so the
# arm does not finish its move toward an older target. The long max age rules staleness
# out as the reason; commands already in flight when the bad input starts drain first.
BAD_INPUTS = {
    'wrong_frame': lambda rig: (rig.joint_states(), rig.command(frame='world')),
    **{f'pose_{i}': (lambda rig, pose=pose: (rig.joint_states(), rig.command(**pose)))
       for i, pose in enumerate(NON_FINITE_POSES)},
    'joint_states': lambda rig: (rig.joint_states(NAN_JOINT_STATES), rig.command()),
}


@pytest.mark.parametrize('rig', [2.0], indirect=True)
@pytest.mark.parametrize('bad', BAD_INPUTS.values(), ids=BAD_INPUTS.keys())
def test_bad_input_after_good_input_stops_the_arm(rig, bad):
    rig.describe()
    rig.spin(0.2, rig.joint_states, rig.command)
    assert rig.received, 'no command for fresh, valid input'
    rig.spin(0.2, lambda: bad(rig))
    rig.received.clear()
    rig.spin(0.3, lambda: bad(rig))
    assert rig.received == []


def test_joint_states_without_this_arm_are_ignored(rig):
    rig.describe()
    other = JointState(name=['other_joint'], position=[0.0])
    short = JointState(name=JOINTS, position=[0.1, 0.2])
    rig.spin(WINDOW_S, lambda: rig.publish_joint_state(other),
             lambda: rig.publish_joint_state(short), rig.command)
    assert rig.received == []


@pytest.mark.parametrize('urdf', [
    '<robot name="broken"><link name="base_link"/></robot>',
    'not xml at all <',
    URDF.replace('<limit lower="-2.0" upper="2.0" effort="10" velocity="3.0"/>', ''),
    URDF.replace('lower="-2.0" upper="2.0" ', ''),
    URDF.replace('velocity="2.0"', 'velocity="0"'),
    URDF + '<!--' + 'x' * MAX_DESCRIPTION_BYTES + '-->',
], ids=['no_chain', 'not_xml', 'no_limit_element', 'no_range', 'no_velocity_limit', 'oversized'])
def test_no_motion_on_unusable_robot_description(rig, urdf):
    rig.describe(urdf)
    rig.spin(WINDOW_S, rig.joint_states, rig.command)
    assert rig.received == []


def test_robot_description_is_locked_to_the_first_one(rig):
    rig.describe()
    rig.spin(0.2, rig.joint_states, rig.command)
    rig.describe('<robot name="broken"><link name="base_link"/></robot>')
    rig.spin(0.2, rig.joint_states, rig.command)
    rig.received.clear()
    rig.spin(0.3, rig.joint_states, rig.command)
    assert rig.received, 'a later, different description replaced the first one'


def test_no_motion_when_chain_joints_differ_from_the_profile(profiles):
    path = profiles / 'arm3.yaml'
    swapped = path.read_text().replace('[arm_joint1, arm_joint2,', '[arm_joint2, arm_joint1,')
    path.write_text(swapped)
    rclpy.init()
    rig = Rig()
    try:
        rig.describe()
        rig.spin(WINDOW_S, rig.joint_states, rig.command)
        assert rig.received == []
    finally:
        rig.close()
        rclpy.shutdown()


@pytest.mark.parametrize('offset', [-1.0, 5.0], ids=['old', 'future'])
def test_command_stamp_out_of_range_does_not_move(rig, offset):
    rig.describe()
    stamp = time.time() + offset
    rig.spin(WINDOW_S, rig.joint_states, lambda: rig.command(stamp=stamp))
    assert rig.received == []


def test_unstamped_command_falls_back_to_arrival_time(rig):
    rig.describe()
    rig.spin(WINDOW_S, rig.joint_states, lambda: rig.command(stamp=0.0))
    assert rig.received


def test_clock_jump_back_drops_the_stored_command(profiles):
    rclpy.init()
    rig = Rig(max_age_s=0.5, sim_time=True)
    try:
        rig.describe()
        rig.spin(0.5, rig.tick_clock, rig.joint_states, lambda: rig.command(stamp=rig.t))
        assert rig.received, 'no command for fresh, valid input on sim time'
        last_command = rig.t
        rig.t -= 0.1
        # The old command is younger than max_command_age again once the clock passes it,
        # so only the drop on the jump keeps the arm still.
        rig.spin(0.6, rig.tick_clock, rig.joint_states)
        assert rig.t > last_command + 0.02
        rig.received.clear()
        rig.spin(0.2, rig.tick_clock, rig.joint_states)
        assert rig.t - last_command < 0.5
        assert rig.received == []
    finally:
        rig.close()
        rclpy.shutdown()


def test_quaternion_is_normalized():
    np.testing.assert_allclose(rotation_from_quaternion(0, 0, 0, 2.0), np.eye(3))


@pytest.mark.parametrize('quat', [(0, 0, 0, 0), (np.nan, 0, 0, 1), (0, np.inf, 0, 1)])
def test_quaternion_that_is_not_a_rotation_is_rejected(quat):
    with pytest.raises(ValueError):
        rotation_from_quaternion(*quat)


def test_speed_cap_parameter_bounds_a_joint_the_urdf_leaves_unbounded(profiles):
    rclpy.init()
    rig = Rig(speed_cap=[3.0, 3.0, 0.5])
    try:
        rig.describe(URDF.replace('velocity="2.0"', 'velocity="0"'))
        rig.spin(WINDOW_S, rig.joint_states, rig.command)
        assert rig.received, 'a speed cap did not stand in for the missing velocity limit'
        # 0.5 rad/s at 50 Hz is 0.01 rad per step on the capped joint.
        assert all(abs(c[2] - 0.3) <= 0.01 + 1e-9 for c in rig.received)
    finally:
        rig.close()
        rclpy.shutdown()


@pytest.mark.parametrize('speed_cap', [[1.0, 1.0], [1.0, 1.0, 0.0]], ids=['short', 'zero'])
def test_invalid_speed_cap_parameter_stops_startup(profiles, speed_cap):
    rclpy.init()
    try:
        with pytest.raises(ValueError, match='joint_speed_cap'):
            Rig(speed_cap=speed_cap)
    finally:
        rclpy.shutdown()
