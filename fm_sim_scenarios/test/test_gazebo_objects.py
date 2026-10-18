"""The scenario node on its Gazebo backend, against a fake Gazebo bridge.

The fake stands in for gz-sim behind ros_gz_bridge: a PoseStamped per free object
on ``gazebo_world.pose_topic`` and the world's ``set_pose`` service. Runs the
shipped ScenarioNode with ``backend: gazebo``.
"""

import json
import math
import threading
import time

from conftest import CAN_ON_TABLE
from fm_sim_msgs.msg import ObjectStates
from fm_sim_msgs.srv import Reset
from fm_sim_scenarios import gazebo_objects, gazebo_world, scenario as sc, scenario_node
from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

START = {'can': (0.38, -0.15, 0.2755), 'ball': (0.6, 0.2, 0.228)}


class FakeGazebo:
    """Publishes each object's pose at 50 Hz in ``frame``; serves set_pose.

    ``stale`` objects keep publishing a stamp a second old. ``answers`` false
    leaves set_pose unserved; ``succeeds`` false refuses every move and ``refuse``
    the moves of those objects. ``applies`` false acknowledges moves without
    making them. ``slow`` delays the reply (not the move, as Gazebo queues it) to
    the first request for each named object by that many seconds; ``slow_apply``
    (``(reply_s, apply_s)``) answers the first one after ``reply_s`` and applies it
    after ``apply_s``. ``applied`` logs each pose a move set, in order.
    """

    def __init__(self, node, frame=gazebo_world.WORLD_NAME, stale=(), answers=True):
        self.positions = dict(START)
        self.frame = frame
        self.stale = set(stale)
        self.succeeds = True
        self.applies = True
        self.refuse = set()
        self.slow = {}
        self.slow_apply = {}
        self.requests = []
        self.applied = []
        self.late_replies = 0
        self.ahead_s = 0.0
        self._node = node
        self._pubs = {
            name: node.create_publisher(PoseStamped, gazebo_world.pose_topic(name), 10)
            for name in START
        }
        if answers:
            # Reentrant, so a slow reply holds neither the poses nor other requests.
            node.create_service(
                SetEntityPose, gazebo_world.SET_POSE_SERVICE, self._set_pose,
                callback_group=ReentrantCallbackGroup(),
            )
        node.create_timer(0.02, self._publish)

    def _set_pose(self, request, response):
        name = request.entity.name
        self.requests.append(request)
        ok = self.succeeds and name not in self.refuse
        stale = self.slow_apply.pop(name, None)
        if stale:
            # Answered after the node gave up, and applied later still, after the
            # rollback that followed: a queued move landing out of order.
            reply_s, apply_s = stale
            p = request.pose.position
            threading.Timer(apply_s, lambda: self._apply(name, p)).start()
            time.sleep(reply_s)
            response.success = True
            return response
        if ok and self.applies:
            self._apply(name, request.pose.position)
        delay = self.slow.pop(name, 0.0)
        if delay:
            time.sleep(delay)
            self.late_replies += 1
        response.success = ok
        return response

    def _apply(self, name, p):
        self.positions[name] = (p.x, p.y, p.z)
        self.applied.append((name, self.positions[name]))

    def _publish(self):
        now = self._node.get_clock().now()
        for name, (x, y, z) in self.positions.items():
            msg = PoseStamped()
            stamp = now - Duration(seconds=1.0) if name in self.stale else now
            stamp = stamp + Duration(seconds=self.ahead_s)
            msg.header.stamp = stamp.to_msg()
            msg.header.frame_id = self.frame
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
            msg.pose.orientation.w = 1.0
            self._pubs[name].publish(msg)


@pytest.fixture
def gazebo(monkeypatch):
    monkeypatch.setattr(gazebo_objects, 'MOVE_TIMEOUT_S', 2.0)
    rclpy.init()
    executor = MultiThreadedExecutor()
    nodes = []

    def start(scenario=CAN_ON_TABLE, **fake_args):
        fake = rclpy.create_node('fake_gazebo')
        gz = FakeGazebo(fake, **fake_args)
        overrides = [
            Parameter('scenario', value=json.dumps(scenario)),
            Parameter('backend', value='gazebo'),
        ]
        node = scenario_node.ScenarioNode(parameter_overrides=overrides)
        probe = rclpy.create_node('gazebo_probe')
        for n in (fake, node, probe):
            nodes.append(n)
            executor.add_node(n)
        threading.Thread(target=executor.spin, daemon=True).start()
        return gz, probe

    yield start
    executor.shutdown()
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


# The first poses reach the probe once DDS discovery completes, which a loaded CI
# host can stretch well past the 5 s the behaviour checks allow.
DISCOVERY_S = 30.0


def _wait(predicate, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _goes_quiet(received, quiet_s=1.0, timeout_s=15.0):
    """Whether ``received`` stops growing for ``quiet_s`` within ``timeout_s``.

    A fixed settle sleep misses poses still queued on a loaded host; a stream that
    never stops (the dropped pose published after all) still fails.
    """
    deadline = time.monotonic() + timeout_s
    count, since = len(received), time.monotonic()
    while time.monotonic() < deadline:
        time.sleep(0.05)
        if len(received) != count:
            count, since = len(received), time.monotonic()
        elif time.monotonic() - since >= quiet_s:
            return True
    return False


def _objects(probe):
    received = []
    probe.create_subscription(ObjectStates, '/sim/objects', received.append, 10)
    return received


def _positions(states):
    return {
        s.name: (s.pose.position.x, s.pose.position.y, s.pose.position.z)
        for s in states.objects
    }


def _reset(probe, seed):
    client = probe.create_client(Reset, '/sim/reset')
    assert client.wait_for_service(timeout_sec=5.0)
    future = client.call_async(Reset.Request(seed=seed))
    assert _wait(future.done, 30.0)
    return future.result()


def test_objects_come_from_the_bridged_poses(gazebo):
    _, probe = gazebo()
    received = _objects(probe)
    assert _wait(lambda: received and len(received[-1].objects) == 4, DISCOVERY_S)
    positions = _positions(received[-1])
    assert positions['can'] == pytest.approx(START['can'])
    assert positions['ball'] == pytest.approx(START['ball'])
    assert positions['table'] == (0.5, 0.0, 0.187)
    assert received[-1].header.frame_id == 'world'


def test_a_stale_pose_is_left_out(gazebo):
    _, probe = gazebo(stale=['ball'])
    received = _objects(probe)
    assert _wait(lambda: len(received) >= 5)
    assert all('ball' not in _positions(states) for states in received)
    assert all('can' in _positions(states) for states in received)


@pytest.mark.parametrize('bad', ['nan', 'future'])
def test_non_finite_or_future_poses_are_skipped(gazebo, bad):
    gz, probe = gazebo()
    received = _objects(probe)
    assert _wait(lambda: received and len(received[-1].objects) == 4, DISCOVERY_S)
    if bad == 'nan':
        gz.positions['can'] = (float('nan'), 0.0, 0.0)
    else:
        gz.ahead_s = 1.0
    # The can paces /sim/objects: with its poses dropped, nothing more is published.
    assert _goes_quiet(received), f'/sim/objects kept publishing: {received[-1]}'
    if bad == 'nan':
        assert all(math.isfinite(x) for states in received for x in _positions(states)['can'])


def test_poses_in_another_frame_are_skipped(gazebo):
    _, probe = gazebo(frame='other')
    received = _objects(probe)
    time.sleep(0.5)
    assert received == []


def test_reset_moves_each_free_object_through_set_pose(gazebo):
    gz, probe = gazebo()
    received = _objects(probe)
    assert _wait(lambda: received and len(received[-1].objects) == 4, DISCOVERY_S)
    response = _reset(probe, 11)
    assert response.ok, response.message
    spawn = sc.spawn_poses(CAN_ON_TABLE, 11)
    assert [r.entity.name for r in gz.requests] == ['can', 'ball']
    assert all(r.entity.type == Entity.MODEL for r in gz.requests)
    (can_xyz, can_quat) = spawn['can']
    orientation = gz.requests[0].pose.orientation
    assert (orientation.x, orientation.y, orientation.z, orientation.w) == pytest.approx(can_quat)
    received.clear()
    assert _wait(lambda: received and _positions(received[-1])['can'] == pytest.approx(can_xyz))


def test_a_refused_move_fails_the_reset(gazebo):
    gz, probe = gazebo()
    received = _objects(probe)
    assert _wait(lambda: received and len(received[-1].objects) == 4, DISCOVERY_S)
    gz.succeeds = False
    response = _reset(probe, 3)
    assert not response.ok
    assert 'refused or did not answer for can, ball' in response.message


def test_a_second_object_refused_restores_the_first(gazebo):
    gz, probe = gazebo()
    received = _objects(probe)
    assert _wait(lambda: received and len(received[-1].objects) == 4, DISCOVERY_S)
    gz.refuse = {'ball'}
    response = _reset(probe, 3)
    assert not response.ok
    assert 'ball' in response.message and 'restored every object' in response.message
    # The can moved, then went back to where it stood; the refused ball never left.
    moved, restored = [r for r in gz.requests if r.entity.name == 'can']
    assert (moved.pose.position.x, moved.pose.position.y) == pytest.approx(
        sc.spawn_poses(CAN_ON_TABLE, 3)['can'][0][:2]
    )
    assert (restored.pose.position.x, restored.pose.position.y, restored.pose.position.z) == (
        pytest.approx(START['can'])
    )
    assert gz.positions['can'] == pytest.approx(START['can'])


def test_a_late_reply_is_ignored(gazebo, monkeypatch):
    gz, probe = gazebo()
    received = _objects(probe)
    assert _wait(lambda: received and len(received[-1].objects) == 4, DISCOVERY_S)
    gz.slow = {'can': 2.5}
    response = _reset(probe, 3)
    assert not response.ok
    assert 'did not answer for can' in response.message
    assert _wait(lambda: gz.late_replies == 1)
    # The later resets are not about timing: give a loaded host room.
    monkeypatch.setattr(gazebo_objects, 'MOVE_TIMEOUT_S', 5.0)
    # The late success of the first move does not count toward the next one.
    gz.refuse = {'can'}
    response = _reset(probe, 4)
    assert not response.ok
    assert 'refused or did not answer for can' in response.message
    gz.refuse = set()
    response = _reset(probe, 5)
    assert response.ok, response.message
    assert _wait(lambda: _positions(received[-1])['can'] == pytest.approx(
        sc.spawn_poses(CAN_ON_TABLE, 5)['can'][0]
    ))


def test_a_stale_forward_move_after_the_rollback_is_reported(gazebo, monkeypatch):
    # Wide margins: the rollback waits behind the slow reply on a loaded test host.
    monkeypatch.setattr(gazebo_objects, 'MOVE_TIMEOUT_S', 3.0)
    gz, probe = gazebo()
    received = _objects(probe)
    assert _wait(lambda: received and len(received[-1].objects) == 4, DISCOVERY_S)
    gz.slow_apply = {'can': (4.0, 9.0)}
    response = _reset(probe, 3)
    assert not response.ok
    # The rollback landed (the adapter checks that before answering), and the warning
    # names the object whose move may still land.
    assert 'restored every object, but can may still move' in response.message
    target = sc.spawn_poses(CAN_ON_TABLE, 3)['can'][0]
    assert _wait(lambda: gz.positions['can'] == pytest.approx(target), 10.0)
    # The can was back at its start before the stale move landed. Read from the log,
    # not the live pose, which that move can overwrite before the reply reaches a
    # loaded probe.
    landed = [position for name, position in gz.applied if name == 'can']
    assert landed == [pytest.approx(START['can']), pytest.approx(target)]


def test_a_move_that_does_not_land_is_undone(gazebo):
    gz, probe = gazebo()
    received = _objects(probe)
    assert _wait(lambda: received and len(received[-1].objects) == 4, DISCOVERY_S)
    gz.applies = False
    response = _reset(probe, 3)
    assert not response.ok
    assert 'did not land' in response.message


def test_an_unserved_set_pose_fails_the_reset(gazebo):
    _, probe = gazebo(answers=False)
    response = _reset(probe, 3)
    assert not response.ok
    assert 'not available' in response.message


def test_static_only_scenario_publishes_on_a_timer(gazebo):
    scenario = dict(CAN_ON_TABLE)
    scenario['objects'] = [o for o in CAN_ON_TABLE['objects'] if sc.is_static(o)]
    scenario.pop('success')
    _, probe = gazebo(scenario=scenario)
    received = _objects(probe)
    assert _wait(lambda: len(received) >= 3)
    assert set(_positions(received[-1])) == {'table', 'paper'}
