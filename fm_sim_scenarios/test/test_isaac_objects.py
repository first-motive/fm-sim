"""The scenario node on its Isaac backend, against a fake Isaac Sim stage.

The fake stands in for the stage runner and its bridge graph: it publishes the
free objects' poses as TF on the bridge's topic and applies the teleports the
node publishes. Runs the shipped ScenarioNode with ``backend: isaac``.
"""

import json
import threading
import time

from conftest import CAN_ON_TABLE, EXECUTOR_THREADS
from fm_sim_msgs.msg import ObjectStates
from fm_sim_msgs.srv import Reset
from fm_sim_scenarios import isaac_objects, isaac_stage, scenario as sc, scenario_node
from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

START = {'can': (0.38, -0.15, 0.2755), 'ball': (0.6, 0.2, 0.228)}


class FakeStage:
    """Publishes poses at 50 Hz from ``frame``; applies commands when ``applies``.

    ``stamp_offset_s`` shifts every pose stamp; ``applied_offset_m`` lands each
    teleport that far above its target.
    """

    def __init__(
        self, node, frame='World', listening=True, stamp_offset_s=0.0, applied_offset_m=0.0,
    ):
        self.positions = dict(START)
        self.frame = frame
        self.stamp_offset_ns = int(stamp_offset_s * 1e9)
        self.applied_offset_m = applied_offset_m
        self.applies = True
        self.commands = []
        self._node = node
        self._pub = node.create_publisher(TFMessage, isaac_stage.OBJECT_POSES_TOPIC, 10)
        if listening:
            node.create_subscription(
                TFMessage, isaac_stage.OBJECT_COMMANDS_TOPIC, self._on_command, 10
            )
        node.create_timer(0.02, self._publish)

    def _on_command(self, msg):
        self.commands.append(msg)
        if self.applies:
            for t in msg.transforms:
                v = t.transform.translation
                self.positions[t.child_frame_id] = (v.x, v.y, v.z + self.applied_offset_m)

    def _publish(self):
        msg = TFMessage()
        stamp = Time(nanoseconds=self._node.get_clock().now().nanoseconds + self.stamp_offset_ns)
        for name, (x, y, z) in self.positions.items():
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.frame
            t.child_frame_id = name
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            msg.transforms.append(t)
        self._pub.publish(msg)


@pytest.fixture
def isaac(monkeypatch):
    monkeypatch.setattr(isaac_objects, 'MOVE_TIMEOUT_S', 1.0)
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=EXECUTOR_THREADS)
    nodes = []

    def start(**stage):
        fake = rclpy.create_node('fake_isaac_stage')
        stage_fake = FakeStage(fake, **stage)
        overrides = [
            Parameter('scenario', value=json.dumps(CAN_ON_TABLE)),
            Parameter('backend', value='isaac'),
        ]
        node = scenario_node.ScenarioNode(parameter_overrides=overrides)
        probe = rclpy.create_node('isaac_probe')
        for n in (fake, node, probe):
            nodes.append(n)
            executor.add_node(n)
        threading.Thread(target=executor.spin, daemon=True).start()
        return stage_fake, probe

    yield start
    executor.shutdown()
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def _until(predicate, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _objects(probe):
    received = []
    probe.create_subscription(ObjectStates, '/sim/objects', received.append, 10)
    return received


def _reset(probe, seed):
    client = probe.create_client(Reset, '/sim/reset')
    assert client.wait_for_service(timeout_sec=5.0)
    future = client.call_async(Reset.Request(seed=seed))
    assert _until(future.done, 10.0)
    return future.result()


def _position(states, name):
    (state,) = [s for s in states.objects if s.name == name]
    p = state.pose.position
    return (p.x, p.y, p.z)


def test_isaac_poses_reach_sim_objects(isaac):
    _, probe = isaac()
    objects = _objects(probe)
    assert _until(lambda: objects)
    assert objects[-1].header.frame_id == 'world'
    assert [s.name for s in objects[-1].objects] == ['table', 'paper', 'can', 'ball']
    assert _position(objects[-1], 'can') == pytest.approx(START['can'])


def test_reset_teleports_through_the_command_topic(isaac):
    stage, probe = isaac()
    objects = _objects(probe)
    response = _reset(probe, 6)
    assert response.ok, response.message
    spawn = sc.spawn_poses(CAN_ON_TABLE, 6)
    (command,) = stage.commands
    assert {t.child_frame_id for t in command.transforms} == {'can', 'ball'}
    assert {t.header.frame_id for t in command.transforms} == {'World'}
    assert _until(lambda: objects and _position(objects[-1], 'can') == pytest.approx(
        spawn['can'][0]
    ))


def test_reset_fails_when_the_stage_never_moves_the_objects(isaac):
    stage, probe = isaac()
    stage.applies = False
    response = _reset(probe, 6)
    assert not response.ok
    assert 'not at their targets' in response.message


def test_reset_fails_without_isaac_listening(isaac):
    _, probe = isaac(listening=False)
    response = _reset(probe, 6)
    assert not response.ok
    assert 'is Isaac Sim running?' in response.message


def test_poses_outside_the_world_frame_are_skipped(isaac):
    _, probe = isaac(frame='openarm_body_link0')
    objects = _objects(probe)
    time.sleep(0.5)
    assert objects == []


def test_poses_stamped_before_the_command_do_not_acknowledge_it(isaac):
    # The stage moves the objects, but every pose it sends is stamped 10 s early,
    # so none of them is newer than the command.
    stage, probe = isaac(stamp_offset_s=-10.0)
    response = _reset(probe, 6)
    assert stage.commands, 'the command never reached the stage'
    assert not response.ok
    assert 'not at their targets' in response.message


def test_an_object_beyond_the_tolerance_does_not_acknowledge_the_move(isaac):
    stage, probe = isaac(applied_offset_m=2 * isaac_objects.POSITION_TOLERANCE_M)
    response = _reset(probe, 6)
    assert stage.commands
    assert not response.ok


class _StubNode:
    """A node whose clock stands still at ``now_ns``; records the command publisher."""

    def __init__(self, now_ns):
        self.now_ns = now_ns
        self.published = []
        self.callbacks = {}

    def get_logger(self):
        return rclpy.logging.get_logger('stub')

    def get_clock(self):
        stub = self

        class Clock:
            def now(self):
                return Time(nanoseconds=stub.now_ns)

        return Clock()

    def create_subscription(self, msg_type, topic, callback, qos):
        self.callbacks[topic] = callback

    def create_publisher(self, msg_type, topic, qos):
        stub = self

        class Publisher:
            def get_subscription_count(self):
                return 1

            def publish(self, msg):
                stub.published.append(msg)

        return Publisher()


def _poses_at(stamp_ns, xyz):
    t = TransformStamped()
    t.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    t.header.frame_id = 'World'
    t.child_frame_id = 'can'
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.w = 1.0
    return TFMessage(transforms=[t])


@pytest.mark.parametrize('stamp_offset_ns, acknowledged', [(0, False), (1, True)])
def test_only_a_pose_stamped_strictly_after_the_command_acknowledges(
    monkeypatch, stamp_offset_ns, acknowledged,
):
    monkeypatch.setattr(isaac_objects, 'MOVE_TIMEOUT_S', 0.5)
    node = _StubNode(now_ns=5_000_000_000)
    adapter = isaac_objects.IsaacObjects(node, lambda *_: None)
    target = (0.4, -0.3, 0.2755)
    result = []
    mover = threading.Thread(
        target=lambda: result.append(adapter.move({'can': (target, (0.0, 0.0, 0.0, 1.0))}))
    )
    mover.start()
    assert _until(lambda: node.published)
    node.callbacks[isaac_stage.OBJECT_POSES_TOPIC](
        _poses_at(node.now_ns + stamp_offset_ns, target)
    )
    mover.join(2.0)
    (ok, _), = result
    assert ok is acknowledged
