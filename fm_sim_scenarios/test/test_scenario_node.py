"""The scenario node on a fake backend: topics, reset, outcome, and a slow reset.

Runs the shipped ScenarioNode in-process on a multi-threaded executor, as main()
does, with a stand-in for the engine registered in BACKENDS.
"""

import json
import threading
import time

from conftest import CAN_ON_TABLE
from fm_sim_msgs.msg import ObjectStates, Outcome
from fm_sim_msgs.srv import Reset
from fm_sim_scenarios import scenario as sc, scenario_node
from geometry_msgs.msg import Pose
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
import rclpy.logging
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

OUTSIDE = (0.38, -0.15, 0.2745)
INSIDE = (0.40, -0.30, 0.2745)


class FakeObjects:
    """Publishes the can and ball at 50 Hz; ``move`` blocks until released.

    ``move`` then raises ``error`` when it is set, else answers ``ok``.
    """

    last = None

    def __init__(self, node, on_poses):
        self.positions = {'can': OUTSIDE, 'ball': (0.6, 0.2, 0.227)}
        self.release = threading.Event()
        self.ok = True
        self.error = None
        self.moves = []
        self._node = node
        self._on_poses = on_poses
        node.create_timer(0.02, self._publish)
        FakeObjects.last = self

    def _publish(self):
        poses = {}
        for name, (x, y, z) in self.positions.items():
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.0
            poses[name] = pose
        self._on_poses(self._node.get_clock().now().to_msg(), poses)

    def move(self, poses):
        self.moves.append(poses)
        self.release.wait(5.0)
        if self.error is not None:
            raise self.error
        if self.ok:
            self.positions.update({name: xyz for name, (xyz, _) in poses.items()})
        return self.ok, 'fake engine said no' if not self.ok else 'moved'


@pytest.fixture
def ros(monkeypatch):
    monkeypatch.setitem(scenario_node.BACKENDS, 'fake', FakeObjects)
    rclpy.init()
    executor = MultiThreadedExecutor()
    spinner = threading.Thread(target=executor.spin, daemon=True)
    nodes = []

    def start(scenario=CAN_ON_TABLE, **parameters):
        parameters = {'scenario': json.dumps(scenario), 'backend': 'fake', **parameters}
        overrides = [Parameter(name, value=value) for name, value in parameters.items()]
        node = scenario_node.ScenarioNode(parameter_overrides=overrides)
        probe = rclpy.create_node('scenario_probe')
        nodes.extend([node, probe])
        executor.add_node(node)
        executor.add_node(probe)
        spinner.start()
        return node, probe

    yield start
    executor.shutdown()
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def _collect(probe, msg_type, topic, qos=10):
    received = []
    probe.create_subscription(msg_type, topic, received.append, qos)
    return received


def _latched():
    return QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def _until(predicate, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def _reset(probe, seed):
    client = probe.create_client(Reset, '/sim/reset')
    assert client.wait_for_service(timeout_sec=5.0)
    return client.call_async(Reset.Request(seed=seed))


def test_objects_and_markers_cover_every_object(ros):
    _, probe = ros(seed=4)
    objects = _collect(probe, ObjectStates, '/sim/objects')
    markers = _collect(probe, MarkerArray, '/sim/markers')
    assert _until(lambda: objects and markers)
    states = objects[-1]
    assert states.header.frame_id == 'world'
    assert [o.name for o in states.objects] == ['table', 'paper', 'can', 'ball']
    table = states.objects[0].pose.position
    assert (table.x, table.y, table.z) == (0.5, 0.0, 0.187)
    can = states.objects[2].pose.position
    assert (can.x, can.y, can.z) == OUTSIDE
    namespaces = [(m.ns, m.id) for m in markers[-1].markers]
    assert namespaces == [
        ('objects', 0), ('objects', 1), ('objects', 2), ('objects', 3), ('regions', 0),
    ]
    cylinder = markers[-1].markers[2]
    assert (cylinder.scale.x, cylinder.scale.z) == (pytest.approx(0.066), pytest.approx(0.135))


def test_a_slow_reset_does_not_stall_the_objects(ros):
    _, probe = ros()
    objects = _collect(probe, ObjectStates, '/sim/objects')
    assert _until(lambda: objects)
    future = _reset(probe, 5)
    assert _until(lambda: FakeObjects.last.moves)
    before = len(objects)
    time.sleep(1.0)   # 50 at the fake's rate; a stalled node publishes none
    assert len(objects) - before >= 10, 'objects stopped while the reset waited'
    assert not future.done()
    FakeObjects.last.release.set()
    assert _until(future.done)
    assert future.result().ok
    assert future.result().message == 'respawned 2 object(s) from seed 5'
    assert FakeObjects.last.moves == [sc.spawn_poses(CAN_ON_TABLE, 5)]
    moved = sc.spawn_poses(CAN_ON_TABLE, 5)['can'][0]
    assert _until(lambda: objects[-1].objects[2].pose.position.x == moved[0])


def test_outcome_is_latched_once_per_episode(ros):
    _, probe = ros(seed=3, timeout_s=0.3)
    first = _collect(probe, Outcome, '/sim/outcome', _latched())
    assert _until(lambda: first)
    time.sleep(0.5)
    assert len(first) == 1
    (outcome,) = first
    assert (outcome.scenario, outcome.seed, outcome.success) == ('can_on_table', 3, False)
    assert outcome.reason == 'timeout after 0.3 s: can not held in target'
    assert outcome.elapsed_s == pytest.approx(0.3, abs=0.15)
    # A late subscriber still gets it.
    late = _collect(probe, Outcome, '/sim/outcome', _latched())
    assert _until(lambda: late)
    # A reset opens a new episode, judged under its seed.
    FakeObjects.last.release.set()
    future = _reset(probe, 9)
    assert _until(future.done) and future.result().ok
    assert _until(lambda: len(first) == 2)
    assert first[1].seed == 9


def test_success_when_the_can_holds_in_the_target(ros):
    scenario = json.loads(json.dumps(CAN_ON_TABLE))
    scenario['success']['hold_s'] = 0.2
    _, probe = ros(scenario)
    outcomes = _collect(probe, Outcome, '/sim/outcome', _latched())
    FakeObjects.last.positions['can'] = INSIDE
    assert _until(lambda: outcomes)
    assert outcomes[0].success
    assert outcomes[0].reason == 'can in target for 0.2 s'


def test_a_failed_reset_keeps_the_seed(ros):
    _, probe = ros(seed=2, timeout_s=0.5)
    FakeObjects.last.ok = False
    FakeObjects.last.release.set()
    future = _reset(probe, 8)
    assert _until(future.done)
    assert not future.result().ok
    assert future.result().message == 'reset to seed 8 failed: fake engine said no'
    outcomes = _collect(probe, Outcome, '/sim/outcome', _latched())
    assert _until(lambda: outcomes)
    assert outcomes[0].seed == 2


def test_a_raising_reset_answers_and_keeps_the_seed(ros):
    _, probe = ros(seed=2, timeout_s=0.5)
    FakeObjects.last.error = RuntimeError('engine crashed')
    FakeObjects.last.release.set()
    future = _reset(probe, 8)
    assert _until(future.done), 'a raising engine left the caller without an answer'
    assert not future.result().ok
    assert future.result().message == 'reset to seed 8 failed: RuntimeError: engine crashed'
    outcomes = _collect(probe, Outcome, '/sim/outcome', _latched())
    assert _until(lambda: outcomes)
    assert outcomes[0].seed == 2


def test_a_non_finite_pose_is_left_out(ros):
    _, probe = ros()
    FakeObjects.last.positions['can'] = (float('nan'), -0.15, 0.2745)
    objects = _collect(probe, ObjectStates, '/sim/objects')
    assert _until(lambda: len(objects) >= 3)
    assert [o.name for o in objects[-1].objects] == ['table', 'paper', 'ball']


def test_unknown_backend_is_refused(ros):
    with pytest.raises(sc.ScenarioError, match="backend 'bullet' cannot read"):
        ros(backend='bullet')


@pytest.mark.parametrize('parameters, match', [
    ({'timeout_s': -1.0}, 'timeout_s must be finite'),
    ({'timeout_s': float('inf')}, 'timeout_s must be finite'),
    ({'frame_id': '../world'}, 'not a frame name'),
])
def test_bad_parameters_are_refused(ros, parameters, match):
    with pytest.raises(sc.ScenarioError, match=match):
        ros(**parameters)


def test_no_verdict_while_a_reset_moves_objects(ros):
    _, probe = ros(timeout_s=0.3)
    outcomes = _collect(probe, Outcome, '/sim/outcome', _latched())
    future = _reset(probe, 4)
    assert _until(lambda: FakeObjects.last.moves)
    time.sleep(0.6)                       # past the old episode's timeout
    assert outcomes == []
    FakeObjects.last.release.set()
    assert _until(future.done) and future.result().ok
    assert _until(lambda: outcomes)
    assert outcomes[0].seed == 4


class _StubNode:
    """Records what MujocoObjects creates, for the adapter tests."""

    def __init__(self, ready=True):
        self.subscriptions = {}
        self.clients = {}
        self.ready = ready

    def get_logger(self):
        return rclpy.logging.get_logger('stub')

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = (msg_type, callback)

    def create_client(self, srv_type, name, callback_group=None):
        stub = self

        class Client:
            def service_is_ready(self):
                return stub.ready

        self.clients[name] = (srv_type, callback_group)
        return Client()


def test_mujoco_adapter_reads_world_frame_poses():
    node = _StubNode()
    received = []
    scenario_node.MujocoObjects(node, lambda stamp, poses: received.append(poses))
    msg_type, callback = node.subscriptions['/mujoco/free_joint_states']
    assert msg_type is FreeJointStateArray
    msg = FreeJointStateArray()
    joint = FreeJointState(name='can')
    joint.pose.pose.position.x = 0.4
    msg.free_joints = [joint]
    callback(msg)                          # 0.1.2 stamps its default world frame ''
    assert received[0]['can'].position.x == 0.4
    msg.header.frame_id = 'world'          # the plugin's name for the same frame
    callback(msg)
    assert len(received) == 2
    msg.header.frame_id = 'openarm_body_link0'
    callback(msg)
    assert len(received) == 2
    srv_type, group = node.clients['/mujoco_ros2_control_node/set_free_joint_state']
    assert srv_type is SetFreeJointState and group is not None


def test_mujoco_adapter_refuses_a_move_before_the_engine_is_up():
    adapter = scenario_node.MujocoObjects(_StubNode(ready=False), lambda *_: None)
    ok, message = adapter.move({'can': ((0, 0, 0), (0, 0, 0, 1))})
    assert not ok and 'not available yet' in message
