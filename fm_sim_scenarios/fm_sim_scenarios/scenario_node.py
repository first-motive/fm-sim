"""Scenario node: ground-truth objects, markers, reset and the episode outcome.

Serves the scenario contract on the sim host:

- ``/sim/objects`` (fm_sim_msgs/ObjectStates): every scenario object, republished
  on each pose update from the backend; static objects from their yaml pose.
- ``/sim/markers`` (visualization_msgs/MarkerArray): the objects and the regions.
- ``/sim/reset`` (fm_sim_msgs/srv/Reset): draws spawn poses from the seed, moves
  the free objects there in the engine and starts a new episode.
- ``/sim/outcome`` (fm_sim_msgs/Outcome, transient local): one per episode, from
  the scenario's ``success`` predicate. A scenario without one publishes none.

Parameters: ``scenario`` (the catalog entry as JSON), ``seed`` (the spawn the
world was generated with), ``backend`` (``mujoco`` or ``isaac``: how objects
are read and moved), ``timeout_s`` (0 keeps the scenario's), ``frame_id``. Run it with
``use_sim_time``: episode times are sim seconds.

A reset waits on the engine in its own callback group, so pose updates keep
flowing on a multi-threaded executor; a lock guards the episode state only.
"""

import json
import math
import threading
import time

from fm_sim_msgs.msg import ObjectState, ObjectStates, Outcome
from fm_sim_msgs.srv import Reset
from fm_sim_scenarios import scenario as sc
from fm_sim_scenarios.isaac_objects import IsaacObjects
from geometry_msgs.msg import Pose
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

TICK_PERIOD_S = 0.1
MOVE_TIMEOUT_S = 5.0
_MARKER_TYPES = {'box': Marker.CUBE, 'cylinder': Marker.CYLINDER, 'sphere': Marker.SPHERE}


def _pose(xyz, quat_xyzw):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
    (
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    ) = (float(v) for v in quat_xyzw)
    return pose


def _finite(pose):
    """Return True when every position and orientation value of ``pose`` is finite."""
    p, q = pose.position, pose.orientation
    return all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))


def _marker_scale(shape, size):
    """Marker scale (full extents) of a box, cylinder (radius, height) or sphere."""
    if shape == 'box':
        return size
    if shape == 'cylinder':
        return (2 * size[0], 2 * size[0], size[1])
    if shape == 'sphere':
        return (2 * size[0],) * 3
    raise sc.ScenarioError(f'unknown shape {shape!r}')


def _marker(namespace, marker_id, shape, size, color, pose, frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = _MARKER_TYPES[shape]
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in _marker_scale(shape, size))
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (float(v) for v in color)
    return marker


def wait_for(future, timeout_s, poll_s=0.01):
    """Block until an rclpy future completes; return False on timeout.

    Another executor thread must complete it (its client in a callback group
    other than the caller's). Polling, not a done-callback: rclpy may queue a
    callback added after completion as an executor task instead of running it.
    """
    deadline = time.monotonic() + timeout_s
    while not future.done():
        if time.monotonic() >= deadline:
            return False
        time.sleep(poll_s)
    return True


class MujocoObjects:
    """Read and move free objects in mujoco_ros2_control's MuJoCo world.

    Poses come from the FreeJointStatePublisherPlugin that fm_sim_backends'
    mujoco launch loads (``config/mujoco_plugins.yaml``), in the world frame;
    respawns go through the hardware interface's ``set_free_joint_state`` service.
    """

    POSES_TOPIC = '/mujoco/free_joint_states'
    # The plugin's frame_id when left at its default, the MuJoCo world: mujoco_ros2_control
    # 0.1.2 stamps '' (checked on a live run); 'world' is the plugin's own name for it.
    # Any other frame is a body frame and is refused.
    WORLD_FRAMES = ('', 'world')
    SET_SERVICE = '/mujoco_ros2_control_node/set_free_joint_state'

    def __init__(self, node, on_poses):
        self._on_poses = on_poses
        self._logger = node.get_logger()
        node.create_subscription(FreeJointStateArray, self.POSES_TOPIC, self._poses, 10)
        self._set = node.create_client(
            SetFreeJointState, self.SET_SERVICE,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _poses(self, msg):
        if msg.header.frame_id not in self.WORLD_FRAMES:
            self._logger.error(
                f'{self.POSES_TOPIC} is in frame {msg.header.frame_id!r}, not the world; '
                'skipping', throttle_duration_sec=5.0,
            )
            return
        poses = {joint.name: joint.pose.pose for joint in msg.free_joints}
        self._on_poses(msg.header.stamp, poses)

    def move(self, poses):
        """Teleport ``{name: (xyz, quat_xyzw)}`` at rest; return ``(ok, message)``."""
        if not self._set.service_is_ready():
            return False, f'{self.SET_SERVICE} is not available yet'
        request = SetFreeJointState.Request()
        for name, (xyz, quat) in poses.items():
            joint = FreeJointState()
            joint.name = name
            joint.pose.pose = _pose(xyz, quat)
            request.free_joints.append(joint)
        future = self._set.call_async(request)
        if not wait_for(future, MOVE_TIMEOUT_S):
            self._set.remove_pending_request(future)
            return False, f'{self.SET_SERVICE} did not answer in {MOVE_TIMEOUT_S:g} s'
        response = future.result()
        return response.success, response.message


# Backend name -> how this node reads and moves objects there. A backend joins by
# adding its class here (Gazebo with its generator).
BACKENDS = {'mujoco': MujocoObjects, 'isaac': IsaacObjects}


class ScenarioNode(Node):

    def __init__(self, **kwargs):
        super().__init__('sim_scenario', **kwargs)
        self._scenario = json.loads(self.declare_parameter('scenario', '').value or 'null')
        if not isinstance(self._scenario, dict):
            raise sc.ScenarioError("parameter 'scenario' must be the scenario entry as JSON")
        sc.check(self._scenario)
        seed = sc.check_seed(self.declare_parameter('seed', 0).value)
        backend = self.declare_parameter('backend', '').value
        timeout_s = self.declare_parameter('timeout_s', 0.0).value
        if not (math.isfinite(timeout_s) and timeout_s >= 0.0):
            raise sc.ScenarioError(f'timeout_s must be finite and >= 0, got {timeout_s!r}')
        self._frame_id = self.declare_parameter('frame_id', 'world').value
        if not sc.FRAME_NAME.fullmatch(self._frame_id):
            raise sc.ScenarioError(f'frame_id {self._frame_id!r} is not a frame name')
        if backend not in BACKENDS:
            raise sc.ScenarioError(
                f'backend {backend!r} cannot read scenario objects; supported: '
                f'{", ".join(sorted(BACKENDS))}'
            )

        self._static = sc.static_poses(self._scenario)
        # Episode state, shared by the pose, tick and reset callbacks.
        self._lock = threading.Lock()
        self._seed = seed
        self._monitor = (
            sc.EpisodeMonitor(self._scenario, timeout_s) if 'success' in self._scenario else None
        )
        self._episode_opened = False
        # Set while a reset moves objects: the old episode stops judging.
        self._resetting = False

        self._objects_pub = self.create_publisher(ObjectStates, '/sim/objects', 10)
        self._markers_pub = self.create_publisher(MarkerArray, '/sim/markers', 10)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._outcome_pub = self.create_publisher(Outcome, '/sim/outcome', latched)
        self._backend = BACKENDS[backend](self, self._on_poses)
        self.create_service(
            Reset, '/sim/reset', self._on_reset, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.create_timer(TICK_PERIOD_S, self._on_tick)
        judged = 'no success predicate'
        if self._monitor:
            judged = f'timeout {self._monitor.timeout_s:g} s'
        self.get_logger().info(
            f'scenario {self._scenario["name"]!r} seed {seed} on {backend}; {judged}'
        )

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tick(self):
        if self._monitor is None:
            return
        now = self._now_s()
        with self._lock:
            # The first episode opens with the sim clock; before /clock arrives, now is 0.
            if not self._episode_opened and now > 0.0:
                self._monitor.start(now)
                self._episode_opened = True
            verdict = None if self._resetting else self._monitor.tick(now)
            seed = self._seed
        self._publish_verdict(verdict, seed)

    def _on_poses(self, stamp, engine_poses):
        """Publish objects and markers from one engine pose update; feed the predicate."""
        poses = {}
        for obj in self._scenario['objects']:
            name = obj['name']
            if name in self._static:
                poses[name] = _pose(*self._static[name])
            elif name in engine_poses and _finite(engine_poses[name]):
                poses[name] = engine_poses[name]
            else:
                self.get_logger().warn(
                    f'no finite engine pose for {name!r}; leaving it out',
                    throttle_duration_sec=5.0,
                )
        states = ObjectStates()
        states.header.stamp = stamp
        states.header.frame_id = self._frame_id
        states.objects = [ObjectState(name=name, pose=pose) for name, pose in poses.items()]
        self._objects_pub.publish(states)
        self._markers_pub.publish(self._markers(poses, stamp))
        if self._monitor is None:
            return
        pose = poses.get(self._monitor.object)
        position = None if pose is None else (pose.position.x, pose.position.y, pose.position.z)
        t = Time.from_msg(stamp).nanoseconds * 1e-9
        with self._lock:
            verdict = None if self._resetting else self._monitor.observe(t, position)
            seed = self._seed
        self._publish_verdict(verdict, seed)

    def _markers(self, poses, stamp):
        markers = MarkerArray()
        for index, obj in enumerate(self._scenario['objects']):
            if obj['name'] in poses:
                markers.markers.append(_marker(
                    'objects', index, obj['shape'], obj['size'], obj['color'],
                    poses[obj['name']], self._frame_id, stamp,
                ))
        for index, region in enumerate(self._scenario.get('regions', [])):
            xyz, rpy = sc.nominal_pose(region)
            markers.markers.append(_marker(
                'regions', index, 'box', region['size'], region['color'],
                _pose(xyz, sc.quat_from_rpy(*rpy)), self._frame_id, stamp,
            ))
        return markers

    def _publish_verdict(self, verdict, seed):
        if verdict is None:
            return
        outcome = Outcome()
        outcome.header.stamp = self.get_clock().now().to_msg()
        outcome.header.frame_id = self._frame_id
        outcome.scenario = self._scenario['name']
        outcome.seed = seed
        outcome.success = verdict.success
        outcome.reason = verdict.reason
        outcome.elapsed_s = float(verdict.elapsed_s)
        self._outcome_pub.publish(outcome)
        self.get_logger().info(
            f'outcome: {"success" if verdict.success else "failure"}: {verdict.reason}'
        )

    def _on_reset(self, request, response):
        poses = sc.spawn_poses(self._scenario, request.seed)
        ok, message = False, ''
        with self._lock:
            # The old episode stops judging while objects move.
            self._resetting = True
        try:
            if poses:
                # Slow: waits on the engine, outside the lock.
                ok, message = self._backend.move(poses)
            else:
                ok = True
        except Exception as error:  # noqa: B902 - any engine failure refuses the reset
            ok, message = False, f'{type(error).__name__}: {error}'
        with self._lock:
            # A refused reset keeps the old seed and the old episode.
            self._resetting = False
            if ok:
                self._seed = request.seed
                if self._monitor is not None:
                    self._monitor.start(self._now_s())
                    self._episode_opened = True
        response.ok = ok
        if ok:
            response.message = f'respawned {len(poses)} object(s) from seed {request.seed}'
            self.get_logger().info(response.message)
        else:
            response.message = f'reset to seed {request.seed} failed: {message}'
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
