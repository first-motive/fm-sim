"""The scenario node's Gazebo backend: read and move the world's free objects.

Each free object of the world ``gazebo_world`` generates carries a PosePublisher
reporting its pose in the world frame on ``gazebo_world.pose_topic(name)``,
bridged as PoseStamped. The first free object's update sets the pace: it
publishes every object whose latest pose is from the same moment (within
``FRESH_S`` of sim time); an object with no such pose is left out, so a stale
pose never stands in. A pose that is not finite, not in the world frame, or
stamped more than ``FRESH_S`` ahead of the node's sim clock is dropped. A
scenario with static objects only publishes on a sim-time timer at the
PosePublisher's rate instead.

A move is all or nothing. It sends every object's ``set_pose`` (the world's
bridged ros_gz_interfaces SetEntityPose) at once and waits on them under one
deadline; then, with the old poses dropped, it waits (same deadline) until each
object reports a pose stamped after the answers within ``POSITION_TOLERANCE_M``
of its target: Gazebo answers when it queues a move and applies it a step later.
If any of that fails, every object that moved or may still move is sent back to
its last pose before the move (a refused one never left), the rollback waits to
land the same way, and the move reports failure; one whose forward request went
unanswered is reported as one that may still move, since Gazebo may apply it
after the rollback. Each move has a generation: a reply that arrives after its move gave up
is ignored. Gazebo keeps a teleported model's velocity.
"""

import json
import math
import threading
import time

from fm_sim_scenarios import gazebo_world, scenario as sc
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

MOVE_TIMEOUT_S = 5.0
POLL_S = 0.01
# A teleported object settles from its spawn under gravity before its first pose
# after the move: a millimetre or so for a scenario's spawns.
POSITION_TOLERANCE_M = 0.005


def pose_from_xyz_quat(xyz, quat_xyzw):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
    (
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    ) = (float(v) for v in quat_xyzw)
    return pose


def _xyz_quat(pose):
    p, q = pose.position, pose.orientation
    return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)


def _finite(pose):
    return all(math.isfinite(v) for part in _xyz_quat(pose) for v in part)


class GazeboObjects:
    """Read and move free objects in the Gazebo world."""

    WORLD_FRAME = gazebo_world.WORLD_NAME
    SET_SERVICE = gazebo_world.SET_POSE_SERVICE
    FRESH_S = 0.1

    def __init__(self, node, on_poses):
        self._node = node
        self._on_poses = on_poses
        self._logger = node.get_logger()
        # The node's checked scenario: its free objects are the world's.
        scenario = json.loads(node.get_parameter('scenario').value)
        objects = [obj['name'] for obj in scenario['objects'] if not sc.is_static(obj)]
        self._pace = objects[0] if objects else None
        # State shared by the pose callbacks and a move on another thread.
        self._lock = threading.Lock()
        self._latest = {}           # name -> (sim seconds, Pose); emptied by a move
        self._known = {}            # name -> last accepted Pose; what a move undoes to
        self._ignore_before = {}    # name -> sim seconds; earlier poses are pre-move
        # One per set_pose round (a move and its rollback each take one).
        self._generation = 0
        for name in objects:
            # A group per object: in one shared group the executor can starve the
            # later subscriptions while the first stays ready (seen in the tests).
            # The lock keeps the shared state consistent across them.
            node.create_subscription(
                PoseStamped, gazebo_world.pose_topic(name),
                lambda msg, name=name: self._on_pose(name, msg), 10,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        if not objects:
            node.create_timer(
                1.0 / gazebo_world.POSE_RATE_HZ,
                lambda: on_poses(node.get_clock().now().to_msg(), {}),
            )
        self._set = node.create_client(
            SetEntityPose, self.SET_SERVICE, callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _now_s(self):
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, name, msg):
        topic = gazebo_world.pose_topic(name)
        if msg.header.frame_id != self.WORLD_FRAME:
            self._logger.error(
                f'{topic} is in frame {msg.header.frame_id!r}, not the world '
                f'{self.WORLD_FRAME!r}; skipping', throttle_duration_sec=5.0,
            )
            return
        if not _finite(msg.pose):
            self._logger.error(f'{topic}: pose is not finite; skipping',
                               throttle_duration_sec=5.0)
            return
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if t > self._now_s() + self.FRESH_S:
            self._logger.error(f'{topic}: stamped {t:.3f} s, ahead of the sim clock; skipping',
                               throttle_duration_sec=5.0)
            return
        with self._lock:
            if t < self._ignore_before.get(name, -math.inf):
                return
            self._latest[name] = (t, msg.pose)
            self._known[name] = msg.pose
            if name != self._pace:
                return
            poses = {
                other: pose for other, (other_t, pose) in self._latest.items()
                if abs(other_t - t) <= self.FRESH_S
            }
        self._on_poses(msg.header.stamp, poses)

    def _send(self, poses):
        """Call set_pose for every object at once; return ``{name: future}``."""
        futures = {}
        for name, (xyz, quat) in poses.items():
            request = SetEntityPose.Request()
            request.entity = Entity(name=name, type=Entity.MODEL)
            request.pose = pose_from_xyz_quat(xyz, quat)
            futures[name] = self._set.call_async(request)
        return futures

    def _await(self, condition, deadline):
        while not condition():
            if time.monotonic() >= deadline:
                return False
            time.sleep(POLL_S)
        return True

    def _set_all(self, poses, deadline):
        """Send set_pose for ``poses`` under one deadline.

        Return ``(refused, unanswered)`` names: a refused object did not move; an
        unanswered one may still move once Gazebo gets to it.
        """
        with self._lock:
            self._generation += 1
            generation = self._generation
        futures = self._send(poses)
        # The futures are polled here, not given done-callbacks: those run as
        # executor tasks and can lag behind the deadline on a busy executor.
        self._await(lambda: all(f.done() for f in futures.values()), deadline)
        refused, unanswered = [], []
        with self._lock:
            # Each move reads only its own generation's futures; one that is
            # still pending is dropped, so its late reply reaches no move.
            current = generation == self._generation
        for name, future in futures.items():
            if not current or not future.done():
                self._set.remove_pending_request(future)
                unanswered.append(name)
            elif not (future.result() and future.result().success):
                refused.append(name)
        return refused, unanswered

    def _landed(self, targets, since_s, deadline):
        """Wait until each object has a pose stamped after ``since_s`` at its target.

        Gazebo answers set_pose once it queues the move and applies it on its next
        step, so the first poses after the answer may still show the old place;
        this waits through them until the deadline. Returns the names that did not
        get there.
        """
        def missed():
            with self._lock:
                latest = {name: self._latest.get(name) for name in targets}
            return [
                name for name, (xyz, _) in targets.items()
                if latest[name] is None or latest[name][0] < since_s
                or math.dist(xyz, _xyz_quat(latest[name][1])[0]) > POSITION_TOLERANCE_M
            ]

        self._await(lambda: not missed(), deadline)
        return missed()

    def move(self, poses):
        """Teleport ``{name: (xyz, quat_xyzw)}`` all or nothing; return ``(ok, message)``.

        Another executor thread must serve the client (its own callback group) and
        the pose subscriptions.
        """
        if not poses:
            return True, 'nothing to move'
        if not self._set.service_is_ready():
            return False, f'{self.SET_SERVICE} is not available yet'
        with self._lock:
            missing = [name for name in poses if name not in self._known]
            before = {name: _xyz_quat(self._known[name]) for name in poses if name in self._known}
        if missing:
            # Without a pose to go back to, a failed move could not be undone.
            return False, f'no pose yet for {", ".join(missing)}; not moving'
        deadline = time.monotonic() + MOVE_TIMEOUT_S
        since_s = self._now_s()
        with self._lock:
            for name in poses:
                self._latest.pop(name, None)
                self._ignore_before[name] = since_s
        refused, unanswered = self._set_all(poses, deadline)
        failed = refused + unanswered
        if failed:
            problem = f'{self.SET_SERVICE} refused or did not answer for {", ".join(failed)}'
        else:
            failed = self._landed(poses, self._now_s(), deadline)
            problem = (
                f'{", ".join(failed)} did not land within '
                f'{POSITION_TOLERANCE_M * 1000:g} mm of the target'
            )
        if not failed:
            return True, f'moved {len(poses)} object(s)'
        # Every object that moved or may still move goes back; a refused one never left.
        back = {name: pose for name, pose in before.items() if name not in refused}
        rollback_deadline = time.monotonic() + MOVE_TIMEOUT_S
        rollback_since = self._now_s()
        undone = sum(self._set_all(back, rollback_deadline), [])
        undone += [
            name for name in self._landed(back, rollback_since, rollback_deadline)
            if name not in undone
        ]
        if undone:
            restored = f'could not restore {", ".join(undone)}'
        elif unanswered:
            # Gazebo may still apply the unanswered moves after the rollback.
            restored = (
                f'restored every object, but {", ".join(unanswered)} may still move to '
                'the abandoned target: Gazebo had not answered'
            )
        else:
            restored = 'restored every object to its pose before the move'
        return False, f'{problem}; {restored}'
