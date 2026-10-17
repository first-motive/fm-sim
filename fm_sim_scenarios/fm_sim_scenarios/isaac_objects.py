"""The scenario node's Isaac Sim backend: read and move the stage's free objects.

Poses come from the stage's ROS 2 bridge graph (``isaac_stage.bridge_graph``):
one tf2_msgs/TFMessage per tick on ``isaac_stage.OBJECT_POSES_TOPIC``, a
transform per free object from ``World``, named by its child frame. A move
publishes the target poses on ``isaac_stage.OBJECT_COMMANDS_TOPIC``, which the
stage runner inside Isaac Sim applies at rest on its next step; the move counts
as done once a pose update stamped strictly after the command puts every object
within ``POSITION_TOLERANCE_M`` of its target.

The stage runs Isaac Sim's own Python and ROS 2 libraries, which cannot load
fm_sim_msgs, so the command is a standard message and the acknowledgement is
the poses themselves rather than a service reply.
"""

import threading
import time

from fm_sim_scenarios import isaac_stage
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

MOVE_TIMEOUT_S = 5.0
# A teleported object settles from its spawn under gravity before the update
# that acknowledges it: a millimetre or so for a scenario's spawns.
POSITION_TOLERANCE_M = 0.005
POLL_S = 0.01


def _pose(transform):
    pose = Pose()
    t, r = transform.translation, transform.rotation
    pose.position.x, pose.position.y, pose.position.z = t.x, t.y, t.z
    pose.orientation = r
    return pose


def _stamp_ns(stamp):
    return Time.from_msg(stamp).nanoseconds


class IsaacObjects:
    """Read and move free objects on the Isaac Sim stage."""

    POSES_TOPIC = isaac_stage.OBJECT_POSES_TOPIC
    COMMANDS_TOPIC = isaac_stage.OBJECT_COMMANDS_TOPIC
    WORLD_FRAMES = isaac_stage.WORLD_FRAMES

    def __init__(self, node, on_poses):
        self._node = node
        self._on_poses = on_poses
        self._logger = node.get_logger()
        # The latest poses and their stamp, read by a move on another thread.
        self._lock = threading.Lock()
        self._latest = {}
        self._latest_ns = -1
        node.create_subscription(TFMessage, self.POSES_TOPIC, self._poses, 10)
        self._commands = node.create_publisher(TFMessage, self.COMMANDS_TOPIC, 10)

    def _poses(self, msg):
        poses, stamps = {}, []
        for transform in msg.transforms:
            if transform.header.frame_id not in self.WORLD_FRAMES:
                self._logger.error(
                    f'{self.POSES_TOPIC}: {transform.child_frame_id!r} is in frame '
                    f'{transform.header.frame_id!r}, not the world; skipping',
                    throttle_duration_sec=5.0,
                )
                continue
            poses[transform.child_frame_id] = _pose(transform.transform)
            stamps.append(transform.header.stamp)
        if not poses:
            return
        stamp = max(stamps, key=_stamp_ns)
        with self._lock:
            self._latest = poses
            self._latest_ns = _stamp_ns(stamp)
        self._on_poses(stamp, poses)

    def _reached(self, targets, since_ns):
        with self._lock:
            if self._latest_ns <= since_ns:
                return False
            latest = dict(self._latest)
        for name, (xyz, _) in targets.items():
            pose = latest.get(name)
            if pose is None:
                return False
            position = (pose.position.x, pose.position.y, pose.position.z)
            if any(abs(p - t) > POSITION_TOLERANCE_M for p, t in zip(position, xyz)):
                return False
        return True

    def move(self, poses):
        """Teleport ``{name: (xyz, quat_xyzw)}`` at rest; return ``(ok, message)``."""
        if self._commands.get_subscription_count() == 0:
            return False, f'nothing subscribes {self.COMMANDS_TOPIC}; is Isaac Sim running?'
        now = self._node.get_clock().now()
        command = TFMessage()
        for name, (xyz, quat) in poses.items():
            transform = TransformStamped()
            transform.header.stamp = now.to_msg()
            transform.header.frame_id = isaac_stage.WORLD_FRAMES[0]
            transform.child_frame_id = name
            t, r = transform.transform.translation, transform.transform.rotation
            t.x, t.y, t.z = (float(v) for v in xyz)
            r.x, r.y, r.z, r.w = (float(v) for v in quat)
            command.transforms.append(transform)
        self._commands.publish(command)
        deadline = time.monotonic() + MOVE_TIMEOUT_S
        while not self._reached(poses, now.nanoseconds):
            if time.monotonic() >= deadline:
                return False, (
                    f'objects not at their targets on {self.POSES_TOPIC} '
                    f'{MOVE_TIMEOUT_S:g} s after {self.COMMANDS_TOPIC}'
                )
            time.sleep(POLL_S)
        return True, f'moved {len(poses)} object(s)'
