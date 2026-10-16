"""ee_follower: follow a commanded tool pose with damped-least-squares IK.

    /robot_description  ─┐
    /joint_states       ─┼─▶ ee_follower ──▶ /<arm_controller>/commands
    /commanded_ee_right ─┘

Each tick takes one bounded DLS step from the measured joint positions toward the latest
commanded pose and sends the result to the right arm's forward position controller. It
stands in for the Anvil loader's IK, so a controller that streams poses (classical_control)
drives the sim the way it drives the loader.

A tick sends nothing when any input is missing, stale or not finite (see ``gate``); the
forward position controller then holds its last command. A command whose frame is not the
profile's base_frame, or whose quaternion is not a rotation, is rejected and clears the
last good command.
"""

from __future__ import annotations

import hashlib

from fm_sim_ee_follower.dls import dls_step, StepLimits
from fm_sim_ee_follower.gate import blocked_reason
from fm_sim_ee_follower.kinematics import ArmKinematics, speed_caps
from fm_sim_ee_follower.profile import DEFAULT_PROFILE, load_arm, shipped_profiles
from geometry_msgs.msg import PoseStamped
import numpy as np
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, ParameterType
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

#: The arm this node drives; the contract names only /commanded_ee_right.
ARM = 'right'
COMMAND_TOPIC = f'/commanded_ee_{ARM}'
LOG_THROTTLE_S = 2.0
# The OpenArm sim URDF is about 42 kB; anything near this is not a robot description.
MAX_DESCRIPTION_BYTES = 2 * 1024 * 1024


def rotation_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return the rotation matrix of a quaternion, normalized first.

    Raises:
        ValueError: If the quaternion is not finite or has near-zero norm.
    """
    quat = np.array([x, y, z, w], dtype=float)
    norm = float(np.linalg.norm(quat))
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError(f'quaternion {quat.tolist()} is not a rotation')
    x, y, z, w = quat / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _range(low: float, high: float, description: str) -> ParameterDescriptor:
    return ParameterDescriptor(
        description=description,
        read_only=True,
        floating_point_range=[FloatingPointRange(from_value=low, to_value=high, step=0.0)],
    )


class EeFollower(Node):
    """Stream DLS IK steps toward the commanded pose to the forward position controller."""

    def __init__(self, **kwargs):
        super().__init__('ee_follower', **kwargs)
        profile = self.declare_parameter('robot_profile', DEFAULT_PROFILE, ParameterDescriptor(
            description=f'Shipped robot profile name ({", ".join(shipped_profiles())})',
            read_only=True,
        )).value
        self._rate_hz = self.declare_parameter('rate_hz', 100.0, _range(
            1.0, 1000.0, 'IK step and command rate (Hz)')).value
        self._damping = self.declare_parameter('damping', 0.05, _range(
            1e-4, 1.0, 'DLS damping lambda; larger is steadier near singularities, slower'
        )).value
        self._gain = self.declare_parameter('gain', 0.5, _range(
            0.01, 1.0, 'Fraction of the pose error closed per step')).value
        self._max_joint_step = self.declare_parameter('max_joint_step', 0.02, _range(
            1e-4, 0.5, 'Largest joint change per step (rad); each joint speed bound '
            'times the step period caps it further')).value
        age = 'age = now - header.stamp, or now - arrival when the stamp is zero'
        self._max_joint_state_age = self.declare_parameter('max_joint_state_age', 0.2, _range(
            0.01, 10.0, f'Oldest /joint_states (s) a step may start from; {age}')).value
        self._max_command_age = self.declare_parameter('max_command_age', 0.5, _range(
            0.01, 10.0, f'Oldest {COMMAND_TOPIC} (s) the follower tracks; {age}')).value

        cap_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description=('Speed cap per joint (rad/s) in profile joints order; unset or empty '
                         'uses the URDF velocity limits alone'),
            read_only=True,
        )
        cap = self.declare_parameter(
            'joint_speed_cap', Parameter.Type.DOUBLE_ARRAY, cap_descriptor).value

        self._arm = load_arm(profile, ARM)
        self._speed_caps = speed_caps(cap, len(self._arm.joints))
        self._kinematics: ArmKinematics | None = None
        self._limits: StepLimits | None = None
        self._description_hash: str | None = None
        # Each input with its time in seconds: header stamp, or arrival if unstamped.
        self._joint_state: tuple[np.ndarray, float] | None = None
        self._command: tuple[tuple[np.ndarray, np.ndarray], float] | None = None
        self._last_tick: float | None = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self._on_description, latched)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(PoseStamped, COMMAND_TOPIC, self._on_command, 10)
        self._publisher = self.create_publisher(
            Float64MultiArray, self._arm.command_topic, 10)
        self.create_timer(1.0 / self._rate_hz, self._tick)
        self.get_logger().info(
            f'following {COMMAND_TOPIC} in {self._arm.base_frame} to '
            f'{self._arm.command_topic} ({len(self._arm.joints)} joints, profile {profile})')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _input_time(self, stamp) -> float:
        """Return a header stamp in seconds; a zero stamp falls back to arrival time."""
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self._now()
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_description(self, message: String) -> None:
        text = message.data
        if len(text.encode('utf-8')) > MAX_DESCRIPTION_BYTES:
            self.get_logger().error(
                f'robot description over {MAX_DESCRIPTION_BYTES} bytes rejected')
            return
        digest = hashlib.sha256(text.encode('utf-8')).hexdigest()
        if self._description_hash is not None:
            # The chain is fixed once built: a robot that changes under a running follower
            # is a different robot, and its limits were never checked against this one.
            if digest != self._description_hash:
                self.get_logger().error(
                    'a different robot description was published; keeping the first one',
                    throttle_duration_sec=LOG_THROTTLE_S)
            return
        try:
            kinematics = ArmKinematics(text, self._arm.base_frame, self._arm.ee_frame)
            if kinematics.joint_names != self._arm.joints:
                raise ValueError(f'chain joints {kinematics.joint_names} do not match '
                                 f'profile joints {self._arm.joints}')
            limits = kinematics.step_limits(
                self._max_joint_step, 1.0 / self._rate_hz, self._speed_caps)
        except Exception as error:  # noqa: B902 - any parse failure must hold, not crash
            self._kinematics = None
            self.get_logger().error(f'robot description unusable, not moving: {error}')
            return
        slowed = [name for name, step in zip(kinematics.joint_names, limits.max_step)
                  if step < self._max_joint_step]
        if slowed:
            self.get_logger().warning(
                f'max_joint_step x rate_hz = {self._max_joint_step * self._rate_hz:.3g} rad/s '
                f'is above the speed bound of {slowed}; those joints step slower')
        self._limits = limits
        self._kinematics = kinematics
        self._description_hash = digest

    def _on_joint_states(self, message: JointState) -> None:
        index = {name: i for i, name in enumerate(message.name)}
        if not all(j in index and index[j] < len(message.position) for j in self._arm.joints):
            return  # Another publisher's subset; this arm's joints are not in it.
        # Kept even when not finite: it replaces the last good state, and dls_step returns
        # no step for it, so bad joint states stop motion rather than reuse old ones.
        q = np.array([message.position[index[j]] for j in self._arm.joints])
        self._joint_state = (q, self._input_time(message.header.stamp))

    def _on_command(self, message: PoseStamped) -> None:
        # A rejected command also clears the last good one: once the sender publishes bad
        # data, the arm stops instead of finishing the move toward an older target.
        frame = message.header.frame_id
        if frame != self._arm.base_frame:
            self._command = None
            self.get_logger().warning(
                f'{COMMAND_TOPIC} in frame {frame!r} rejected; expected '
                f'{self._arm.base_frame!r}', throttle_duration_sec=LOG_THROTTLE_S)
            return
        p, o = message.pose.position, message.pose.orientation
        try:
            rotation = rotation_from_quaternion(o.x, o.y, o.z, o.w)
        except ValueError as error:
            self._command = None
            self.get_logger().warning(
                f'{COMMAND_TOPIC} rejected: {error}', throttle_duration_sec=LOG_THROTTLE_S)
            return
        # A non-finite position is kept like a non-finite joint state: dls_step refuses it.
        target = (np.array([p.x, p.y, p.z]), rotation)
        self._command = (target, self._input_time(message.header.stamp))

    def _tick(self) -> None:
        now = self._now()
        if self._last_tick is not None and now < self._last_tick:
            # A clock that jumps back (a sim reset) makes every stored time meaningless.
            self._joint_state = None
            self._command = None
            self.get_logger().warning('clock jumped back; dropped stored command and state')
        self._last_tick = now
        reason = blocked_reason(
            now,
            has_kinematics=self._kinematics is not None,
            joint_state_time=None if self._joint_state is None else self._joint_state[1],
            command_time=None if self._command is None else self._command[1],
            max_joint_state_age=self._max_joint_state_age,
            max_command_age=self._max_command_age,
        )
        if reason:
            self.get_logger().info(f'holding: {reason}', throttle_duration_sec=LOG_THROTTLE_S)
            return
        q, _ = self._joint_state
        target, _ = self._command
        kinematics = self._kinematics
        q_next = dls_step(q, target, kinematics.fk, kinematics.jacobian,
                          self._damping, self._gain, self._limits)
        if q_next is None:
            self.get_logger().warning(
                'IK step not finite, holding', throttle_duration_sec=LOG_THROTTLE_S)
            return
        self._publisher.publish(Float64MultiArray(data=q_next.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = EeFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
