"""The follower's motion gate: may this tick move the arm, and if not, why.

ROS-free: the node passes its clock reading and the time of each input as seconds. An
input's time is its header stamp, or its arrival time when the stamp is zero.
"""

from __future__ import annotations


def blocked_reason(
    now: float,
    *,
    has_kinematics: bool,
    joint_state_time: float | None,
    command_time: float | None,
    max_joint_state_age: float,
    max_command_age: float,
) -> str | None:
    """Return why this tick must not move, or None when every input is usable.

    An input is stale when its age is above its maximum or below zero. A negative age
    means a stamp from the future or a clock that jumped back, and neither is trusted.
    """
    if not has_kinematics:
        return 'no usable /robot_description'
    if joint_state_time is None:
        return 'no joint state yet'
    if not 0.0 <= now - joint_state_time <= max_joint_state_age:
        return 'joint state stale'
    if command_time is None:
        return 'no command yet'
    if not 0.0 <= now - command_time <= max_command_age:
        return 'command stale'
    return None
