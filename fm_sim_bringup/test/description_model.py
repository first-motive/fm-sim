"""Helpers the OpenArm tests share: URDF kinematics, MuJoCo lookups and the skip gate."""

import math

from fm_sim_bringup import catalog
from fm_sim_scenarios import isaac_stage
import numpy as np
import pytest

# The openarm entry runs the private Anvil description: where that is not installed
# the entry is listed as unavailable, and the tests that run it skip.
needs_openarm = pytest.mark.skipif(
    not catalog.load('robots', 'openarm')['available'],
    reason='openarm_v2_anvil_description (private; in the fm-sim-controllers image)',
)


def axis_angle(axis, angle):
    x, y, z = axis / np.linalg.norm(axis)
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def rpy(roll, pitch, yaw):
    def turn(axis, angle):
        return axis_angle(np.array(axis, dtype=float), angle)
    return turn((0, 0, 1), yaw) @ turn((0, 1, 0), pitch) @ turn((1, 0, 0), roll)


def fk(urdf, link, positions):
    """Return the 4x4 pose of ``link`` in the URDF root at joint ``positions``."""
    parent_joint = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    transform = np.eye(4)
    while link in parent_joint:
        joint = parent_joint[link]
        origin = joint.find('origin')
        step = np.eye(4)
        if origin is not None:
            step[:3, :3] = rpy(*[float(v) for v in origin.get('rpy', '0 0 0').split()])
            step[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        motion = np.eye(4)
        value = positions.get(joint.get('name'), 0.0)
        if joint.get('type') != 'fixed':
            axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
            if joint.get('type') == 'prismatic':
                motion[:3, 3] = axis * value
            else:
                motion[:3, :3] = axis_angle(axis, value)
        transform = step @ motion @ transform
        link = joint.find('parent').get('link')
    return transform


def moving_joints(urdf):
    return [j for j in urdf.findall('joint') if j.get('type') in ('revolute', 'prismatic')]


def actuator(model, joint_id):
    """Return the actuator that drives joint ``joint_id``, or None."""
    found = [i for i in range(model.nu) if model.actuator_trnid[i, 0] == joint_id]
    return model.actuator(found[0]) if found else None


def initial_positions(urdf):
    """Return the joints' spawn positions from the description's ros2_control block."""
    return isaac_stage.initial_positions(urdf)


def settle(mujoco, model, data, seconds, hold=None):
    """Step ``seconds`` of sim time, calling ``hold`` before every step."""
    for _ in range(int(seconds / model.opt.timestep)):
        if hold:
            hold()
        mujoco.mj_step(model, data)
