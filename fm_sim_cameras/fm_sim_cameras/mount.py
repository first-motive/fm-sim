"""Where a camera frame sits on a simulated robot, read from its URDF. ROS-free.

A camera's optical frame is a fixed child of some robot link in the description
(fm-robot mounts the OpenArm cameras that way). A simulator attaches the camera to
a body of its own model, so ``fixed_mount`` walks the frame's fixed joints up to
the nearest link the simulator also has as a body, and returns the frame's pose
relative to it. Reaching the description's root instead means the camera is fixed
in the world. A moving joint before either is refused: the camera would need a
body the simulator does not have.
"""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from fm_sim_cameras.cameras_file import CameraError
import numpy as np


@dataclass(frozen=True)
class Mount:
    """``frame``'s pose in ``parent``; ``parent`` None is the description's root."""

    frame: str
    parent: object
    xyz: tuple
    quat_wxyz: tuple


def _rpy_matrix(roll, pitch, yaw):
    """URDF rpy: fixed-axis x, then y, then z."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _origin(joint):
    origin = joint.find('origin')
    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if origin is not None:
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    transform = np.eye(4)
    transform[:3, :3] = _rpy_matrix(*rpy)
    transform[:3, 3] = xyz
    return transform


def quat_wxyz(rotation):
    """Return the unit quaternion (w, x, y, z), w >= 0, of a rotation matrix."""
    m = np.asarray(rotation, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q) / np.linalg.norm(q)
    return tuple(float(v) for v in (q if q[0] >= 0 else -q))


def fixed_mount(urdf_xml, frame, bodies):
    """Return the ``Mount`` of link ``frame`` in the URDF text ``urdf_xml``.

    ``bodies`` holds the link names the simulator has as bodies whose frames match
    the description's.
    """
    robot = ET.fromstring(urdf_xml)
    links = {link.get('name') for link in robot.findall('link')}
    if frame not in links:
        raise CameraError(f'camera frame {frame!r} is not a link of the robot description')
    parent_joint = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    transform = np.eye(4)
    link = frame
    while link not in bodies and link in parent_joint:
        joint = parent_joint[link]
        if joint.get('type') != 'fixed':
            raise CameraError(
                f'camera frame {frame!r} moves with joint {joint.get("name")!r} before it '
                f'reaches a simulator body; bodies: {", ".join(sorted(bodies)) or "none"}'
            )
        transform = _origin(joint) @ transform
        link = joint.find('parent').get('link')
    parent = link if link in bodies else None
    return Mount(
        frame, parent, tuple(float(v) for v in transform[:3, 3]), quat_wxyz(transform[:3, :3])
    )
