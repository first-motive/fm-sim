"""Arm kinematics from a URDF: a KDL chain from the base frame to the tool frame.

The URDF is parsed with urdf_parser_py and the chain is built the way kdl_parser does it,
since Jazzy ships kdl_parser for C++ only. Forward kinematics and the Jacobian come from
PyKDL's recursive solvers. Joint limits come from the URDF.
"""

from __future__ import annotations

import math

from fm_sim_ee_follower.dls import StepLimits
import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF

_ROTARY = ('revolute', 'continuous')


class ArmKinematics:
    """Forward kinematics, Jacobian and limits of the chain ``base_frame`` to ``tip_frame``.

    Attributes:
        joint_names: Movable joints from base to tip.
        lower: Lower position limits, ``-inf`` for a continuous joint.
        upper: Upper position limits, ``inf`` for a continuous joint.
        velocity: Velocity limits; NaN where the URDF gives none or 0 (urdfdom reads an
            omitted velocity as 0).
    """

    def __init__(self, urdf_xml: str, base_frame: str, tip_frame: str):
        """Build the chain.

        Raises:
            ValueError: If either frame is not a link, no chain joins them, or a joint on
                it is of a type the follower does not drive.
        """
        robot = URDF.from_xml_string(urdf_xml)
        links = {link.name for link in robot.links}
        for frame in (base_frame, tip_frame):
            if frame not in links:
                raise ValueError(f'frame {frame!r} is not a link of robot {robot.name!r}')
        try:
            joint_names = robot.get_chain(base_frame, tip_frame, links=False)
        except KeyError as error:
            raise ValueError(f'no chain from {base_frame!r} to {tip_frame!r}') from error
        joints = robot.joint_map
        self.chain = kdl.Chain()
        movable = []
        for name in joint_names:
            joint = joints[name]
            if joint.mimic is not None:
                raise ValueError(f'joint {name!r} is a mimic joint; the follower drives none')
            self.chain.addSegment(_segment(joint))
            if joint.type != 'fixed':
                movable.append(joint)
        if not movable:
            raise ValueError(f'no movable joint between {base_frame!r} and {tip_frame!r}')
        self.joint_names = tuple(j.name for j in movable)
        self.lower, self.upper, self.velocity = _limits(movable)
        self._fk = kdl.ChainFkSolverPos_recursive(self.chain)
        self._jac = kdl.ChainJntToJacSolver(self.chain)

    def fk(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return the tool position and rotation matrix in the base frame."""
        frame = kdl.Frame()
        self._fk.JntToCart(_jnt_array(q), frame)
        position = np.array([frame.p[i] for i in range(3)])
        rotation = np.array([[frame.M[i, j] for j in range(3)] for i in range(3)])
        return position, rotation

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """Return the 6 x n geometric Jacobian, linear rows first, in the base frame."""
        jac = kdl.Jacobian(len(self.joint_names))
        self._jac.JntToJac(_jnt_array(q), jac)
        return np.array([[jac[i, j] for j in range(jac.columns())] for i in range(6)])

    def step_limits(
        self, max_joint_step: float, period: float, speed_caps: tuple[float, ...] | None = None
    ) -> StepLimits:
        """Return the joint limits and per-step bound for steps ``period`` seconds apart.

        A joint's speed bound is its URDF velocity limit, or the smaller of that and its
        speed cap (the node's ``joint_speed_cap``). Its step bound is the smaller of
        ``max_joint_step`` and speed times period.

        Raises:
            ValueError: If a joint has neither a URDF velocity limit nor a speed cap, or the
                caps do not match the chain's joint count.
        """
        speed = self.velocity
        if speed_caps is not None:
            if len(speed_caps) != len(self.joint_names):
                raise ValueError(f'{len(speed_caps)} speed caps for '
                                 f'{len(self.joint_names)} joints')
            speed = np.fmin(speed, np.array(speed_caps))  # fmin skips a NaN velocity
        unbounded = [name for name, s in zip(self.joint_names, speed) if not s > 0.0]
        if unbounded:
            raise ValueError(f'joints {unbounded} have no URDF velocity limit and no '
                             'joint_speed_cap; refusing an unbounded step')
        return StepLimits(self.lower, self.upper, np.minimum(max_joint_step, speed * period))


def _segment(joint) -> kdl.Segment:
    origin = _frame(joint.origin)
    if joint.type == 'fixed':
        kdl_joint = kdl.Joint(joint.name, kdl.Joint.Fixed)
    elif joint.type in (*_ROTARY, 'prismatic'):
        axis = origin.M * kdl.Vector(*(joint.axis or (1.0, 0.0, 0.0)))
        kind = kdl.Joint.RotAxis if joint.type in _ROTARY else kdl.Joint.TransAxis
        kdl_joint = kdl.Joint(joint.name, origin.p, axis, kind)
    else:
        raise ValueError(f'joint {joint.name!r} has type {joint.type!r}; the follower drives '
                         'revolute, continuous and prismatic joints only')
    return kdl.Segment(joint.child, kdl_joint, origin)


def _frame(origin) -> kdl.Frame:
    if origin is None:
        return kdl.Frame()
    xyz = origin.xyz or (0.0, 0.0, 0.0)
    rpy = origin.rpy or (0.0, 0.0, 0.0)
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


def _limits(joints) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lower, upper, velocity = [], [], []
    for joint in joints:
        limit = joint.limit
        if joint.type == 'continuous':
            lower.append(-np.inf)
            upper.append(np.inf)
        elif limit is None or limit.lower is None or limit.upper is None:
            raise ValueError(f'{joint.type} joint {joint.name!r} has no lower and upper limit')
        elif not float(limit.lower) < float(limit.upper):
            # urdfdom reads an omitted lower or upper as 0, so 0..0 is a missing range, and
            # a joint locked by its limits is not one the follower can drive.
            raise ValueError(
                f'{joint.type} joint {joint.name!r} has limits {limit.lower}..{limit.upper}; '
                'lower must be below upper (an omitted lower or upper reads as 0)')
        else:
            lower.append(float(limit.lower))
            upper.append(float(limit.upper))
        speed = float(limit.velocity) if limit is not None and limit.velocity else 0.0
        velocity.append(speed if math.isfinite(speed) and speed > 0.0 else np.nan)
    return np.array(lower), np.array(upper), np.array(velocity)


def _jnt_array(q: np.ndarray) -> kdl.JntArray:
    array = kdl.JntArray(len(q))
    for i, value in enumerate(q):
        array[i] = float(value)
    return array


def speed_caps(values, joint_count: int) -> tuple[float, ...] | None:
    """Validate the node's ``joint_speed_cap`` values: empty means none, else one per joint.

    Raises:
        ValueError: If the values are not ``joint_count`` finite positive speeds (rad/s).
    """
    caps = tuple(float(v) for v in values or ())
    if not caps:
        return None
    if len(caps) != joint_count or not all(math.isfinite(c) and c > 0.0 for c in caps):
        raise ValueError(f'joint_speed_cap {list(caps)} must be {joint_count} positive speeds')
    return caps
