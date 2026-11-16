"""Axol MJCF built at run time from fm-robot's vendored ox_cart description.

fm-robot's ``axol.sim.urdf.xacro`` includes Almond's ``axol_ox_cart.urdf``, which
``fm_description`` installs under ``axol_description/``. Almond ships no MJCF, so
``mjcf_path`` reads that URDF, points its relative mesh paths at the installed files,
lets MuJoCo compile it, and patches in what MuJoCo does not take from it:

- the cart's ``lift`` joint fixed at 0 (fully raised, the CAD pose), as fm-robot
  treats it: no ros2_control system claims it;
- a position actuator on each arm joint, with the motor's impedance gains from the
  URDF's ``<drive stiffness damping>`` (an element MuJoCo's importer ignores), its
  control range the joint's and its force cap the URDF effort limit;
- a position actuator on each ``finger_1_joint`` with the OpenArm loader hand's
  finger gains (the URDF has no finger drive) and the URDF effort limit;
- the implicitfast integrator, which keeps the light wrist stable under those gains;
- a stiff mimic equality, so ``finger_2_joint`` follows ``finger_1_joint`` under load;
- contact excludes between each gripper's two fingers and between the shoulder bar
  (``s1``) and each arm's first link.

MuJoCo takes each arm joint's ``<dynamics friction damping>`` as its frictionloss and
damping. Every body is gravity compensated, as Almond's controller compensates the
arm. Body names are the URDF link names, and the MJCF world is the URDF's ``world``
link, so TF and the scenario frame line up with the description. Because the model is
compiled from the installed URDF, an Almond release picked up by fm-robot's pin
reaches the sim with no change here. The model is written once per content hash
(``fm_sim_models.cache``).
"""

import os
import xml.etree.ElementTree as ET

from fm_sim_models import cache
# The URDF compile and the joint and actuator patches are the OpenArm Anvil model's,
# which is compiled from its description the same way.
from fm_sim_models.openarm_anvil import _child, _compile, _named, _servo, MIMIC_SOLIMP, \
    MIMIC_SOLREF
# The loader hand's finger servo; the Axol's finger is as light and as short-stroked.
from fm_sim_models.openarm_loader import FINGER_KP, FINGER_KV

DESCRIPTION_PACKAGE = 'fm_description'
DESCRIPTION_URDF = ('axol_description', 'axol_ox_cart.urdf')
SIDES = ('left_', 'right_')
ARM_JOINTS = ('s1', 's2', 's3', 'e1', 'e2', 'w1', 'w2')
LIFT_JOINT = 'lift'


def _fail(what):
    raise RuntimeError(f'the Axol description does not match the sim patch: {what}')


def description_urdf():
    """Return the installed ox_cart URDF's path, or raise naming fm_description."""
    try:
        from ament_index_python.packages import (
            get_package_share_directory,
            PackageNotFoundError,
        )
    except ImportError:
        raise RuntimeError(f'{DESCRIPTION_PACKAGE} is not installed (no ament index)') from None
    try:
        share = get_package_share_directory(DESCRIPTION_PACKAGE)
    except PackageNotFoundError:
        raise RuntimeError(f'{DESCRIPTION_PACKAGE} is not installed') from None
    return os.path.join(share, *DESCRIPTION_URDF)


def _resolve_meshes(urdf, urdf_dir):
    """Point every mesh at its file beside the URDF, refusing one outside it or missing."""
    base = os.path.normpath(urdf_dir)
    for mesh in urdf.iter('mesh'):
        relative = mesh.get('filename', '')
        path = os.path.normpath(os.path.join(base, relative))
        if '://' in relative or os.path.commonpath([path, base]) != base:
            _fail(f'mesh {relative!r} is not a path inside {base}')
        if not os.path.isfile(path):
            _fail(f'mesh {relative!r} resolves to {path}, which does not exist')
        mesh.set('filename', path)


def _urdf_joint(urdf, name):
    joint = urdf.find(f"joint[@name='{name}']")
    if joint is None:
        _fail(f'no joint {name!r}')
    return joint


def _numbers(urdf, name, tag, attributes):
    """Return the named attributes of the URDF joint's ``tag`` element, as floats."""
    element = _urdf_joint(urdf, name).find(tag)
    missing = [a for a in attributes if element is None or element.get(a) is None]
    if missing:
        _fail(f'no <{tag} {" ".join(missing)}> on {name!r}')
    return tuple(float(element.get(a)) for a in attributes)


def _limits(urdf, name):
    """Return the URDF joint's (lower, upper, effort)."""
    return _numbers(urdf, name, 'limit', ('lower', 'upper', 'effort'))


def _drive(urdf, name):
    """Return the URDF joint's motor impedance gains (kp, kd) from its <drive>."""
    return _numbers(urdf, name, 'drive', ('stiffness', 'damping'))


def _actuate(root, actuators, urdf, name, kp, kv):
    """Add a position servo on MJCF joint ``name`` over the URDF range.

    MuJoCo writes the compiled model to six significant digits, which moves a limit
    such as 1.570796 to 1.5708; the range is set again at full precision.
    """
    lower, upper, effort = _limits(urdf, name)
    joint = _named(root, 'joint', name)
    joint.set('range', f'{lower:.9g} {upper:.9g}')
    _servo(actuators, joint, kp, kv, effort)


def _stiffen_mimic(root, side):
    """Make the equality that drives finger_2_joint from finger_1_joint stiff, or fail."""
    leader, follower = f'{side}finger_1_joint', f'{side}finger_2_joint'
    for equality in root.iterfind('equality/joint'):
        if (equality.get('joint1'), equality.get('joint2')) == (follower, leader):
            equality.attrib.update({'solref': MIMIC_SOLREF, 'solimp': MIMIC_SOLIMP})
            return
    _fail(f'no equality making {follower} follow {leader}')


def mjcf_text(urdf_path):
    """Return the patched MJCF of the Axol URDF at ``urdf_path``."""
    urdf = ET.parse(urdf_path).getroot()
    _resolve_meshes(urdf, os.path.dirname(os.path.abspath(urdf_path)))
    # Read the drive gains before the compile, which drops the element.
    gains = {f'{side}{j}_0': _drive(urdf, f'{side}{j}_0') for side in SIDES for j in ARM_JOINTS}
    lift = _urdf_joint(urdf, LIFT_JOINT)
    lift.set('type', 'fixed')
    root = _compile(urdf)
    # The URDF's drive damping on the light wrist links is past what explicit Euler
    # integrates stably at the 2 ms step (the e2 joint diverges); implicitfast
    # integrates the actuators' damping implicitly and tracks within a few mrad.
    _child(root, 'option').set('integrator', 'implicitfast')
    for body in root.iter('body'):
        body.set('gravcomp', '1')
    actuators = _child(root, 'actuator')
    contact = _child(root, 'contact')
    for side in SIDES:
        for j in ARM_JOINTS:
            name = f'{side}{j}_0'
            _actuate(root, actuators, urdf, name, *gains[name])
        _actuate(root, actuators, urdf, f'{side}finger_1_joint', FINGER_KP, FINGER_KV)
        _stiffen_mimic(root, side)
        # The two fingers meet when closed; an empty close must still reach its
        # limit, so they pass each other rather than collide.
        ET.SubElement(contact, 'exclude', body1=f'{side}finger_1', body2=f'{side}finger_2')
        # The first arm link sits inside the shoulder bar's hull at every pose. MuJoCo
        # filters parent-child contacts, but not when the parent is welded to the
        # world, as the fixed torso is.
        ET.SubElement(contact, 'exclude', body1='s1', body2=f'{side}s2')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


def mjcf_path(urdf_path=None):
    """Write the patched MJCF of the description and return its path.

    ``urdf_path`` defaults to the ox_cart URDF installed by fm_description; when that
    is not installed, this raises naming it.
    """
    return cache.write(mjcf_text(urdf_path or description_urdf()), 'axol')
