"""OpenArm v2 MJCF built at run time from Anvil's description of fm-rob-01.

The sim's ``openarm`` robot runs fm-robot's ``openarm_loader.sim.urdf.xacro``, which
includes the URDF of ``openarm_v2_anvil_description``: Anvil's CAD model under the
loader's names (``follower_l_*``, ``follower_r_*``). That package is private and is
installed only in the private fm-sim-controllers image, so this module carries no
geometry. ``mjcf_path`` reads the installed URDF, resolves its ``package://`` mesh
URIs to the installed files, lets MuJoCo compile it, and patches in what a URDF
cannot say:

- a position actuator on each arm joint and on each ``finger_joint1``, its control
  range the joint's and its force cap the URDF effort limit (``FINGER_FORCE_N`` for
  a finger);
- joint damping, armature and friction loss, which the URDF does not carry;
- contact excludes between each hand's two fingers and between ``link5`` and
  ``link7``;
- noslip iterations, so a squeezed can does not creep down the fingers.

MuJoCo makes the URDF's ``<mimic>`` an equality that drives ``finger_joint2`` from
``finger_joint1``; the patch checks it is there and makes it stiff. Body names are the URDF link
names, so camera mounts and TF line up. The model is written once per content hash
(``fm_sim_models.cache``) with absolute mesh paths, on the host where the package
is installed.
"""

from dataclasses import dataclass
import os
import xml.etree.ElementTree as ET

from fm_sim_models import cache
# The loader hand's finger tuning and noslip, shared with the patch of the vendored
# model so the two hands grip alike.
from fm_sim_models.openarm_loader import FINGER_FORCE_N, FINGER_KP, FINGER_KV, NOSLIP_ITERATIONS

DESCRIPTION_PACKAGE = 'openarm_v2_anvil_description'
DESCRIPTION_URDF = ('urdf', 'robot.urdf')
SIDES = ('follower_l_', 'follower_r_')


@dataclass(frozen=True)
class Motor:
    """A joint's position servo (kp, kv) and its drive's damping, armature, friction."""

    kp: float
    kv: float
    damping: float
    armature: float
    frictionloss: float


# Per arm joint, openarm_mujoco v2's actuator gains and the joint parameters of its
# motor class (DM8009 on joints 1 and 2, DM4340 on 3 and 4, DM4310 on 5 and 6,
# DM3507 on 7), which the vendored model this one replaces ran with.
ARM_MOTORS = {
    1: Motor(230.0, 2.7, 1.0, 0.0081, 0.2),
    2: Motor(230.0, 2.7, 1.0, 0.0081, 0.2),
    3: Motor(190.0, 2.2, 0.9, 0.16, 0.1),
    4: Motor(190.0, 2.2, 0.9, 0.16, 0.1),
    5: Motor(30.0, 1.5, 0.9, 0.01, 0.04),
    6: Motor(30.0, 1.5, 0.9, 0.01, 0.04),
    7: Motor(30.0, 1.5, 0.01, 0.0049, 0.01),
}
# Both fingers' joints, as the vendored loader hand has them: with the actuator's kv
# they keep the light finger overdamped at the 2 ms step.
FINGER_DAMPING = 2.0
FINGER_ARMATURE = 0.01
# MuJoCo turns <mimic> into joint1 = polycoef(joint2); the URDF's is a plain copy.
MIMIC_POLYCOEF = (0.0, 1.0, 0.0, 0.0, 0.0)
# The mimic equality made stiff, as openarm_mujoco v2's finger mimic is, so the
# follower finger does not give way when the hand squeezes an object and the two
# fingers close on it together. MuJoCo's default for an imported <mimic> is softer.
MIMIC_SOLREF = '0.005 1'
MIMIC_SOLIMP = '0.95 0.99 0.001'


def _fail(what):
    raise RuntimeError(f'{DESCRIPTION_PACKAGE} does not match the sim patch: {what}')


def _share_directory(package):
    """Return ``package``'s share directory from the ament index."""
    try:
        from ament_index_python.packages import (
            get_package_share_directory,
            PackageNotFoundError,
        )
    except ImportError:
        # A ROS-free context (plain pytest, the native sim-core smoke) has no index.
        raise RuntimeError(f'{package} is not installed (no ament index)') from None
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        raise RuntimeError(
            f'{package} is not installed; it is private and ships only in the '
            'fm-sim-controllers image'
        ) from None


def description_urdf():
    """Return the installed description's URDF path, or raise naming the package."""
    return os.path.join(_share_directory(DESCRIPTION_PACKAGE), *DESCRIPTION_URDF)


def _resolve_meshes(urdf, urdf_dir, share_directory):
    """Point every mesh at its file: ``package://`` from its share, else beside the URDF.

    A mesh must stay inside its share, or the URDF's directory, and every file must
    exist, so MuJoCo never fails deep in its compiler on a path it cannot open.
    """
    for mesh in urdf.iter('mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith('package://'):
            package, _, relative = uri[len('package://'):].partition('/')
            base = share_directory(package)
        else:
            base, relative = urdf_dir, uri.removeprefix('file://')
        base = os.path.normpath(base)
        path = os.path.normpath(os.path.join(base, relative))
        if os.path.commonpath([path, base]) != base:
            _fail(f'mesh {uri!r} resolves outside {base}')
        if not os.path.isfile(path):
            _fail(f'mesh {uri!r} resolves to {path}, which does not exist')
        mesh.set('filename', path)


def _mujoco():
    """The mujoco module, imported without a GL backend.

    Importing mujoco loads the rendering backend MUJOCO_GL names, and a run may set one
    its process cannot load (osmesa with no OSMesa library), which fails the import.
    Compiling renders nothing, so the import goes ahead with GL disabled.
    """
    gl = os.environ.get('MUJOCO_GL')
    os.environ['MUJOCO_GL'] = 'disable'
    try:
        import mujoco
    finally:
        if gl is None:
            del os.environ['MUJOCO_GL']
        else:
            os.environ['MUJOCO_GL'] = gl
    return mujoco


def _compile(urdf):
    """Return the MJCF MuJoCo compiles from ``urdf`` (an ElementTree root)."""
    mujoco = _mujoco()

    block = urdf.find('mujoco')
    if block is None:
        block = ET.SubElement(urdf, 'mujoco')
    compiler = block.find('compiler')
    if compiler is None:
        compiler = ET.SubElement(block, 'compiler')
    # Keep the resolved mesh paths whole, every link a body (the cameras and TF
    # mount on them) and the visual meshes (the cameras render them).
    compiler.attrib.update({'strippath': 'false', 'fusestatic': 'false',
                            'discardvisual': 'false'})
    spec = mujoco.MjSpec.from_string(ET.tostring(urdf, encoding='unicode'))
    spec.compile()
    return ET.fromstring(spec.to_xml())


def _one(root, path, what):
    element = root.find(path)
    if element is None:
        _fail(f'no {what}')
    return element


def _child(root, tag):
    element = root.find(tag)
    return element if element is not None else ET.SubElement(root, tag)


def _named(root, tag, name):
    return _one(root, f".//{tag}[@name='{name}']", f'{tag} {name!r}')


def _joint(root, urdf, name, **attributes):
    """Return MJCF joint ``name`` with ``attributes`` and the URDF's range, and its effort.

    MuJoCo writes the compiled model to six significant digits, which moves a
    limit such as 1.570796 to 1.5708; the range is set again at full precision.
    """
    limit = _one(urdf, f"joint[@name='{name}']/limit", f'<limit> on {name!r}')
    joint = _named(root, 'joint', name)
    joint.attrib.update(attributes)
    joint.set('range', f'{float(limit.get("lower")):.9g} {float(limit.get("upper")):.9g}')
    return joint, float(limit.get('effort'))


def _servo(actuators, joint, kp, kv, force):
    """Add a position actuator on ``joint`` (an MJCF element) over its whole range."""
    ET.SubElement(actuators, 'position', {
        'name': f'{joint.get("name")}_position', 'joint': joint.get('name'),
        'kp': f'{kp:g}', 'kv': f'{kv:g}',
        'ctrllimited': 'true', 'ctrlrange': joint.get('range'),
        'forcelimited': 'true', 'forcerange': f'{-force:g} {force:g}',
    })


def _stiffen_mimic(root, side):
    """Make the equality that drives finger_joint2 from finger_joint1 stiff, or fail."""
    leader, follower = f'{side}finger_joint1', f'{side}finger_joint2'
    for equality in root.iterfind('equality/joint'):
        if (equality.get('joint1'), equality.get('joint2')) == (follower, leader):
            coefficients = [float(v) for v in equality.get('polycoef', '0 1 0 0 0').split()]
            if tuple(coefficients) != MIMIC_POLYCOEF:
                _fail(f'{follower} follows {leader} with polycoef {coefficients}')
            equality.attrib.update({'solref': MIMIC_SOLREF, 'solimp': MIMIC_SOLIMP})
            return
    _fail(f'no equality making {follower} follow {leader}')


def mjcf_text(urdf_path, share_directory=_share_directory):
    """Return the patched MJCF of the URDF at ``urdf_path``.

    ``share_directory`` maps a package name to its share, for ``package://`` meshes.
    """
    urdf = ET.parse(urdf_path).getroot()
    _resolve_meshes(urdf, os.path.dirname(os.path.abspath(urdf_path)), share_directory)
    root = _compile(urdf)
    _child(root, 'option').set('noslip_iterations', str(NOSLIP_ITERATIONS))
    actuators = _child(root, 'actuator')
    contact = _child(root, 'contact')
    for side in SIDES:
        for index, motor in ARM_MOTORS.items():
            joint, effort = _joint(
                root, urdf, f'{side}joint{index}', damping=f'{motor.damping:g}',
                armature=f'{motor.armature:g}', frictionloss=f'{motor.frictionloss:g}',
            )
            _servo(actuators, joint, motor.kp, motor.kv, effort)
        finger = {'damping': f'{FINGER_DAMPING:g}', 'armature': f'{FINGER_ARMATURE:g}'}
        leader, _ = _joint(root, urdf, f'{side}finger_joint1', **finger)
        _joint(root, urdf, f'{side}finger_joint2', **finger)
        _servo(actuators, leader, FINGER_KP, FINGER_KV, FINGER_FORCE_N)
        _stiffen_mimic(root, side)
        # The two fingers may meet when closed; an empty close must still reach
        # zero, so they pass each other rather than collide.
        ET.SubElement(contact, 'exclude', body1=f'{side}right_finger',
                      body2=f'{side}left_finger')
        # The wrist may touch link5 within the joint limits, as in the vendored
        # model, which excludes the pair too.
        ET.SubElement(contact, 'exclude', body1=f'{side}link5', body2=f'{side}link7')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


def mjcf_path(urdf_path=None, share_directory=_share_directory):
    """Write the patched MJCF of the description and return its path.

    ``urdf_path`` defaults to the installed ``openarm_v2_anvil_description``; when
    that is not installed, this raises naming it.
    """
    text = mjcf_text(urdf_path or description_urdf(), share_directory)
    return cache.write(text, 'openarm_anvil')
