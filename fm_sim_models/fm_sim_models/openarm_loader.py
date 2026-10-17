"""OpenArm v2 MJCF with the loader hand, patched from the vendored openarm_mujoco.

fm-robot's openarm.sim.urdf.xacro builds the OpenArm fm-rob-01's loader runs by
default (hand:=loader): the v2.0 arm with the loader's joint limits and Enactic's
v1.0 parallel-jaw hand instead of the v2.0 pinch gripper. openarm_mujoco ships that
hand only on its v1 arm, so this module patches the v2 MJCF to match the URDF: body
names are the URDF link names and the finger joints keep the URDF joint names. The
geometry is the loader URDF snapshot's (fm-skills openarm_anvil.urdf, loader image
ros2:1.2.16), with openarm_mujoco v1's meshes, which are the same hand in a global
millimetre frame (hence the large z offsets).

``mjcf_path`` re-derives the model on every call and writes it once per content
hash under ``$XDG_CACHE_HOME/fm_sim_models`` (default ``~/.cache``). Old hashes are
left in place, one file of about 40 kB per patch or upstream change, so a sim
still running from one never loses its model.
"""

import hashlib
import os
import tempfile
import xml.etree.ElementTree as ET

# The loader's arm limits per side, (lower, upper) in rad, as fm-robot's loader
# preset writes them (fm_description/config/openarm/loader_presets): joint1 and
# joint6 differ from v2.0's, the rest only in rounding.
JOINT_RANGES = {
    'right': {
        1: (-2.3562, 2.3562), 2: (-0.1745, 3.3161), 3: (-1.570796, 1.570796),
        4: (0.0, 2.443461), 5: (-1.570796, 1.570796), 6: (-1.22173, 0.785398),
        7: (-1.570796, 1.570796),
    },
    'left': {
        1: (-2.3562, 2.3562), 2: (-3.3161, 0.1745), 3: (-1.570796, 1.570796),
        4: (0.0, 2.443461), 5: (-1.570796, 1.570796), 6: (-0.785398, 1.22173),
        7: (-1.570796, 1.570796),
    },
}
# Each finger's travel in metres, 0 closed to 0.05 open, as the loader commands it.
FINGER_RANGE_M = (0.0, 0.05)
# Finger position actuator, commanded closed (0) on the 66 mm can: the fingers stop
# at about 0.034 m and squeeze with about 17 N each (kp x 0.034), five times the
# can's 3.4 N weight at friction 1. kp 300 also holds it, with less margin; 1000
# reaches the force cap. kv, the joint damping and armature keep the 36 g finger
# overdamped at the 2 ms step; a stiffer kv slows the close.
FINGER_KP = 500.0
FINGER_KV = 10.0
# The force cap is the most a finger ever squeezes: 30 N holds the can at a 9x
# margin, and caps how hard a commanded close drives into anything. A 10 N cap
# lost the can in tests.
FINGER_FORCE_N = 30.0
# MuJoCo's soft contacts let a squeezed can creep down the fingers (about 10 mm/s
# under gravity); noslip holds it. Its cost is world-wide: every contact in the
# scenario world gets the extra iterations, since the world generator copies this
# <option> into the world.
NOSLIP_ITERATIONS = 5

# The hand from <side>_ee_base_link (link7's frame): 100.1 mm out, flipped about x.
_HAND_POS = '0 0 -0.1001'
_HAND_QUAT = '0 1 0 0'
_GRASP_POS = '0 0 0.08'
# The v1.0 wrist (the loader's link7 mesh) in ee_base_link, per side: the mesh sits
# at (0, 0, -0.5585) in the loader's link7, which is ee_base_link rotated half a
# turn about (1, +-1, 0); the left mesh is mirrored in y.
_LINK7_GEOM = {
    'right': {'pos': '0 0 0.5585', 'quat': '0 0.70710678 0.70710678 0', 'mirror': False},
    'left': {'pos': '0 0 0.5585', 'quat': '0 0.70710678 -0.70710678 0', 'mirror': True},
}
# The loader's link7 inertial in ee_base_link, per side (rotated as above). fm-robot's
# openarm_parallel_hand.xacro puts the same wrist on ee_base_link; fm_sim_bringup's
# test_openarm_hand checks the two agree.
_LINK7_INERTIAL = {
    'right': {'pos': '0.0126618 6.8755e-05 -0.0695195',
              'fullinertia': '0.000497 0.000639 0.000342 1e-06 -8.9e-05 -1e-06'},
    'left': {'pos': '0.0126618 -6.8755e-05 -0.0695195',
             'fullinertia': '0.000497 0.000639 0.000342 -1e-06 -8.9e-05 1e-06'},
}
_LINK7_MASS = '0.4659771'
# The two fingers, identical on both arms: (link suffix, joint suffix, side sign).
# finger_joint1 drives the right finger along -y of the hand; finger_joint2 mirrors
# it along +y through the equality the vendored model already has.
_FINGERS = (('right_finger', 'finger_joint1', -1), ('left_finger', 'finger_joint2', 1))
# Meshes the pinch gripper used, dropped with it.
_PINCH_MESH_PREFIXES = ('ee_base_link_', 'finger_inner_', 'finger_outer_')
_SOURCE = ('v2', 'openarm_bimanual.xml')
_V1_MESHES = ('v1', 'meshes')


def _fail(what):
    raise RuntimeError(f'openarm_mujoco does not match the loader-hand patch: {what}')


def _v1_meshes(openarm_mujoco):
    """Return ``(name, file, mirrored)`` for every loader-hand mesh, absolute paths.

    The wrist and the fingers come plain and mirrored in y (left wrist, right
    finger); the hand is symmetric.
    """
    root = os.path.join(openarm_mujoco, *_V1_MESHES)
    if not os.path.isdir(root):
        _fail(f'no v1 mesh directory {root}')
    parts = (
        ('link7', 'arm/link7', (False, True)),
        ('hand', 'gripper/hand', (False,)),
        ('finger', 'gripper/finger', (False, True)),
    )
    meshes = []
    for name, stem, variants in parts:
        directory, base = os.path.split(stem)
        files = {
            '0': f'visual/{directory}/{base}_0.obj',
            '1': f'visual/{directory}/{base}_1.obj',
            'collision': f'collision/{directory}/{base}.stl',
        }
        for part, rel in files.items():
            path = os.path.join(root, rel)
            if not os.path.isfile(path):
                _fail(f'no v1 mesh {path}')
            for mirrored in variants:
                meshes.append((
                    f'loader_{name}_{part}' + ('_mirrored' if mirrored else ''), path, mirrored,
                ))
    return meshes


def _sub(parent, tag, **attributes):
    return ET.SubElement(parent, tag, attributes)


def _one(root, path, what):
    element = root.find(path)
    if element is None:
        _fail(f'no {what}')
    return element


def _named(root, tag, name):
    return _one(root, f".//{tag}[@name='{name}']", f'{tag} {name!r}')


def _mesh_geoms(body, name, mesh, pos, mirrored, materials, quat='1 0 0 0'):
    """Add a mesh's visual parts and its collision geom to ``body``."""
    suffix = '_mirrored' if mirrored else ''
    for index, material in enumerate(materials):
        _sub(body, 'geom', name=f'{name}_visual_{index}', type='mesh',
             mesh=f'loader_{mesh}_{index}{suffix}', pos=pos, quat=quat,
             material=material, **{'class': 'visual'})
    _sub(body, 'geom', name=f'{name}_collision', type='mesh',
         mesh=f'loader_{mesh}_collision{suffix}', pos=pos, quat=quat,
         **{'class': 'collision'})


def _loader_hand(flange, side):
    """Swap the pinch gripper under ``flange`` (<side>_ee_base_link) for the loader hand."""
    prefix = f'openarm_{side}_'
    for child in list(flange):
        # Keep joint7 and the flange site; the pinch base meshes, its inertial, the
        # camera on it and the finger bodies go.
        if child.tag in ('geom', 'camera', 'body', 'inertial'):
            flange.remove(child)
    link7 = _LINK7_GEOM[side]
    flange.insert(0, ET.Element('inertial', {
        'pos': _LINK7_INERTIAL[side]['pos'], 'mass': _LINK7_MASS,
        'fullinertia': _LINK7_INERTIAL[side]['fullinertia'],
    }))
    _mesh_geoms(flange, f'{prefix}link7', 'link7', link7['pos'], link7['mirror'],
                ('metal_silver', 'matte_black'), quat=link7['quat'])

    hand = _sub(flange, 'body', name=f'{prefix}hand', pos=_HAND_POS, quat=_HAND_QUAT)
    _sub(hand, 'inertial', pos='0 0 0.03', mass='0.7',
         diaginertia='0.0002473 1.763e-05 0.0002521')
    _mesh_geoms(hand, f'{prefix}hand', 'hand', '0 0 -0.6585', False,
                ('metal_silver', 'metal_silver'))
    _sub(hand, 'site', name=f'{prefix}grasp_frame', pos=_GRASP_POS, size='0.005',
         rgba='0 0 1 1', group='4')
    lower, upper = FINGER_RANGE_M
    for link, joint, sign in _FINGERS:
        finger = _sub(hand, 'body', name=f'{prefix}{link}', pos=f'0 {0.006 * sign:g} 0.015')
        _sub(finger, 'inertial', pos=f'0.0064528 {0.01702 * sign:g} 0.0219685',
             mass='0.03602545', diaginertia='2.375e-06 2.375e-06 7.5e-07')
        _sub(finger, 'joint', name=f'{prefix}{joint}', type='slide', axis=f'0 {sign} 0',
             range=f'{lower:.9g} {upper:.9g}', limited='true', damping='2', armature='0.01',
             actuatorfrcrange=f'{-FINGER_FORCE_N:g} {FINGER_FORCE_N:g}')
        # The finger mesh is a left finger; the right one is it mirrored in y.
        _mesh_geoms(finger, f'{prefix}{link}', 'finger', f'0 {-0.05 * sign:g} -0.673001',
                    sign < 0, ('pale_silver', 'matte_black'))


def _absolute_meshdir(root, source):
    """Make meshdir absolute, so the patched copy resolves its meshes from anywhere.

    The vendored model sets only meshdir; any other asset directory, or a second
    compiler, would need resolving too, so it is refused.
    """
    compilers = root.findall('compiler')
    if len(compilers) != 1:
        _fail(f'{len(compilers)} <compiler> elements, expected one')
    compiler = compilers[0]
    for key in ('assetdir', 'texturedir'):
        if key in compiler.attrib:
            _fail(f'<compiler {key}> is set; only meshdir is resolved')
    if root.find('.//include') is not None:
        _fail('an <include>, whose relative paths would not resolve from the copy')
    meshdir = compiler.get('meshdir')
    if meshdir is None:
        _fail('no <compiler meshdir>')
    compiler.set('meshdir', os.path.join(os.path.dirname(source), meshdir))
    return compiler


def mjcf_text(openarm_mujoco):
    """Return the v2 bimanual MJCF under ``openarm_mujoco`` patched to the loader hand."""
    source = os.path.join(openarm_mujoco, *_SOURCE)
    root = ET.parse(source).getroot()
    compiler = _absolute_meshdir(root, source)
    option = root.find('option')
    if option is None:
        option = ET.Element('option')
        root.insert(list(root).index(compiler) + 1, option)
    option.set('noslip_iterations', str(NOSLIP_ITERATIONS))

    asset = _one(root, 'asset', '<asset>')
    for mesh in list(asset.findall('mesh')):
        if mesh.get('name').startswith(_PINCH_MESH_PREFIXES):
            asset.remove(mesh)
    for name, path, mirrored in _v1_meshes(openarm_mujoco):
        _sub(asset, 'mesh', name=name, file=path,
             scale='0.001 -0.001 0.001' if mirrored else '0.001 0.001 0.001')

    actuators = {a.get('joint'): a for a in _one(root, 'actuator', '<actuator>')}
    contact = _one(root, 'contact', '<contact>')
    mimics = {(j.get('joint1'), j.get('joint2')) for j in root.iterfind('equality/joint')}
    lower, upper = FINGER_RANGE_M
    for side, ranges in JOINT_RANGES.items():
        prefix = f'openarm_{side}_'
        for index, (low, high) in ranges.items():
            joint = _named(root, 'joint', f'{prefix}joint{index}')
            if joint.get('name') not in actuators:
                _fail(f'no actuator on {joint.get("name")!r}')
            joint.set('range', f'{low:.9g} {high:.9g}')
            actuators[joint.get('name')].set('ctrlrange', f'{low:.9g} {high:.9g}')
        _loader_hand(_named(root, 'body', f'{prefix}ee_base_link'), side)
        finger = actuators.get(f'{prefix}finger_joint1')
        if finger is None:
            _fail(f'no actuator on {prefix}finger_joint1')
        if (f'{prefix}finger_joint1', f'{prefix}finger_joint2') not in mimics:
            _fail(f'no equality making {prefix}finger_joint2 follow finger_joint1')
        finger.attrib.pop('class', None)
        finger.attrib.update({
            'kp': f'{FINGER_KP:g}', 'kv': f'{FINGER_KV:g}',
            'ctrllimited': 'true', 'ctrlrange': f'{lower:.9g} {upper:.9g}',
            'forcelimited': 'true', 'forcerange': f'{-FINGER_FORCE_N:g} {FINGER_FORCE_N:g}',
        })
        # The wrist's convex hull overlaps link5's within the loader's joint6/7
        # limits (5.5 mm at the joint6 limit, 36 mm with joint7 at its limit too),
        # so the pair is excluded, as openarm_mujoco v1 does.
        _sub(contact, 'exclude', body1=f'{prefix}link5', body2=f'{prefix}ee_base_link')
        # The finger meshes overlap by 3.5 mm at 0 (as in openarm_mujoco v1), and an
        # empty close reads about 0 on the real hand, so the fingers pass each other.
        _sub(contact, 'exclude', body1=f'{prefix}right_finger', body2=f'{prefix}left_finger')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


def cache_dir():
    """Return where the patched model is written."""
    base = os.environ.get('XDG_CACHE_HOME') or os.path.join(os.path.expanduser('~'), '.cache')
    return os.path.join(base, 'fm_sim_models')


def mjcf_path(openarm_mujoco):
    """Write the loader-hand MJCF for the vendored ``openarm_mujoco`` and return its path.

    The file is named by its content's hash and replaced atomically, so a caller
    never reads a half-written or stale copy; one that differs from the derived
    text (edited, truncated) is rewritten.
    """
    source = os.path.join(openarm_mujoco, *_SOURCE)
    if not os.path.isfile(source):
        raise RuntimeError(
            f'openarm_mujoco is not vendored: {source} is missing (vcs import fm-sim.repos)'
        )
    text = mjcf_text(openarm_mujoco)
    directory = cache_dir()
    path = os.path.join(
        directory, f'openarm_loader_{hashlib.sha256(text.encode()).hexdigest()[:16]}.xml'
    )
    try:
        with open(path) as cached:
            if cached.read() == text:
                return path
    except FileNotFoundError:
        pass
    os.makedirs(directory, exist_ok=True)
    handle, partial = tempfile.mkstemp(dir=directory, prefix='.openarm_loader-', suffix='.tmp')
    try:
        with os.fdopen(handle, 'w') as model_file:
            model_file.write(text)
        os.chmod(partial, 0o644)
        os.replace(partial, path)
    except BaseException:
        os.unlink(partial)
        raise
    return path
