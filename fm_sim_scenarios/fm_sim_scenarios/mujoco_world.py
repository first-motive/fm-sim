"""MuJoCo generator: robot MJCF + scenario + seed -> one MJCF world.

The robot model is attached unchanged (``<model>`` asset + ``<attach>``), so its
own compiler settings, defaults and relative mesh paths keep resolving against its
file. The attach sits in a frame at the robot's ``mjcf_mount_xyz``, which puts the
MuJoCo world on the robot description's root frame; scenario poses are in that
frame. Static objects become world geoms; the others become free bodies spawned
at ``scenario.spawn_poses(scenario, seed)``.

Contact parameters: every object geom takes the scenario's sliding friction with
MuJoCo's default torsional (0.005) and rolling (0.0001) terms. Object geoms have
priority 2, above the robot's collision geoms (priority 1), so an object's own
friction governs its contacts with the fingers. A free object uses condim 4
(torsional friction), which keeps a pinched object from spinning about the
contact normal.

Cameras: a camera fixed in the world (``Camera.body`` None) is a world camera of
the generated file. A camera on a robot body cannot be added to an attached model
from outside it, so ``write`` then attaches a copy of the robot model instead,
written next to the world: the robot's own cameras dropped, the given ones added
to their bodies, and its asset directories made absolute so its meshes still
resolve from the copy's directory.
"""

from dataclasses import dataclass
import os
import xml.etree.ElementTree as ET

from fm_sim_scenarios import scenario as sc

TORSIONAL_FRICTION = 0.005
ROLLING_FRICTION = 0.0001
OBJECT_PRIORITY = 2
FREE_OBJECT_CONDIM = 4
ROBOT_MODEL = 'robot'


@dataclass(frozen=True)
class Camera:
    """A pinhole camera: its pose is the ROS optical frame's (z forward, y down).

    ``body`` is the robot body it rides on, or None for a camera fixed in the world
    (the description's root frame). ``pos`` and ``quat_wxyz`` are relative to it.
    """

    name: str
    body: object
    pos: tuple
    quat_wxyz: tuple
    fovy_deg: float
    resolution: tuple


def _numbers(values):
    return ' '.join(f'{float(v):.9g}' for v in values)


def _geom_size(obj):
    """MJCF ``size`` for an object: half extents for a box, half height for a cylinder."""
    shape, size = obj['shape'], obj['size']
    if shape == 'box':
        return [s / 2 for s in size]
    if shape == 'cylinder':
        return [size[0], size[1] / 2]
    if shape == 'sphere':
        return list(size)
    raise sc.ScenarioError(f'{obj["name"]}: unknown shape {shape!r}')


def _wxyz(quat_xyzw):
    x, y, z, w = quat_xyzw
    return (w, x, y, z)


def _geom(parent, obj, **extra):
    return ET.SubElement(parent, 'geom', {
        'name': obj['name'],
        'type': obj['shape'],
        'size': _numbers(_geom_size(obj)),
        'friction': _numbers((obj['friction'], TORSIONAL_FRICTION, ROLLING_FRICTION)),
        'priority': str(OBJECT_PRIORITY),
        'rgba': _numbers(obj['color']),
        **extra,
    })


def _mujoco_quat(optical_wxyz):
    """Return the MuJoCo camera quaternion: the optical frame turned pi about x.

    MuJoCo looks down its camera's -z with y up; the optical frame looks down +z
    with y down.
    """
    w, x, y, z = optical_wxyz
    return (-x, w, z, -y)


def _camera(parent, camera):
    if not sc.FRAME_NAME.match(camera.name):
        raise sc.ScenarioError(f'camera name {camera.name!r} is not a valid name')
    width, height = camera.resolution
    return ET.SubElement(parent, 'camera', {
        'name': camera.name,
        'pos': _numbers(camera.pos),
        'quat': _numbers(_mujoco_quat(camera.quat_wxyz)),
        'fovy': f'{float(camera.fovy_deg):.12g}',
        'resolution': f'{int(width)} {int(height)}',
    })


def body_names(robot_mjcf):
    """Return the names of every body in the MJCF model at ``robot_mjcf``."""
    root = ET.parse(robot_mjcf).getroot()
    return {body.get('name') for body in root.iter('body') if body.get('name')}


def _absolute_asset_dirs(root, model_dir):
    """Make the compiler's asset directories absolute, resolved as MuJoCo would.

    meshdir and texturedir fall back to assetdir, and every one is relative to the
    model file's directory. Later compiler elements override earlier ones.
    """
    compilers = root.findall('compiler')
    if not compilers:
        compilers = [ET.Element('compiler')]
        root.insert(0, compilers[0])
    effective = {}
    for compiler in compilers:
        for key in ('assetdir', 'meshdir', 'texturedir'):
            if key in compiler.attrib:
                effective[key] = compiler.attrib.pop(key)
    assetdir = os.path.join(model_dir, effective.get('assetdir', ''))
    compilers[0].set('assetdir', assetdir)
    for key in ('meshdir', 'texturedir'):
        value = os.path.join(model_dir, effective[key]) if key in effective else assetdir
        compilers[0].set(key, value)


def robot_with_cameras(robot_mjcf, cameras):
    """Return the robot model at ``robot_mjcf`` with ``cameras`` on their bodies.

    Its own cameras are dropped. Refuses a model with ``<include>`` (an included
    file's own relative paths would not resolve from the copy) and a camera whose
    body the model does not have exactly once.
    """
    if not os.path.isabs(robot_mjcf):
        raise sc.ScenarioError(f'robot MJCF path must be absolute, got {robot_mjcf!r}')
    root = ET.parse(robot_mjcf).getroot()
    if root.find('.//include') is not None:
        raise sc.ScenarioError(f'{robot_mjcf}: a model with <include> cannot take cameras')
    model_dir = os.path.dirname(robot_mjcf)
    _absolute_asset_dirs(root, model_dir)
    for model in root.iter('model'):
        model.set('file', os.path.join(model_dir, model.get('file', '')))
    for parent in root.iter():
        for own in parent.findall('camera'):
            parent.remove(own)
    for camera in cameras:
        bodies = [body for body in root.iter('body') if body.get('name') == camera.body]
        if len(bodies) != 1:
            raise sc.ScenarioError(
                f'camera {camera.name!r}: the robot model has {len(bodies)} bodies named '
                f'{camera.body!r}, not one'
            )
        _camera(bodies[0], camera)
    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


def generate(robot_mjcf, mount_xyz, scenario, seed, model_name='fm_sim_world', world_cameras=()):
    """Return the MJCF world as a string; the same inputs give the same bytes.

    ``robot_mjcf`` is the robot model's absolute path. ``scenario`` is a checked
    catalog entry (``scenario.check``). ``world_cameras`` are cameras fixed in the
    world; a camera on a body is refused here (``write`` puts it in the robot).
    """
    if not os.path.isabs(robot_mjcf):
        raise sc.ScenarioError(f'robot MJCF path must be absolute, got {robot_mjcf!r}')
    spawn = sc.spawn_poses(scenario, seed)
    fixed = sc.static_poses(scenario)

    root = ET.Element('mujoco', {'model': model_name})
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'model', {'name': ROBOT_MODEL, 'file': robot_mjcf})
    world = ET.SubElement(root, 'worldbody')
    mount = ET.SubElement(world, 'frame', {'name': 'robot_mount', 'pos': _numbers(mount_xyz)})
    ET.SubElement(mount, 'attach', {'model': ROBOT_MODEL, 'prefix': ''})
    for camera in world_cameras:
        if camera.body is not None:
            raise sc.ScenarioError(f'camera {camera.name!r} rides on a body; use write')
        _camera(world, camera)

    for obj in scenario['objects']:
        name = obj['name']
        if sc.is_static(obj):
            xyz, quat = fixed[name]
            _geom(world, obj, pos=_numbers(xyz), quat=_numbers(_wxyz(quat)))
            continue
        xyz, quat = spawn[name]
        body = ET.SubElement(world, 'body', {
            'name': name, 'pos': _numbers(xyz), 'quat': _numbers(_wxyz(quat)),
        })
        ET.SubElement(body, 'freejoint', {'name': name})
        _geom(body, obj, mass=_numbers([obj['mass']]), condim=str(FREE_OBJECT_CONDIM))

    ET.indent(root)
    return ET.tostring(root, encoding='unicode') + '\n'


def world_path(scenario, seed, out_dir):
    """Return the path ``write`` writes the world for ``scenario`` and ``seed`` to."""
    if not sc.FRAME_NAME.fullmatch(str(scenario['name'])):
        raise sc.ScenarioError(f'scenario name {scenario["name"]!r} is not a valid name')
    return os.path.join(out_dir, f'{scenario["name"]}_{sc.check_seed(seed)}.xml')


def write(robot_mjcf, mount_xyz, scenario, seed, out_dir, cameras=()):
    """Write the world to ``world_path(...)`` and return that path.

    With ``cameras``, the robot attached is a copy carrying them, written beside
    the world as ``<scenario>_<seed>_robot.xml``.
    """
    path = world_path(scenario, seed, out_dir)
    world_cameras = [camera for camera in cameras if camera.body is None]
    body_cameras = [camera for camera in cameras if camera.body is not None]
    if cameras:
        robot_copy = path[:-len('.xml')] + '_robot.xml'
        with open(robot_copy, 'w') as robot_file:
            robot_file.write(robot_with_cameras(robot_mjcf, body_cameras))
        robot_mjcf = robot_copy
    text = generate(
        robot_mjcf, mount_xyz, scenario, seed, f'fm_sim_{scenario["name"]}', world_cameras
    )
    with open(path, 'w') as world_file:
        world_file.write(text)
    return path
