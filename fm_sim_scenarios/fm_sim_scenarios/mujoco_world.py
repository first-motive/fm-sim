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
"""

import os
import xml.etree.ElementTree as ET

from fm_sim_scenarios import scenario as sc

TORSIONAL_FRICTION = 0.005
ROLLING_FRICTION = 0.0001
OBJECT_PRIORITY = 2
FREE_OBJECT_CONDIM = 4
ROBOT_MODEL = 'robot'


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


def generate(robot_mjcf, mount_xyz, scenario, seed, model_name='fm_sim_world'):
    """Return the MJCF world as a string; the same inputs give the same bytes.

    ``robot_mjcf`` is the robot model's absolute path. ``scenario`` is a checked
    catalog entry (``scenario.check``).
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


def write(robot_mjcf, mount_xyz, scenario, seed, out_dir):
    """Write ``generate(...)`` to ``out_dir/<scenario>_<seed>.xml`` and return the path."""
    if not sc.FRAME_NAME.match(str(scenario['name'])):
        raise sc.ScenarioError(f'scenario name {scenario["name"]!r} is not a valid name')
    path = os.path.join(out_dir, f'{scenario["name"]}_{sc.check_seed(seed)}.xml')
    text = generate(robot_mjcf, mount_xyz, scenario, seed, f'fm_sim_{scenario["name"]}')
    with open(path, 'w') as world_file:
        world_file.write(text)
    return path
