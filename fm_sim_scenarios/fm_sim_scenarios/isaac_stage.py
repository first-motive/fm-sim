"""Isaac Sim generator: robot URDF + scenario + seed + cameras -> one stage spec.

Isaac Sim 4.5 builds the stage, but nothing here imports it. This module turns
the run's inputs into a JSON stage spec (schema ``STAGE_SCHEMA_VERSION``) that
``fm_sim_backends/isaac/fm_sim_isaac.py``, run inside Isaac Sim's Python,
replays with the omni APIs. Every number, prim path, topic and OmniGraph
connection is decided here, so all of it unit-tests without Isaac Sim. The
runner keeps three decisions of its own: which drive (angular or linear) each
imported joint takes from its joint type, which object teleports it accepts
(free objects, finite poses inside ``object_workspace``), and applying the
OpenCV fisheye model to an ``equidistant`` camera.

Two halves, because ``run.launch.py`` calls the world generator before the
robot description exists:

- ``write`` (``fm_sim_bringup.run.WORLD_GENERATORS['isaac']``): scenario + seed
  -> the scenario objects with their physics, at their seeded spawn.
- ``write_stage`` (``fm_sim_backends``' isaac launch): that file + the robot
  description + the robot's cameras -> the full stage spec, with the URDF
  written beside it for the importer and the ROS 2 bridge graph.

Frames: the stage's ``/World`` is the robot description's root frame (the
``world`` frame of ``/sim/objects``), and the URDF is imported at the stage
origin. Scenario poses are in that frame.

Contact parameters: an object's physics material takes the scenario's friction
as both static and dynamic friction, restitution 0, and friction combine mode
``max``, so an object's own friction governs its contacts unless the other
surface's is higher (PhysX uses the higher-priority combine mode of the pair,
and ``max`` outranks the default ``average``).
"""

import json
import math
import os
import re
import xml.etree.ElementTree as ET

from fm_sim_cameras import cameras_file
from fm_sim_scenarios import scenario as sc

STAGE_SCHEMA_VERSION = 1

# ROS 2 topics. The joint topics are read from the description's
# topic_based_ros2_control systems (``bridge_topics``); these are Isaac's own.
CLOCK_TOPIC = '/clock'
# tf2_msgs/TFMessage, one transform per free object, child_frame_id = its name,
# parent ``World`` (the stage's /World prim). Published by the bridge graph.
OBJECT_POSES_TOPIC = '/isaac/object_poses'
# tf2_msgs/TFMessage the stage runner applies: each transform teleports the free
# object it names, at rest. The scenario node's reset publishes it.
OBJECT_COMMANDS_TOPIC = '/isaac/object_pose_commands'
# Frame ids the stage's world carries: the bridge names /World after its prim.
WORLD_FRAMES = ('World', 'world')

WORLD_PRIM = '/World'
SCENARIO_PRIM = WORLD_PRIM + '/scenario'
# World-fixed cameras (mounted on the robot's root, which is fixed at the origin).
CAMERAS_PRIM = WORLD_PRIM + '/cameras'
GRAPH_PRIM = WORLD_PRIM + '/fm_ros_bridge'
# The stage runner replaces this with the prim the URDF importer reports as the
# articulation root, which depends on how the importer handles the root link.
ARTICULATION_ROOT = '$ARTICULATION_ROOT'

PHYSICS_HZ = 240
RENDER_HZ = 60
GRAVITY = 9.81
FRICTION_COMBINE = 'max'
# Joint position drives on every revolute and prismatic joint, in SI units per
# radian or metre. Step 24 tunes them against the acceptance trials.
DRIVE_STIFFNESS = 1000.0
DRIVE_DAMPING = 50.0
DRIVE_MAX_FORCE = 1000.0
# URDF importer settings (``ImportConfig.set_<key>``). Fixed joints are merged:
# a massless frame (a camera optical frame) kept as its own link would get a
# default mass; cameras are placed on their moving parent link instead
# (``camera_mount``).
URDF_IMPORT = {
    'merge_fixed_joints': True,
    'fix_base': True,
    'import_inertia_tensor': True,
    'self_collision': False,
    'create_physics_scene': False,
    'make_default_prim': False,
    'distance_scale': 1.0,
}
# USD camera lens: a nominal 3 um pixel turns pixel intrinsics into the
# millimetre focal length and apertures USD carries. Any pitch gives the same
# image; this one keeps the numbers near a real sensor's.
PIXEL_PITCH_MM = 0.003
CLIPPING_RANGE_M = (0.01, 100.0)
# The optical frame (z forward, y down) to the USD camera (looks down -z, y up):
# a half turn about x, as quaternion (w, x, y, z).
OPTICAL_TO_USD_WXYZ = (0.0, 1.0, 0.0, 0.0)
# The camera_info distortion models Isaac renders cameras for.
DISTORTION_MODELS = ('plumb_bob', 'equidistant')

# OmniGraph node types, Isaac Sim 4.5 names.
NODE_TYPES = {
    'tick': 'omni.graph.action.OnPlaybackTick',
    'context': 'isaacsim.ros2.bridge.ROS2Context',
    'sim_time': 'isaacsim.core.nodes.IsaacReadSimulationTime',
    'clock': 'isaacsim.ros2.bridge.ROS2PublishClock',
    'joint_state': 'isaacsim.ros2.bridge.ROS2PublishJointState',
    'joint_command': 'isaacsim.ros2.bridge.ROS2SubscribeJointState',
    'articulation': 'isaacsim.core.nodes.IsaacArticulationController',
    'render_product': 'isaacsim.core.nodes.IsaacCreateRenderProduct',
    'camera': 'isaacsim.ros2.bridge.ROS2CameraHelper',
    'transforms': 'isaacsim.ros2.bridge.ROS2PublishTransformTree',
}

_MESH_FILENAME = re.compile(r'(<mesh\b[^>]*?\bfilename=")([^"]*)(")')
_PACKAGE_NAME = re.compile(r'[A-Za-z][A-Za-z0-9_]*')
# Where run.launch.py writes a run's generated world: write_stage writes beside
# it only inside such a directory.
WORLD_DIR_PREFIX = 'fm_sim_world_'
# How far beyond the scenario's objects and spawn ranges a teleport may land.
WORKSPACE_MARGIN_M = 0.25
_SHAPES = ('box', 'cylinder', 'sphere')


class StageError(ValueError):
    """Inputs the stage cannot be built from."""


# --- Small rigid-transform helpers (quaternions are (w, x, y, z)) -------------


def _wxyz(quat_xyzw):
    x, y, z, w = quat_xyzw
    return (w, x, y, z)


def quat_multiply(a, b):
    """Hamilton product ``a * b`` of quaternions (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_rotate(q, v):
    """Rotate vector ``v`` by unit quaternion ``q`` (w, x, y, z)."""
    w, x, y, z = quat_multiply(quat_multiply(q, (0.0, *v)), (q[0], -q[1], -q[2], -q[3]))
    return (x, y, z)


def compose(parent, child):
    """``parent * child`` of transforms ``(xyz, quat_wxyz)``."""
    (pt, pq), (ct, cq) = parent, child
    rotated = quat_rotate(pq, ct)
    return (
        tuple(p + r for p, r in zip(pt, rotated)),
        quat_multiply(pq, cq),
    )


def _numbers(text, count, what):
    values = [float(v) for v in (text or ' '.join(['0'] * count)).split()]
    if len(values) != count or not all(math.isfinite(v) for v in values):
        raise StageError(f'{what}: expected {count} finite numbers, got {text!r}')
    return values


def _joint_origin(joint):
    origin = joint.find('origin')
    xyz = _numbers(origin.get('xyz') if origin is not None else None, 3, joint.get('name'))
    rpy = _numbers(origin.get('rpy') if origin is not None else None, 3, joint.get('name'))
    return (tuple(xyz), _wxyz(sc.quat_from_rpy(*rpy)))


# --- The scenario half (WORLD_GENERATORS['isaac']) ----------------------------


def _geom(obj):
    """USD prim type, its attributes and the xform scale for an object's shape."""
    shape, size = obj['shape'], obj['size']
    if shape == 'box':
        # A unit cube scaled to the box's full extents.
        return 'Cube', {'size': 1.0}, list(size)
    if shape == 'cylinder':
        return 'Cylinder', {'radius': size[0], 'height': size[1], 'axis': 'Z'}, [1.0, 1.0, 1.0]
    if shape == 'sphere':
        return 'Sphere', {'radius': size[0]}, [1.0, 1.0, 1.0]
    raise StageError(f'{obj["name"]}: unknown shape {shape!r}; shapes: {", ".join(_SHAPES)}')


def object_specs(scenario, seed):
    """Return the stage spec of every scenario object, free ones at their spawn.

    ``scenario`` is a checked catalog entry (``scenario.check``). A static object
    is collision only; a free one is a rigid body with the scenario's mass.
    """
    spawn = sc.spawn_poses(scenario, seed)
    fixed = sc.static_poses(scenario)
    specs = []
    for obj in scenario['objects']:
        name = obj['name']
        if not sc.FRAME_NAME.fullmatch(name):
            raise StageError(f'object name {name!r} is not a valid name')
        static = sc.is_static(obj)
        xyz, quat = fixed[name] if static else spawn[name]
        prim_type, attributes, scale = _geom(obj)
        specs.append({
            'name': name,
            'prim_path': f'{SCENARIO_PRIM}/{name}',
            'prim_type': prim_type,
            'attributes': attributes,
            'scale': scale,
            'translate': list(xyz),
            'orient_wxyz': list(_wxyz(quat)),
            'color': list(obj['color'][:3]),
            'opacity': obj['color'][3],
            'rigid_body': not static,
            'mass': None if static else obj['mass'],
            'material': {
                'static_friction': obj['friction'],
                'dynamic_friction': obj['friction'],
                'restitution': 0.0,
                'friction_combine_mode': FRICTION_COMBINE,
            },
        })
    return specs


def object_workspace(scenario):
    """Return ``{'min': xyz, 'max': xyz}``: where a teleport may put an object.

    The box around every object's yaml position grown by its spawn ranges, plus
    ``WORKSPACE_MARGIN_M``. A reset only moves objects inside their spawn
    ranges, so a target outside is a bad command.
    """
    if not scenario['objects']:
        return {'min': [0.0] * 3, 'max': [0.0] * 3}
    low, high = [math.inf] * 3, [-math.inf] * 3
    for obj in scenario['objects']:
        xyz, _ = sc.nominal_pose(obj)
        spawn = obj.get('spawn', {})
        for axis, key in enumerate(('x', 'y', 'z')):
            lo, hi = spawn.get(key, (0.0, 0.0))
            low[axis] = min(low[axis], xyz[axis] + lo)
            high[axis] = max(high[axis], xyz[axis] + hi)
    return {
        'min': [v - WORKSPACE_MARGIN_M for v in low],
        'max': [v + WORKSPACE_MARGIN_M for v in high],
    }


def generate(scenario, seed):
    """Return the scenario half as a dict; the same inputs give the same dict."""
    sc.check(scenario)
    return {
        'schema_version': STAGE_SCHEMA_VERSION,
        'scenario': scenario['name'],
        'seed': sc.check_seed(seed),
        'objects': object_specs(scenario, seed),
        'object_workspace': object_workspace(scenario),
    }


def _dump(spec, path):
    with open(path, 'w') as spec_file:
        json.dump(spec, spec_file, indent=2, sort_keys=True)
        spec_file.write('\n')


def write(robot_mjcf, mount_xyz, scenario, seed, out_dir):
    """Write the scenario half to ``out_dir/<scenario>_<seed>.isaac.json``; return it.

    The ``WORLD_GENERATORS`` signature: the MJCF model and its mount belong to
    MuJoCo and are not used; Isaac imports the robot's URDF in ``write_stage``.
    """
    del robot_mjcf, mount_xyz
    if not sc.FRAME_NAME.fullmatch(str(scenario['name'])):
        raise sc.ScenarioError(f'scenario name {scenario["name"]!r} is not a valid name')
    spec = generate(scenario, seed)
    path = os.path.join(out_dir, f'{scenario["name"]}_{spec["seed"]}.isaac.json')
    _dump(spec, path)
    return path


# --- The robot half (URDF, cameras, bridge) -----------------------------------


def _confined(directory, relative, uri):
    base = os.path.normpath(directory)
    path = os.path.normpath(os.path.join(base, relative))
    if os.path.commonpath([base, path]) != base or path == base:
        raise StageError(f'mesh {uri!r} leaves its root {base}')
    return path


def mesh_path(uri, mesh_roots, share_directory):
    """Return the absolute file path a description mesh URI names.

    ``mesh_roots`` (``[(prefix, directory)]``, the robot entry's ``mesh_roots``)
    are tried first: they map what a robot's vendored URDF uses (a relative
    ``assets/`` path, a ``package://`` name no package has) to the directory it
    means. Then ``package://<pkg>/`` resolves in that package's share
    (``share_directory``, ``get_package_share_directory`` in the launch), and
    ``file://`` or an absolute path is kept. The result must stay under the root
    or share it resolved against; a relative path no root matches is refused.
    """
    for prefix, directory in mesh_roots:
        if uri.startswith(prefix):
            return _confined(directory, uri[len(prefix):], uri)
    if uri.startswith('package://'):
        package, _, relative = uri[len('package://'):].partition('/')
        if not _PACKAGE_NAME.fullmatch(package):
            raise StageError(f'mesh package {package!r} is not a package name')
        return _confined(share_directory(package), relative, uri)
    if uri.startswith('file://'):
        uri = uri[len('file://'):]
    if os.path.isabs(uri):
        return os.path.normpath(uri)
    raise StageError(f'mesh {uri!r} is relative and no mesh root of the robot matches it')


def resolve_meshes(urdf_text, mesh_roots, share_directory):
    """Return ``urdf_text`` with every ``<mesh filename>`` an absolute file path.

    Isaac's importer reads mesh files from disk, from a copy of the URDF written
    away from the robot's own files (``mesh_path`` resolves each one).
    """
    return _MESH_FILENAME.sub(
        lambda m: m.group(1) + mesh_path(m.group(2), mesh_roots, share_directory) + m.group(3),
        urdf_text,
    )


def parse_mesh_roots(roots):
    """Check ``[[prefix, directory]]`` (the launch's ``mesh_roots``); return tuples."""
    if not isinstance(roots, list) or not all(
        isinstance(root, list) and len(root) == 2 and all(isinstance(v, str) and v for v in root)
        and os.path.isabs(root[1])
        for root in roots
    ):
        raise StageError(
            f'mesh_roots must be a list of [prefix, absolute directory], got {roots!r}'
        )
    return [tuple(root) for root in roots]


def bridge_topics(robot):
    """Return ``(joint_states_topic, joint_commands_topic)`` of the description.

    ``robot`` is the parsed URDF root. Every topic_based_ros2_control system must
    name the same pair: the bridge graph publishes and subscribes one of each.
    """
    pairs = set()
    for system in robot.findall('ros2_control'):
        hardware = system.find('hardware')
        if hardware is None or hardware.findtext('plugin', '').strip() != (
            'topic_based_ros2_control/TopicBasedSystem'
        ):
            raise StageError(
                f'ros2_control system {system.get("name")!r} is not a TopicBasedSystem; '
                'expand the description with sim_backend:=isaac'
            )
        params = {p.get('name'): (p.text or '').strip() for p in hardware.findall('param')}
        pairs.add((params.get('joint_states_topic'), params.get('joint_commands_topic')))
    if len(pairs) != 1:
        raise StageError(f"the description's systems disagree on their topics: {sorted(pairs)}")
    (pair,) = pairs
    if not all(pair):
        raise StageError('a TopicBasedSystem names no joint_states_topic or joint_commands_topic')
    return pair


def camera_mount(robot, frame):
    """Return ``(link, (xyz, quat_wxyz))``: the frame on the link that carries it.

    Walks up from ``frame`` through fixed joints to the first link whose parent
    joint moves; ``link`` is None when the chain reaches the root, which is fixed
    at the stage origin. The transform is ``frame`` in that link.
    """
    parent_joint = {j.find('child').get('link'): j for j in robot.findall('joint')}
    links = {link.get('name') for link in robot.findall('link')}
    if frame not in links:
        raise StageError(f'camera frame {frame!r} is not a link of the description')
    transform = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    link = frame
    while link in parent_joint and parent_joint[link].get('type') == 'fixed':
        joint = parent_joint[link]
        transform = compose(_joint_origin(joint), transform)
        link = joint.find('parent').get('link')
    return (link if link in parent_joint else None), transform


def check_cameras(cameras):
    """Check a robot's cameras file contents (``{'cameras': {role: camera}}``) for Isaac.

    The shared verdict is ``fm_sim_cameras.cameras_file.check`` (roles, frames,
    rates, intrinsics, the image size cap). Isaac adds its own: ``rate_hz`` must
    divide ``RENDER_HZ``, and the distortion model must be one of
    ``DISTORTION_MODELS``. Returns ``{role: camera}`` as the shared check does.
    """
    try:
        checked = cameras_file.check(cameras)
    except cameras_file.CameraError as error:
        raise StageError(str(error)) from error
    for role, camera in checked.items():
        rate = camera['rate_hz']
        if rate > RENDER_HZ or not math.isclose(
            RENDER_HZ / rate, round(RENDER_HZ / rate), abs_tol=1e-6
        ):
            raise StageError(
                f'camera {role}: rate_hz must divide the {RENDER_HZ} Hz render rate '
                f'(e.g. 60, 30, 15, 10), got {rate!r}'
            )
        model = camera['intrinsics']['distortion_model']
        if model not in DISTORTION_MODELS:
            raise StageError(
                f'camera {role}: Isaac cannot render distortion_model {model!r}; '
                f'models: {", ".join(DISTORTION_MODELS)}'
            )
    return checked


def usd_lens(intrinsics):
    """USD camera attributes (millimetres) that render the pinhole of ``intrinsics``.

    USD puts the principal point at the aperture centre shifted by the aperture
    offsets; image u runs with film x, image v against film y.
    """
    width, height = intrinsics['width'], intrinsics['height']
    focal = intrinsics['fx'] * PIXEL_PITCH_MM
    # Pixel pitch along v that makes fy come out of the same focal length.
    pitch_v = focal / intrinsics['fy']
    return {
        'focalLength': focal,
        'horizontalAperture': width * PIXEL_PITCH_MM,
        'verticalAperture': height * pitch_v,
        'horizontalApertureOffset': (width / 2 - intrinsics['cx']) * PIXEL_PITCH_MM,
        'verticalApertureOffset': (intrinsics['cy'] - height / 2) * pitch_v,
        'clippingRange': list(CLIPPING_RANGE_M),
    }


def camera_specs(robot, robot_prim, cameras):
    """Return the stage spec of every camera, on the link that carries its frame."""
    specs = []
    for role, camera in sorted(cameras.items()):
        link, (xyz, quat) = camera_mount(robot, camera['frame'])
        parent = CAMERAS_PRIM if link is None else f'{robot_prim}/{link}'
        frame_skip = max(0, round(RENDER_HZ / camera['rate_hz']) - 1)
        specs.append({
            'role': role,
            'frame_id': camera['frame'],
            'parent_prim': parent,
            'prim_path': f'{parent}/fm_cam_{role}',
            'translate': list(xyz),
            'orient_wxyz': list(quat_multiply(quat, OPTICAL_TO_USD_WXYZ)),
            'lens': usd_lens(camera['intrinsics']),
            'intrinsics': dict(camera['intrinsics']),
            'rate_hz': RENDER_HZ / (frame_skip + 1),
            'frame_skip_count': frame_skip,
            'image_topic': f'/cam_{role}/image_raw',
        })
    return specs


def bridge_graph(joint_states_topic, joint_commands_topic, cameras, objects):
    """Return the OmniGraph that bridges the stage to ROS 2 (Jazzy).

    One playback tick drives: ``/clock`` from simulation time; the articulation's
    joint states out and joint commands in (by joint name, so each per-arm
    system's partial command moves only its own joints); an RGB image per camera;
    and, when the scenario has free objects, their world poses as TF.
    ``nodes`` are ``[name, type]``, ``values`` and ``prim_values`` are
    ``[attribute, value]`` (prim-typed inputs take paths), ``connections`` are
    ``[source, target]`` attributes; the stage runner hands them to
    ``og.Controller.edit`` as they are.
    """
    nodes = [['tick', NODE_TYPES['tick']], ['context', NODE_TYPES['context']],
             ['sim_time', NODE_TYPES['sim_time']], ['clock', NODE_TYPES['clock']],
             ['joint_state', NODE_TYPES['joint_state']],
             ['joint_command', NODE_TYPES['joint_command']],
             ['articulation', NODE_TYPES['articulation']]]
    values = [
        ['clock.inputs:topicName', CLOCK_TOPIC],
        ['joint_state.inputs:topicName', joint_states_topic],
        ['joint_command.inputs:topicName', joint_commands_topic],
        ['context.inputs:useDomainIDEnvVar', True],
    ]
    prim_values = [
        ['joint_state.inputs:targetPrim', [ARTICULATION_ROOT]],
        ['articulation.inputs:targetPrim', [ARTICULATION_ROOT]],
    ]
    connections = [
        ['tick.outputs:tick', 'clock.inputs:execIn'],
        ['tick.outputs:tick', 'joint_state.inputs:execIn'],
        ['tick.outputs:tick', 'joint_command.inputs:execIn'],
        ['tick.outputs:tick', 'articulation.inputs:execIn'],
        ['context.outputs:context', 'clock.inputs:context'],
        ['context.outputs:context', 'joint_state.inputs:context'],
        ['context.outputs:context', 'joint_command.inputs:context'],
        ['sim_time.outputs:simulationTime', 'clock.inputs:timeStamp'],
        ['sim_time.outputs:simulationTime', 'joint_state.inputs:timeStamp'],
        ['joint_command.outputs:jointNames', 'articulation.inputs:jointNames'],
        ['joint_command.outputs:positionCommand', 'articulation.inputs:positionCommand'],
        ['joint_command.outputs:velocityCommand', 'articulation.inputs:velocityCommand'],
        ['joint_command.outputs:effortCommand', 'articulation.inputs:effortCommand'],
    ]
    for camera in cameras:
        render, helper = f'render_{camera["role"]}', f'camera_{camera["role"]}'
        nodes += [[render, NODE_TYPES['render_product']], [helper, NODE_TYPES['camera']]]
        values += [
            [f'{render}.inputs:width', camera['intrinsics']['width']],
            [f'{render}.inputs:height', camera['intrinsics']['height']],
            [f'{helper}.inputs:type', 'rgb'],
            [f'{helper}.inputs:topicName', camera['image_topic']],
            [f'{helper}.inputs:frameId', camera['frame_id']],
            [f'{helper}.inputs:frameSkipCount', camera['frame_skip_count']],
        ]
        prim_values.append([f'{render}.inputs:cameraPrim', [camera['prim_path']]])
        connections += [
            ['tick.outputs:tick', f'{render}.inputs:execIn'],
            [f'{render}.outputs:execOut', f'{helper}.inputs:execIn'],
            [f'{render}.outputs:renderProductPath', f'{helper}.inputs:renderProductPath'],
            ['context.outputs:context', f'{helper}.inputs:context'],
        ]
    free = [obj['prim_path'] for obj in objects if obj['rigid_body']]
    if free:
        nodes.append(['transforms', NODE_TYPES['transforms']])
        values.append(['transforms.inputs:topicName', OBJECT_POSES_TOPIC])
        prim_values += [
            ['transforms.inputs:parentPrim', [WORLD_PRIM]],
            ['transforms.inputs:targetPrims', free],
        ]
        connections += [
            ['tick.outputs:tick', 'transforms.inputs:execIn'],
            ['context.outputs:context', 'transforms.inputs:context'],
            ['sim_time.outputs:simulationTime', 'transforms.inputs:timeStamp'],
        ]
    return {
        'path': GRAPH_PRIM,
        'nodes': nodes,
        'values': values,
        'prim_values': prim_values,
        'connections': connections,
    }


def drives():
    """Joint position drives in USD units: angular stiffness and damping are per degree."""
    per_degree = math.pi / 180.0
    return {
        'angular': {
            'stiffness': DRIVE_STIFFNESS * per_degree,
            'damping': DRIVE_DAMPING * per_degree,
            'max_force': DRIVE_MAX_FORCE,
        },
        'linear': {
            'stiffness': DRIVE_STIFFNESS,
            'damping': DRIVE_DAMPING,
            'max_force': DRIVE_MAX_FORCE,
        },
    }


def stage_spec(world, robot_description, urdf_path, cameras):
    """Return the full stage spec: the scenario half plus robot, cameras and bridge.

    ``world`` is the scenario half (``generate``), ``robot_description`` the
    expanded URDF, ``urdf_path`` where its mesh-resolved copy is written,
    ``cameras`` checked by ``check_cameras`` (``{}`` for none).
    """
    if world.get('schema_version') != STAGE_SCHEMA_VERSION:
        raise StageError(
            f'scenario stage schema_version {world.get("schema_version")!r}, '
            f'expected {STAGE_SCHEMA_VERSION}'
        )
    try:
        robot = ET.fromstring(robot_description)
    except ET.ParseError as error:
        raise StageError(f'robot description is not XML: {error}') from error
    name = robot.get('name', '')
    if not sc.FRAME_NAME.fullmatch(name):
        raise StageError(f'robot name {name!r} is not a valid prim name')
    joint_states_topic, joint_commands_topic = bridge_topics(robot)
    robot_prim = f'/{name}'
    camera_list = camera_specs(robot, robot_prim, cameras)
    return {
        'schema_version': STAGE_SCHEMA_VERSION,
        'scenario': world['scenario'],
        'seed': world['seed'],
        'physics': {'physics_hz': PHYSICS_HZ, 'render_hz': RENDER_HZ, 'gravity': GRAVITY},
        'robot': {
            'name': name,
            'urdf_path': urdf_path,
            'prim_path': robot_prim,
            'import_config': dict(URDF_IMPORT),
            'drives': drives(),
            'camera_links': sorted({
                c['parent_prim'] for c in camera_list if c['parent_prim'] != CAMERAS_PRIM
            }),
        },
        'objects': world['objects'],
        'object_workspace': world['object_workspace'],
        'cameras': camera_list,
        'object_commands_topic': OBJECT_COMMANDS_TOPIC,
        'graph': bridge_graph(
            joint_states_topic, joint_commands_topic, camera_list, world['objects']
        ),
    }


def write_stage(world_path, robot_description, cameras, mesh_roots, share_directory):
    """Write ``robot.urdf`` and ``stage.json`` beside ``world_path``; return the stage path.

    ``world_path`` must sit in a ``WORLD_DIR_PREFIX`` directory (the run's
    private ``tempfile.mkdtemp`` from ``run.launch.py``). ``cameras`` is
    checked by ``check_cameras`` (``{}`` for none); ``mesh_roots`` is
    ``parse_mesh_roots``' result (``[]`` for none);
    ``share_directory`` resolves the description's ``package://`` mesh URIs.
    """
    out_dir = os.path.dirname(os.path.abspath(world_path))
    if not os.path.basename(out_dir).startswith(WORLD_DIR_PREFIX):
        raise StageError(
            f'{world_path} is not in a {WORLD_DIR_PREFIX}* run directory; the stage is '
            'written beside it'
        )
    with open(world_path) as world_file:
        world = json.load(world_file)
    urdf_path = os.path.join(out_dir, 'robot.urdf')
    spec = stage_spec(world, robot_description, urdf_path, cameras)
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(resolve_meshes(robot_description, mesh_roots, share_directory))
    stage_path = os.path.join(out_dir, 'stage.json')
    _dump(spec, stage_path)
    return stage_path
