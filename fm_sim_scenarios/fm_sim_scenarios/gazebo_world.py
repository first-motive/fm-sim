"""Gazebo generator: the scenario half (world SDF) and the robot half of a Gazebo run.

The scenario half, ``write`` (the ``WORLD_GENERATORS`` signature), is the world SDF
(world ``WORLD_NAME``): physics, the systems the run needs, a ground plane, light,
and the scenario's objects. Static objects are static models; the others are free
models spawned at ``scenario.spawn_poses(scenario, seed)``, each with a
PosePublisher reporting its pose in the world frame.

The robot half, ``write_robot``, is written beside it by the Gazebo backend launch
from the robot description and the robot's cameras file:

- the robot URDF Gazebo spawns: the description with every mesh URI made an
  absolute ``file://`` path (Gazebo resolves neither ``package://`` nor a path
  relative to a URDF it does not read from its own directory) and a pinhole
  camera sensor with the real fx, fy, cx, cy on each camera's optical frame. The
  published ``/robot_description`` stays as xacro made it.
- the ros_gz_bridge config: ``/clock``, each free object's pose, each camera's
  raw image, and the world's ``set_pose`` service the scenario node respawns
  objects through.

Gazebo renders no lens distortion. ``pinhole`` turns the cameras into what it
renders, the real intrinsics with zero ``plumb_bob`` distortion, so the
camera_info matches the image.

The robot is spawned at the world origin, so the Gazebo world is the description's
root frame and scenario poses need no mount offset.

Contact parameters: each object's collision takes the scenario's friction as both
directions of Gazebo's friction pyramid (``surface/friction/ode`` ``mu`` and
``mu2``, which the default DART engine reads). Free objects carry the mass and the
solid-body inertia of their shape. Physics steps at ``STEP_SIZE_S``.
"""

import copy
import math
import os
import xml.etree.ElementTree as ET

from fm_sim_scenarios import isaac_stage, scenario as sc
import yaml

WORLD_NAME = 'fm_sim'
STEP_SIZE_S = 0.001
POSE_RATE_HZ = 30
CLIP_NEAR_M = 0.02
CLIP_FAR_M = 10.0
# Gazebo's camera looks along its +x with +z up; the optical frame looks along +z
# with +y down. This rotation, as fixed-axis roll, pitch, yaw, puts the sensor's +x
# on the optical +z, its +y on optical -x and its +z on optical -y.
OPTICAL_TO_SENSOR_RPY = (math.pi / 2, -math.pi / 2, 0.0)
SET_POSE_SERVICE = f'/world/{WORLD_NAME}/set_pose'
_POSE_PUBLISHER = 'gz::sim::systems::PosePublisher'


def pose_topic(name):
    """Topic a free object's PosePublisher publishes on (and the bridge's ROS name)."""
    return f'/model/{name}/pose'


def image_topic(role):
    """Raw image topic of camera ``role``; the camera relays compress it."""
    return f'/cam_{role}/image_raw'


def _numbers(values):
    return ' '.join(f'{float(v):.9g}' for v in values)


def _text(parent, tag, text):
    element = ET.SubElement(parent, tag)
    element.text = str(text)
    return element


def _geometry(parent, obj):
    geometry = ET.SubElement(parent, 'geometry')
    shape, size = obj['shape'], obj['size']
    if shape == 'box':
        _text(ET.SubElement(geometry, 'box'), 'size', _numbers(size))
    elif shape == 'cylinder':
        cylinder = ET.SubElement(geometry, 'cylinder')
        _text(cylinder, 'radius', f'{size[0]:.9g}')
        _text(cylinder, 'length', f'{size[1]:.9g}')
    elif shape == 'sphere':
        _text(ET.SubElement(geometry, 'sphere'), 'radius', f'{size[0]:.9g}')
    else:
        raise sc.ScenarioError(f'{obj["name"]}: unknown shape {shape!r}')


def inertia(obj):
    """Diagonal ``(ixx, iyy, izz)`` of a solid box, cylinder (axis z) or sphere."""
    shape, size, mass = obj['shape'], obj['size'], obj['mass']
    if shape == 'box':
        x, y, z = size
        return (mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12,
                mass * (x * x + y * y) / 12)
    if shape == 'cylinder':
        radius, length = size
        side = mass * (3 * radius * radius + length * length) / 12
        return (side, side, mass * radius * radius / 2)
    if shape == 'sphere':
        return (2 * mass * size[0] * size[0] / 5,) * 3
    raise sc.ScenarioError(f'{obj["name"]}: unknown shape {shape!r}')


def _object_model(world, obj, xyz, quat_xyzw, static):
    model = ET.SubElement(world, 'model', {'name': obj['name']})
    _text(model, 'static', 'true' if static else 'false')
    pose = _text(model, 'pose', _numbers((*xyz, *quat_xyzw)))
    pose.set('rotation_format', 'quat_xyzw')
    link = ET.SubElement(model, 'link', {'name': 'link'})
    if not static:
        inertial = ET.SubElement(link, 'inertial')
        _text(inertial, 'mass', f'{obj["mass"]:.9g}')
        ixx, iyy, izz = inertia(obj)
        matrix = ET.SubElement(inertial, 'inertia')
        for key, value in (('ixx', ixx), ('iyy', iyy), ('izz', izz),
                           ('ixy', 0), ('ixz', 0), ('iyz', 0)):
            _text(matrix, key, f'{value:.9g}')
    collision = ET.SubElement(link, 'collision', {'name': 'collision'})
    _geometry(collision, obj)
    ode = ET.SubElement(
        ET.SubElement(ET.SubElement(collision, 'surface'), 'friction'), 'ode'
    )
    _text(ode, 'mu', f'{obj["friction"]:.9g}')
    _text(ode, 'mu2', f'{obj["friction"]:.9g}')
    visual = ET.SubElement(link, 'visual', {'name': 'visual'})
    _geometry(visual, obj)
    material = ET.SubElement(visual, 'material')
    for tag in ('ambient', 'diffuse'):
        _text(material, tag, _numbers(obj['color']))
    if not static:
        publisher = ET.SubElement(model, 'plugin', {
            'filename': 'gz-sim-pose-publisher-system',
            'name': _POSE_PUBLISHER,
        })
        for tag, value in (('publish_model_pose', 'true'), ('publish_link_pose', 'false'),
                           ('publish_nested_model_pose', 'false'),
                           ('use_pose_vector_msg', 'false'),
                           ('update_frequency', POSE_RATE_HZ)):
            _text(publisher, tag, value)


def _plugin(world, filename, name):
    return ET.SubElement(world, 'plugin', {'filename': filename, 'name': name})


def generate_world(scenario, seed):
    """Return the world SDF as a string; the same inputs give the same bytes.

    The Sensors system (ogre2) is always loaded: the robot's camera sensors need
    it, and it loads no render engine while the world has no sensor.
    """
    for obj in scenario['objects']:
        # Each becomes a model, its topic and the set_pose target.
        if not sc.FRAME_NAME.fullmatch(str(obj['name'])):
            raise sc.ScenarioError(f'object name {obj["name"]!r} is not a valid name')
    spawn = sc.spawn_poses(scenario, seed)
    fixed = sc.static_poses(scenario)
    root = ET.Element('sdf', {'version': '1.9'})
    world = ET.SubElement(root, 'world', {'name': WORLD_NAME})
    physics = ET.SubElement(world, 'physics', {'name': 'default', 'type': 'dart'})
    _text(physics, 'max_step_size', f'{STEP_SIZE_S:g}')
    _text(physics, 'real_time_factor', '1')
    _plugin(world, 'gz-sim-physics-system', 'gz::sim::systems::Physics')
    _plugin(world, 'gz-sim-user-commands-system', 'gz::sim::systems::UserCommands')
    _plugin(world, 'gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster')
    sensors = _plugin(world, 'gz-sim-sensors-system', 'gz::sim::systems::Sensors')
    _text(sensors, 'render_engine', 'ogre2')
    scene = ET.SubElement(world, 'scene')
    _text(scene, 'ambient', '0.6 0.6 0.6 1')
    _text(scene, 'background', '0.7 0.7 0.7 1')
    _text(scene, 'shadows', 'false')
    light = ET.SubElement(world, 'light', {'type': 'directional', 'name': 'sun'})
    _text(light, 'cast_shadows', 'false')
    _text(light, 'pose', '0 0 10 0 0 0')
    _text(light, 'diffuse', '0.8 0.8 0.8 1')
    _text(light, 'specular', '0.2 0.2 0.2 1')
    _text(light, 'direction', '-0.5 0.1 -0.9')
    ground = ET.SubElement(world, 'model', {'name': 'ground_plane'})
    _text(ground, 'static', 'true')
    ground_link = ET.SubElement(ground, 'link', {'name': 'link'})
    for tag in ('collision', 'visual'):
        element = ET.SubElement(ground_link, tag, {'name': tag})
        plane = ET.SubElement(ET.SubElement(element, 'geometry'), 'plane')
        _text(plane, 'normal', '0 0 1')
        _text(plane, 'size', '10 10')

    for obj in scenario['objects']:
        name = obj['name']
        if sc.is_static(obj):
            _object_model(world, obj, *fixed[name], static=True)
        else:
            _object_model(world, obj, *spawn[name], static=False)

    ET.indent(root)
    return '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


def horizontal_fov(intrinsics):
    """Horizontal field of view (radians) of a pinhole with these intrinsics."""
    return 2.0 * math.atan(intrinsics['width'] / (2.0 * intrinsics['fx']))


def pinhole(cameras):
    """Return ``cameras`` (``{role: camera}``) as Gazebo renders them: no distortion."""
    rendered = copy.deepcopy(cameras)
    for camera in rendered.values():
        camera['intrinsics']['distortion_model'] = 'plumb_bob'
        camera['intrinsics']['distortion'] = [0.0] * 5
    return rendered


def _camera_sensor(robot, role, camera):
    intrinsics = camera['intrinsics']
    gazebo = ET.SubElement(robot, 'gazebo', {'reference': camera['frame']})
    sensor = ET.SubElement(gazebo, 'sensor', {'name': f'cam_{role}', 'type': 'camera'})
    _text(sensor, 'pose', _numbers((0, 0, 0, *OPTICAL_TO_SENSOR_RPY)))
    _text(sensor, 'always_on', 'true')
    _text(sensor, 'update_rate', f'{camera["rate_hz"]:g}')
    _text(sensor, 'topic', image_topic(role))
    # The image header's frame (Gazebo Harmonic); the URDF parser copies it through.
    _text(sensor, 'gz_frame_id', camera['frame'])
    camera_element = ET.SubElement(sensor, 'camera', {'name': role})
    _text(camera_element, 'horizontal_fov', f'{horizontal_fov(intrinsics):.9g}')
    image = ET.SubElement(camera_element, 'image')
    _text(image, 'width', intrinsics['width'])
    _text(image, 'height', intrinsics['height'])
    _text(image, 'format', 'R8G8B8')
    clip = ET.SubElement(camera_element, 'clip')
    _text(clip, 'near', f'{CLIP_NEAR_M:g}')
    _text(clip, 'far', f'{CLIP_FAR_M:g}')
    lens = ET.SubElement(ET.SubElement(camera_element, 'lens'), 'intrinsics')
    for key in ('fx', 'fy', 'cx', 'cy'):
        _text(lens, key, f'{intrinsics[key]:.9g}')
    _text(lens, 's', '0')


def robot_urdf(robot_description, cameras, mesh_roots, package_share):
    """Return the URDF Gazebo spawns: meshes resolved, a sensor per camera.

    ``cameras`` is ``{role: camera}`` as ``fm_sim_cameras.cameras_file`` loads it;
    each mesh resolves as ``isaac_stage.mesh_path`` resolves it (``mesh_roots`` is
    ``[(prefix, directory)]``). Refuses a camera whose frame is not a link of the
    description.
    """
    robot = ET.fromstring(robot_description)
    for mesh in robot.iter('mesh'):
        path = isaac_stage.mesh_path(mesh.get('filename', ''), mesh_roots, package_share)
        mesh.set('filename', 'file://' + path)
    links = {link.get('name') for link in robot.findall('link')}
    for role, camera in sorted(cameras.items()):
        if camera['frame'] not in links:
            raise sc.ScenarioError(
                f'camera {role!r}: frame {camera["frame"]!r} is not a link of the robot'
            )
        _camera_sensor(robot, role, camera)
    ET.indent(robot)
    return ET.tostring(robot, encoding='unicode') + '\n'


def _topic(ros_name, ros_type, gz_type, gz_name=None):
    return {
        'ros_topic_name': ros_name,
        'gz_topic_name': gz_name or ros_name,
        'ros_type_name': ros_type,
        'gz_type_name': gz_type,
        'direction': 'GZ_TO_ROS',
    }


def free_objects(world_sdf):
    """Names of the free objects in a world ``generate_world`` wrote, in world order."""
    world = ET.fromstring(world_sdf).find('world')
    return [
        model.get('name') for model in world.findall('model')
        if any(p.get('name') == _POSE_PUBLISHER for p in model.findall('plugin'))
    ]


def bridge_config(objects, cameras):
    """Return the ros_gz_bridge config entries for these free objects and cameras."""
    entries = [_topic('/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock')]
    for name in objects:
        entries.append(_topic(pose_topic(name), 'geometry_msgs/msg/PoseStamped', 'gz.msgs.Pose'))
    for role in sorted(cameras):
        entries.append(_topic(image_topic(role), 'sensor_msgs/msg/Image', 'gz.msgs.Image'))
    entries.append({
        'service_name': SET_POSE_SERVICE,
        'ros_type_name': 'ros_gz_interfaces/srv/SetEntityPose',
        'gz_req_type_name': 'gz.msgs.Pose',
        'gz_rep_type_name': 'gz.msgs.Boolean',
    })
    return entries


def _write_new(path, text):
    """Create ``path`` and write ``text``; refuse a file or symlink already there."""
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644)
    with os.fdopen(fd, 'w') as out_file:
        out_file.write(text)


def write(robot_mjcf, mount_xyz, scenario, seed, out_dir):
    """Write the world SDF to ``out_dir/<scenario>_<seed>.sdf``; return its path.

    The ``WORLD_GENERATORS`` signature: the MJCF model and its mount belong to
    MuJoCo and are not used; the backend launch spawns the robot (``write_robot``).
    """
    del robot_mjcf, mount_xyz
    if not sc.FRAME_NAME.fullmatch(str(scenario['name'])):
        raise sc.ScenarioError(f'scenario name {scenario["name"]!r} is not a valid name')
    path = os.path.join(out_dir, f'{scenario["name"]}_{sc.check_seed(seed)}.sdf')
    with open(path, 'w') as world_file:
        world_file.write(generate_world(scenario, seed))
    return path


def write_robot(world_path, robot_description, entity_name, cameras, mesh_roots,
                package_share):
    """Write ``<entity_name>.urdf`` and ``bridge.yaml`` beside the world; return both paths.

    ``world_path`` must sit in an ``isaac_stage.WORLD_DIR_PREFIX`` directory (the
    run's private ``tempfile.mkdtemp`` from ``run.launch.py``), and neither file
    may exist there yet. ``package_share`` resolves ``package://`` meshes
    (``get_package_share_directory`` in the launch).

    ``cameras`` is ``{role: camera}`` (``{}`` for none); ``mesh_roots`` is
    ``[(prefix, directory)]`` (``isaac_stage.parse_mesh_roots``).
    """
    if not sc.FRAME_NAME.fullmatch(str(entity_name)):
        raise sc.ScenarioError(f'entity name {entity_name!r} is not a valid name')
    # Resolved first, so a symlink cannot pass a foreign directory off as the run's.
    out_dir = os.path.dirname(os.path.realpath(world_path))
    if not os.path.basename(out_dir).startswith(isaac_stage.WORLD_DIR_PREFIX):
        raise sc.ScenarioError(
            f'{world_path} is not in a {isaac_stage.WORLD_DIR_PREFIX}* run directory; '
            'the robot half is written beside it'
        )
    with open(world_path) as world_file:
        objects = free_objects(world_file.read())
    robot_file = os.path.join(out_dir, f'{entity_name}.urdf')
    bridge_file = os.path.join(out_dir, 'bridge.yaml')
    _write_new(robot_file, robot_urdf(robot_description, cameras, mesh_roots, package_share))
    _write_new(bridge_file, yaml.safe_dump(bridge_config(objects, cameras), sort_keys=False))
    return robot_file, bridge_file
