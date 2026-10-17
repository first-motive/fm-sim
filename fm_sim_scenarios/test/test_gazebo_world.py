"""The Gazebo generator: the world SDF, the robot URDF it spawns, the bridge config.

``gz sdf`` (sdformat, the parser gz-sim loads both files with) checks the output;
``gz sdf -p`` also shows where the URDF parser puts a camera sensor once it folds
the optical frame's fixed joint into the parent link.
"""

import math
import os
import subprocess
import xml.etree.ElementTree as ET

from fm_sim_cameras import cameras_file as cameras_file_module
from fm_sim_scenarios import gazebo_world, isaac_stage, scenario as sc
import pytest
import yaml

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
# The cameras file fm_sim_bringup ships, read from the source tree.
CAMERAS_YAML = os.path.join(
    TEST_DIR, '..', '..', 'fm_sim_bringup', 'config', 'openarm', 'cameras.yaml'
)
with open(CAMERAS_YAML) as cameras_file:
    SHIPPED_CAMERAS = cameras_file_module.check(yaml.safe_load(cameras_file))
# The optical frame hangs off the base by a fixed joint with a rotation, as the
# OpenArm's cameras do; the parser folds it into the base.
OPTICAL_XYZ = (0.1, -0.02, 0.5)
OPTICAL_RPY = (-2.4, 0.1, -1.5)
PROBE_URDF = f"""<robot name="probe">
  <link name="world"/>
  <link name="base">
    <inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual><geometry><mesh filename="package://probe_pkg/meshes/base.stl"/></geometry></visual>
    <collision><geometry><mesh filename="assets/base.stl"/></geometry></collision>
  </link>
  <joint name="fix" type="fixed"><parent link="world"/><child link="base"/></joint>
  <link name="cam_optical"/>
  <joint name="cam_joint" type="fixed">
    <parent link="base"/><child link="cam_optical"/>
    <origin xyz="{' '.join(map(str, OPTICAL_XYZ))}" rpy="{' '.join(map(str, OPTICAL_RPY))}"/>
  </joint>
</robot>
"""
ROOTS = [('assets/', '/vendored/probe/assets')]
CAMERA = {
    'frame': 'cam_optical', 'rate_hz': 10,
    'intrinsics': {
        'width': 640, 'height': 480, 'fx': 500.0, 'fy': 510.0, 'cx': 320.5, 'cy': 240.5,
        'distortion_model': 'equidistant', 'distortion': [0.1, -0.02, 0.0, 0.0],
    },
}


def _share(package):
    return f'/share/{package}'


def _gz_sdf(*args):
    return subprocess.run(['gz', 'sdf', *args], capture_output=True, text=True, timeout=60)


def _world(scenario, seed=0):
    return ET.fromstring(gazebo_world.generate_world(scenario, seed)).find('world')


def _model(world, name):
    (model,) = [m for m in world.findall('model') if m.get('name') == name]
    return model


def test_same_seed_gives_the_same_bytes(can_on_table):
    first = gazebo_world.generate_world(can_on_table, 5)
    assert first == gazebo_world.generate_world(can_on_table, 5)
    assert first != gazebo_world.generate_world(can_on_table, 6)


def test_free_objects_spawn_from_the_seed(can_on_table):
    world = _world(can_on_table, 7)
    spawn = sc.spawn_poses(can_on_table, 7)
    for name in ('can', 'ball'):
        model = _model(world, name)
        assert model.find('static').text == 'false'
        pose = model.find('pose')
        assert pose.get('rotation_format') == 'quat_xyzw'
        xyz, quat = spawn[name]
        assert [float(v) for v in pose.text.split()] == pytest.approx([*xyz, *quat])
        (plugin,) = model.findall('plugin')
        assert plugin.get('filename') == 'gz-sim-pose-publisher-system'
        assert plugin.find('publish_model_pose').text == 'true'
        assert plugin.find('publish_link_pose').text == 'false'
    assert gazebo_world.free_objects(gazebo_world.generate_world(can_on_table, 7)) == [
        'can', 'ball'
    ]


def test_static_objects_keep_their_yaml_pose(can_on_table):
    table = _model(_world(can_on_table), 'table')
    assert table.find('static').text == 'true'
    assert table.findall('plugin') == []
    assert table.find('link/inertial') is None
    assert [float(v) for v in table.find('pose').text.split()] == [0.5, 0.0, 0.187, 0, 0, 0, 1]
    assert table.find('link/collision/geometry/box/size').text == '0.7 1.2 0.04'


def test_free_objects_carry_mass_inertia_and_friction(can_on_table):
    link = _model(_world(can_on_table), 'can').find('link')
    mass, radius, length = 0.35, 0.033, 0.135
    assert float(link.find('inertial/mass').text) == mass
    side = mass * (3 * radius ** 2 + length ** 2) / 12
    inertia = link.find('inertial/inertia')
    assert float(inertia.find('ixx').text) == pytest.approx(side)
    assert float(inertia.find('iyy').text) == pytest.approx(side)
    assert float(inertia.find('izz').text) == pytest.approx(mass * radius ** 2 / 2)
    cylinder = link.find('collision/geometry/cylinder')
    assert (float(cylinder.find('radius').text), float(cylinder.find('length').text)) == (
        radius, length
    )
    ode = link.find('collision/surface/friction/ode')
    assert (float(ode.find('mu').text), float(ode.find('mu2').text)) == (1.0, 1.0)
    ball = _model(_world(can_on_table), 'ball').find('link')
    assert float(ball.find('inertial/inertia/ixx').text) == pytest.approx(2 * 0.05 * 0.02 ** 2 / 5)
    assert float(ball.find('collision/surface/friction/ode/mu').text) == 0.5


def test_world_loads_the_systems_a_run_needs(can_on_table):
    world = _world(can_on_table)
    assert world.get('name') == gazebo_world.WORLD_NAME
    names = {plugin.get('name') for plugin in world.findall('plugin')}
    assert {
        'gz::sim::systems::Physics', 'gz::sim::systems::UserCommands',
        'gz::sim::systems::SceneBroadcaster', 'gz::sim::systems::Sensors',
    } <= names
    assert float(world.find('physics/max_step_size').text) == gazebo_world.STEP_SIZE_S


def test_world_is_valid_sdf(can_on_table, tmp_path):
    path = gazebo_world.write('/unused.xml', [0, 0, 0.698], can_on_table, 3, str(tmp_path))
    assert path == str(tmp_path / 'can_on_table_3.sdf')
    result = _gz_sdf('-k', path)
    assert 'Valid' in result.stdout, result.stdout + result.stderr


def test_write_refuses_a_name_that_is_not_a_frame_name(can_on_table, tmp_path):
    can_on_table['name'] = '../escape'
    with pytest.raises(sc.ScenarioError, match='not a valid name'):
        gazebo_world.write('/unused.xml', [0, 0, 0], can_on_table, 0, str(tmp_path))
    assert list(tmp_path.iterdir()) == []


def test_meshes_resolve_to_absolute_files():
    robot = ET.fromstring(gazebo_world.robot_urdf(PROBE_URDF, {}, ROOTS, _share))
    assert [mesh.get('filename') for mesh in robot.iter('mesh')] == [
        'file:///share/probe_pkg/meshes/base.stl',
        'file:///vendored/probe/assets/base.stl',
    ]


def test_a_relative_mesh_without_a_root_is_refused():
    with pytest.raises(isaac_stage.StageError, match="'assets/base.stl' is relative"):
        gazebo_world.robot_urdf(PROBE_URDF, {}, [], _share)


def test_a_camera_on_a_missing_frame_is_refused():
    camera = {**CAMERA, 'frame': 'no_such_link'}
    with pytest.raises(sc.ScenarioError, match="frame 'no_such_link' is not a link"):
        gazebo_world.robot_urdf(PROBE_URDF, {'chest': camera}, ROOTS, _share)


def test_optical_rotation_puts_the_gazebo_view_on_optical_z():
    # Columns of R are the sensor axes in the optical frame.
    rotation = sc._rotation_from_rpy(*gazebo_world.OPTICAL_TO_SENSOR_RPY)
    columns = [[rotation[row][col] for row in range(3)] for col in range(3)]
    assert columns[0] == pytest.approx([0, 0, 1])   # looks along optical +z
    assert columns[1] == pytest.approx([-1, 0, 0])  # left is optical -x
    assert columns[2] == pytest.approx([0, -1, 0])  # up is optical -y


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def test_camera_sensor_renders_the_intrinsics_from_the_optical_frame(tmp_path):
    path = tmp_path / 'probe.urdf'
    path.write_text(gazebo_world.robot_urdf(PROBE_URDF, {'chest': CAMERA}, ROOTS, _share))
    result = _gz_sdf('-p', str(path))
    assert result.returncode == 0, result.stderr
    sdf = ET.fromstring(result.stdout)
    (link,) = [link for link in sdf.iter('link') if link.find('sensor') is not None]
    assert link.get('name') == 'base'
    sensor = link.find('sensor')
    assert sensor.get('name') == 'cam_chest'
    assert sensor.find('topic').text == '/cam_chest/image_raw'
    assert sensor.find('gz_frame_id').text == 'cam_optical'
    assert float(sensor.find('update_rate').text) == 10
    camera = sensor.find('camera')
    assert (int(camera.find('image/width').text), int(camera.find('image/height').text)) == (
        640, 480
    )
    lens = camera.find('lens/intrinsics')
    assert [float(lens.find(k).text) for k in ('fx', 'fy', 'cx', 'cy')] == [
        500.0, 510.0, 320.5, 240.5
    ]
    assert float(camera.find('horizontal_fov').text) == pytest.approx(2 * math.atan(320 / 500))
    # Folded into the base: the optical joint's transform, then the optical rotation.
    pose = [float(v) for v in sensor.find('pose').text.split()]
    want = _matmul(
        sc._rotation_from_rpy(*OPTICAL_RPY),
        sc._rotation_from_rpy(*gazebo_world.OPTICAL_TO_SENSOR_RPY),
    )
    assert pose[:3] == pytest.approx(OPTICAL_XYZ)
    got = sc._rotation_from_rpy(*pose[3:])
    assert [v for row in got for v in row] == pytest.approx(
        [v for row in want for v in row], abs=1e-6
    )


def test_pinhole_drops_the_distortion_gazebo_does_not_render():
    cameras = {'chest': CAMERA}
    rendered = gazebo_world.pinhole(cameras)
    assert rendered['chest']['intrinsics']['distortion_model'] == 'plumb_bob'
    assert rendered['chest']['intrinsics']['distortion'] == [0.0] * 5
    assert rendered['chest']['intrinsics']['fx'] == 500.0
    assert cameras['chest']['intrinsics']['distortion_model'] == 'equidistant'
    cameras_file_module.check({'cameras': rendered})


def test_bridge_config_names_every_topic_and_the_set_pose_service():
    entries = gazebo_world.bridge_config(['can'], SHIPPED_CAMERAS)
    topics = {e['ros_topic_name']: e for e in entries if 'ros_topic_name' in e}
    assert set(topics) == {
        '/clock', '/model/can/pose', '/cam_chest/image_raw', '/cam_wrist_right/image_raw',
    }
    assert topics['/model/can/pose']['ros_type_name'] == 'geometry_msgs/msg/PoseStamped'
    assert topics['/cam_chest/image_raw']['gz_type_name'] == 'gz.msgs.Image'
    assert all(e['direction'] == 'GZ_TO_ROS' for e in topics.values())
    (service,) = [e for e in entries if 'service_name' in e]
    assert service['service_name'] == '/world/fm_sim/set_pose'
    assert service['ros_type_name'] == 'ros_gz_interfaces/srv/SetEntityPose'


def _run_dir(tmp_path):
    run_dir = tmp_path / 'fm_sim_world_test'
    run_dir.mkdir()
    return run_dir


def test_write_robot_writes_beside_the_world(can_on_table, tmp_path):
    run_dir = _run_dir(tmp_path)
    world = gazebo_world.write('/unused.xml', [0, 0, 0], can_on_table, 1, str(run_dir))
    robot_file, bridge_file = gazebo_world.write_robot(
        world, PROBE_URDF, 'probe', {'chest': CAMERA}, ROOTS, _share
    )
    assert robot_file == str(run_dir / 'probe.urdf')
    with open(bridge_file) as bridge:
        entries = yaml.safe_load(bridge)
    assert entries == gazebo_world.bridge_config(['can', 'ball'], {'chest': CAMERA})
    with pytest.raises(sc.ScenarioError, match="entity name '../x'"):
        gazebo_world.write_robot(world, PROBE_URDF, '../x', {}, ROOTS, _share)


def test_write_robot_writes_only_in_a_run_directory(can_on_table, tmp_path):
    world = gazebo_world.write('/unused.xml', [0, 0, 0], can_on_table, 1, str(tmp_path))
    with pytest.raises(sc.ScenarioError, match='not in a fm_sim_world_'):
        gazebo_world.write_robot(world, PROBE_URDF, 'probe', {}, ROOTS, _share)
    assert not (tmp_path / 'probe.urdf').exists()


@pytest.mark.parametrize('planted', ['file', 'symlink'])
def test_write_robot_refuses_a_file_already_there(can_on_table, tmp_path, planted):
    run_dir = _run_dir(tmp_path)
    world = gazebo_world.write('/unused.xml', [0, 0, 0], can_on_table, 1, str(run_dir))
    target = tmp_path / 'elsewhere'
    target.write_text('untouched')
    if planted == 'file':
        (run_dir / 'probe.urdf').write_text('planted')
    else:
        (run_dir / 'probe.urdf').symlink_to(target)
    with pytest.raises(OSError):
        gazebo_world.write_robot(world, PROBE_URDF, 'probe', {}, ROOTS, _share)
    assert target.read_text() == 'untouched'


def test_a_symlink_cannot_pose_as_the_run_directory(can_on_table, tmp_path):
    elsewhere = tmp_path / 'elsewhere'
    elsewhere.mkdir()
    world = gazebo_world.write('/unused.xml', [0, 0, 0], can_on_table, 1, str(elsewhere))
    (tmp_path / 'fm_sim_world_fake').symlink_to(elsewhere)
    with pytest.raises(sc.ScenarioError, match='not in a fm_sim_world_'):
        gazebo_world.write_robot(
            str(tmp_path / 'fm_sim_world_fake' / os.path.basename(world)),
            PROBE_URDF, 'probe', {}, ROOTS, _share,
        )


def test_object_names_must_be_frame_names(can_on_table):
    can_on_table['objects'][0]['name'] = 'table/../x'
    with pytest.raises(sc.ScenarioError, match="object name 'table/../x'"):
        gazebo_world.generate_world(can_on_table, 0)
