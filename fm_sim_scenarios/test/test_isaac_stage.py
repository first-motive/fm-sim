"""The Isaac stage generator: scenario half, URDF, cameras and the bridge graph.

All pure: no Isaac Sim. The OpenArm test expands fm-robot's real description
(skipped when fm_control is not installed) with the cameras file the catalog
ships, so frame names, topics and camera links are checked against the source.
"""

import copy
import json
import math
import os
import pathlib
import tempfile
import xml.etree.ElementTree as ET

from conftest import CAN_ON_TABLE, TEST_DIR
from fm_sim_scenarios import isaac_stage as stage, scenario as sc
import pytest
import yaml

CAMERAS_YAML = os.path.join(
    TEST_DIR, '..', '..', 'fm_sim_bringup', 'config', 'openarm', 'cameras.yaml'
)
TOPICS = ('/isaac_joint_states', '/isaac_joint_commands')


def _system(name, joint, states=TOPICS[0], commands=TOPICS[1]):
    return f"""
  <ros2_control name="{name}" type="system">
    <hardware>
      <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
      <param name="joint_commands_topic">{commands}</param>
      <param name="joint_states_topic">{states}</param>
    </hardware>
    <joint name="{joint}"/>
  </ros2_control>"""


# world -fixed-> base -revolute-> arm -fixed(0.1 z, yaw 90)-> mount -fixed(0.05 x)-> cam
# and base -fixed(0.5 z)-> head_cam: one camera on a moving link, one on the root.
PROBE_URDF = f"""<robot name="probe">
  <link name="world"/><link name="base"/><link name="arm"/><link name="mount"/>
  <link name="cam"/><link name="head_cam"/>
  <joint name="base_joint" type="fixed"><parent link="world"/><child link="base"/></joint>
  <joint name="arm_joint" type="revolute">
    <parent link="base"/><child link="arm"/><origin xyz="0 0 0.2"/>
  </joint>
  <joint name="mount_joint" type="fixed">
    <parent link="arm"/><child link="mount"/><origin xyz="0 0 0.1" rpy="0 0 {math.pi / 2}"/>
  </joint>
  <joint name="cam_joint" type="fixed">
    <parent link="mount"/><child link="cam"/><origin xyz="0.05 0 0"/>
  </joint>
  <joint name="head_joint" type="fixed">
    <parent link="base"/><child link="head_cam"/><origin xyz="0 0 0.5"/>
  </joint>
  <visual><geometry><mesh filename="package://probe_meshes/meshes/arm.stl"/></geometry></visual>
  {_system('left', 'arm_joint')}{_system('right', 'arm_joint')}
</robot>"""

INTRINSICS = {
    'width': 640, 'height': 480, 'fx': 500.0, 'fy': 510.0, 'cx': 330.0, 'cy': 230.0,
    'distortion_model': 'plumb_bob', 'distortion': [0.0, 0.0, 0.0, 0.0, 0.0],
}
CAMERAS = {'cameras': {
    'wrist_right': {'frame': 'cam', 'rate_hz': 30, 'intrinsics': INTRINSICS},
    'chest': {'frame': 'head_cam', 'rate_hz': 10, 'intrinsics': INTRINSICS},
}}


def _robot():
    return ET.fromstring(PROBE_URDF)


def _close(a, b):
    return all(math.isclose(x, y, abs_tol=1e-9) for x, y in zip(a, b))


def test_same_seed_gives_the_same_scenario_half():
    assert stage.generate(CAN_ON_TABLE, 5) == stage.generate(CAN_ON_TABLE, 5)
    assert stage.generate(CAN_ON_TABLE, 5) != stage.generate(CAN_ON_TABLE, 6)


def test_free_objects_spawn_from_the_seed_with_their_physics():
    objects = {o['name']: o for o in stage.generate(CAN_ON_TABLE, 7)['objects']}
    xyz, quat = sc.spawn_poses(CAN_ON_TABLE, 7)['can']
    can = objects['can']
    assert can['translate'] == list(xyz)
    assert can['orient_wxyz'] == [quat[3], quat[0], quat[1], quat[2]]
    assert can['rigid_body'] and can['mass'] == 0.35
    assert can['prim_type'] == 'Cylinder'
    assert can['attributes'] == {'radius': 0.033, 'height': 0.135, 'axis': 'Z'}
    assert can['material'] == {
        'static_friction': 1.0, 'dynamic_friction': 1.0, 'restitution': 0.0,
        'friction_combine_mode': 'max',
    }
    table = objects['table']
    assert not table['rigid_body'] and table['mass'] is None
    assert table['prim_type'] == 'Cube' and table['scale'] == [0.7, 1.2, 0.04]
    assert table['translate'] == [0.5, 0.0, 0.187]
    assert objects['ball']['prim_type'] == 'Sphere'
    assert can['prim_path'] == '/World/scenario/can'


def test_write_names_the_file_by_scenario_and_seed(tmp_path):
    path = stage.write('/unused.xml', [0, 0, 0.698], CAN_ON_TABLE, 9, str(tmp_path))
    assert path == str(tmp_path / 'can_on_table_9.isaac.json')
    with open(path) as world_file:
        assert json.load(world_file) == stage.generate(CAN_ON_TABLE, 9)


@pytest.mark.parametrize('name', ['../escape', 'can_on_table\n'])
def test_write_refuses_a_name_that_is_not_a_frame_name(tmp_path, name):
    scenario = copy.deepcopy(CAN_ON_TABLE)
    scenario['name'] = name
    with pytest.raises(sc.ScenarioError, match='not a valid name'):
        stage.write('/unused.xml', [0, 0, 0], scenario, 0, str(tmp_path))
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize('uri', [
    'package://probe_meshes/../other/m.stl',
    'package://probe_meshes/meshes/../../../etc/passwd',
    'package://probe_meshes/',
])
def test_package_paths_must_stay_in_their_share(uri):
    with pytest.raises(stage.StageError, match='leaves its root /opt/share/probe_meshes'):
        stage.mesh_path(uri, [], lambda p: f'/opt/share/{p}')


# Axol's package://assembly/ (no such package) and SO-101's relative assets/.
ROOTS = [('package://assembly/', '/opt/share/fm_description/axol_description'),
         ('assets/', '/opt/share/fm_description/so101_description/assets')]


@pytest.mark.parametrize('uri, path', [
    ('package://assembly/meshes/base.stl',
     '/opt/share/fm_description/axol_description/meshes/base.stl'),
    ('assets/base.stl', '/opt/share/fm_description/so101_description/assets/base.stl'),
    ('package://probe_meshes/m.stl', '/opt/share/probe_meshes/m.stl'),
    ('file:///abs/m.stl', '/abs/m.stl'),
    ('/abs/m.stl', '/abs/m.stl'),
])
def test_mesh_roots_come_before_packages(uri, path):
    assert stage.mesh_path(uri, ROOTS, lambda p: f'/opt/share/{p}') == path


@pytest.mark.parametrize('uri, match', [
    ('package://assembly/../../../etc/passwd', 'leaves its root /opt/share/fm_description/axol'),
    ('assets/../../x.stl', 'leaves its root /opt/share/fm_description/so101'),
    ('meshes/arm.stl', 'no mesh root'),
])
def test_mesh_roots_confine_and_relative_paths_need_one(uri, match):
    with pytest.raises(stage.StageError, match=match):
        stage.mesh_path(uri, ROOTS, lambda p: f'/opt/share/{p}')


@pytest.mark.parametrize('roots', [
    {'a': '/b'}, [['a']], [['a', 'relative/dir']], [['', '/b']], [['a', 3]],
])
def test_mesh_roots_must_be_prefix_absolute_directory_pairs(roots):
    with pytest.raises(stage.StageError, match='mesh_roots'):
        stage.parse_mesh_roots(roots)


def test_object_workspace_covers_the_spawn_ranges_with_a_margin():
    workspace = stage.object_workspace(CAN_ON_TABLE)
    margin = stage.WORKSPACE_MARGIN_M
    # The table spans x 0.5 (no ranges); the can's x reaches 0.38 - 0.03.
    assert workspace['min'][0] == pytest.approx(0.35 - margin)
    assert workspace['max'][0] == pytest.approx(0.6 + margin)
    for name, (xyz, _) in sc.spawn_poses(CAN_ON_TABLE, 11).items():
        assert all(lo <= v <= hi for v, lo, hi in zip(xyz, workspace['min'], workspace['max']))
    assert stage.generate(CAN_ON_TABLE, 0)['object_workspace'] == workspace


def test_package_uris_resolve_to_share_directories():
    text = stage.resolve_meshes(PROBE_URDF, [], lambda package: f'/opt/share/{package}/')
    assert 'filename="/opt/share/probe_meshes/meshes/arm.stl"' in text
    assert 'package://' not in text
    with pytest.raises(stage.StageError, match='not a package name'):
        stage.mesh_path('package://../x/m.stl', [], lambda p: '/s')


def test_bridge_topics_come_from_the_description():
    assert stage.bridge_topics(_robot()) == TOPICS


@pytest.mark.parametrize('change, match', [
    (lambda t: t.replace('TopicBasedSystem', 'MujocoSystem', 1), 'not a TopicBasedSystem'),
    (lambda t: t.replace('/isaac_joint_states', '/other', 1), 'disagree'),
])
def test_bridge_topics_refuse_a_mismatched_description(change, match):
    with pytest.raises(stage.StageError, match=match):
        stage.bridge_topics(ET.fromstring(change(PROBE_URDF)))


def test_camera_on_a_moving_link_mounts_on_that_link():
    link, (xyz, quat) = stage.camera_mount(_robot(), 'cam')
    assert link == 'arm'
    # 0.1 up, then 0.05 along the mount's x, which the 90 deg yaw turns onto y.
    assert _close(xyz, (0.0, 0.05, 0.1))
    assert _close(quat, (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)))


def test_camera_fixed_to_the_root_mounts_on_the_world():
    link, (xyz, _) = stage.camera_mount(_robot(), 'head_cam')
    assert link is None
    assert _close(xyz, (0.0, 0.0, 0.5))


def test_camera_frame_must_be_a_link():
    with pytest.raises(stage.StageError, match='not a link'):
        stage.camera_mount(_robot(), 'nowhere')


def test_usd_camera_looks_along_the_optical_axis():
    (camera, _) = stage.camera_specs(_robot(), '/probe', stage.check_cameras(CAMERAS))
    assert camera['role'] == 'chest'
    assert camera['parent_prim'] == stage.CAMERAS_PRIM
    # USD cameras look down -z: rotated into the stage, that must be the optical +z.
    q = camera['orient_wxyz']
    assert _close(stage.quat_rotate(q, (0.0, 0.0, -1.0)), (0.0, 0.0, 1.0))
    # USD up (+y) is the optical frame's up, -y.
    assert _close(stage.quat_rotate(q, (0.0, 1.0, 0.0)), (0.0, -1.0, 0.0))


def test_camera_specs_place_each_camera_and_its_rate():
    head, wrist = stage.camera_specs(_robot(), '/probe', stage.check_cameras(CAMERAS))
    assert wrist['prim_path'] == '/probe/arm/fm_cam_wrist_right'
    assert wrist['image_topic'] == '/cam_wrist_right/image_raw'
    assert (wrist['frame_skip_count'], wrist['rate_hz']) == (1, 30.0)
    assert (head['frame_skip_count'], head['rate_hz']) == (5, 10.0)


def test_usd_lens_reproduces_the_intrinsics():
    lens = stage.usd_lens(INTRINSICS)
    pitch_u = lens['horizontalAperture'] / INTRINSICS['width']
    pitch_v = lens['verticalAperture'] / INTRINSICS['height']
    assert lens['focalLength'] / pitch_u == pytest.approx(INTRINSICS['fx'])
    assert lens['focalLength'] / pitch_v == pytest.approx(INTRINSICS['fy'])
    assert INTRINSICS['width'] / 2 - lens['horizontalApertureOffset'] / pitch_u == (
        pytest.approx(INTRINSICS['cx'])
    )
    assert INTRINSICS['height'] / 2 + lens['verticalApertureOffset'] / pitch_v == (
        pytest.approx(INTRINSICS['cy'])
    )


@pytest.mark.parametrize('change, match', [
    (lambda c: c['cameras'].update({'../x': c['cameras']['chest']}), 'role'),
    (lambda c: c['cameras']['chest'].update(frame='a/b'), 'frame name'),
    (lambda c: c['cameras']['chest'].update(rate_hz=0), 'rate_hz'),
    (lambda c: c['cameras']['chest'].update(rate_hz=25), 'divide the 60 Hz'),
    (lambda c: c['cameras']['chest'].update(rate_hz=120), 'divide the 60 Hz'),
    (lambda c: c['cameras']['chest']['intrinsics'].update(width=3840, height=2160),
     'allow_large_image'),
    (lambda c: c['cameras']['chest']['intrinsics'].update(width=640.5), 'width'),
    (lambda c: c['cameras']['chest']['intrinsics'].update(fx=float('nan')), 'fx'),
    (lambda c: c['cameras']['chest']['intrinsics'].update(distortion_model='rational'),
     'unknown distortion_model'),
    (lambda c: c['cameras']['chest']['intrinsics'].update(distortion=[0.0]), 'takes 5'),
])
def test_bad_cameras_are_refused(change, match):
    cameras = copy.deepcopy(CAMERAS)
    change(cameras)
    with pytest.raises(stage.StageError, match=match):
        stage.check_cameras(cameras)


def test_a_large_camera_renders_when_it_opts_in():
    cameras = copy.deepcopy(CAMERAS)
    # deepcopy keeps the two cameras sharing one intrinsics dict; give chest its own.
    cameras['cameras']['chest']['intrinsics'] = dict(INTRINSICS, width=3840, height=2160)
    cameras['cameras']['chest']['allow_large_image'] = True
    assert stage.check_cameras(cameras)['chest']['intrinsics']['width'] == 3840


def test_the_shipped_cameras_fit_the_caps():
    with open(CAMERAS_YAML) as cameras_file:
        assert set(stage.check_cameras(yaml.safe_load(cameras_file))) == {'chest', 'wrist_right'}


def _graph(cameras=CAMERAS, scenario=CAN_ON_TABLE):
    camera_list = stage.camera_specs(_robot(), '/probe', stage.check_cameras(cameras))
    objects = stage.object_specs(scenario, 0)
    return stage.bridge_graph(*TOPICS, camera_list, objects)


def test_graph_wires_only_nodes_it_creates():
    graph = _graph()
    names = [name for name, _ in graph['nodes']]
    assert len(names) == len(set(names))
    for node_type in (t for _, t in graph['nodes']):
        assert node_type in stage.NODE_TYPES.values()
    attributes = [a for a, _ in graph['values'] + graph['prim_values']]
    attributes += [end for connection in graph['connections'] for end in connection]
    for attribute in attributes:
        assert attribute.split('.')[0] in names, attribute
    # Every node runs on the tick, except the context and the time reader it feeds.
    executed = {target.split('.')[0] for _, target in graph['connections']
                if target.endswith('inputs:execIn')}
    assert executed == set(names) - {'tick', 'context', 'sim_time'}


def test_graph_carries_the_contract_topics():
    values = dict(_graph()['values'])
    prims = dict(_graph()['prim_values'])
    assert values['clock.inputs:topicName'] == '/clock'
    assert values['joint_state.inputs:topicName'] == '/isaac_joint_states'
    assert values['joint_command.inputs:topicName'] == '/isaac_joint_commands'
    assert values['camera_wrist_right.inputs:topicName'] == '/cam_wrist_right/image_raw'
    assert values['camera_wrist_right.inputs:frameId'] == 'cam'
    assert values['camera_wrist_right.inputs:type'] == 'rgb'
    assert values['render_wrist_right.inputs:width'] == 640
    assert prims['render_wrist_right.inputs:cameraPrim'] == ['/probe/arm/fm_cam_wrist_right']
    assert prims['joint_state.inputs:targetPrim'] == [stage.ARTICULATION_ROOT]
    assert values['transforms.inputs:topicName'] == stage.OBJECT_POSES_TOPIC
    assert prims['transforms.inputs:targetPrims'] == [
        '/World/scenario/can', '/World/scenario/ball',
    ]


def test_graph_without_free_objects_or_cameras_publishes_no_poses():
    scenario = copy.deepcopy(CAN_ON_TABLE)
    scenario['objects'] = [o for o in scenario['objects'] if sc.is_static(o)]
    del scenario['success'], scenario['regions']
    names = [name for name, _ in _graph({'cameras': {}}, scenario)['nodes']]
    assert 'transforms' not in names
    assert not [n for n in names if n.startswith(('render_', 'camera_'))]


def test_drives_are_per_degree_for_revolute_joints():
    angular = stage.drives()['angular']
    assert angular['stiffness'] == pytest.approx(stage.DRIVE_STIFFNESS * math.pi / 180)


def _run_dir(tmp_path):
    return tempfile.mkdtemp(prefix=stage.WORLD_DIR_PREFIX, dir=str(tmp_path))


def test_write_stage_refuses_a_world_outside_a_run_directory(tmp_path):
    world = stage.write('/unused.xml', [0, 0, 0], CAN_ON_TABLE, 3, str(tmp_path))
    with pytest.raises(stage.StageError, match='run directory'):
        stage.write_stage(world, PROBE_URDF, {}, [], lambda p: f'/share/{p}')
    assert not (tmp_path / 'stage.json').exists()


def test_write_stage_writes_urdf_and_spec(tmp_path):
    tmp_path = pathlib.Path(_run_dir(tmp_path))
    world = stage.write('/unused.xml', [0, 0, 0], CAN_ON_TABLE, 3, str(tmp_path))
    path = stage.write_stage(
        world, PROBE_URDF, stage.check_cameras(CAMERAS), [], lambda p: f'/share/{p}'
    )
    assert path == str(tmp_path / 'stage.json')
    with open(path) as spec_file:
        spec = json.load(spec_file)
    assert spec['schema_version'] == stage.STAGE_SCHEMA_VERSION
    assert (spec['scenario'], spec['seed']) == ('can_on_table', 3)
    assert spec['robot']['urdf_path'] == str(tmp_path / 'robot.urdf')
    assert spec['robot']['prim_path'] == '/probe'
    assert spec['robot']['camera_links'] == ['/probe/arm']
    assert spec['robot']['import_config']['merge_fixed_joints'] is True
    assert spec['object_commands_topic'] == stage.OBJECT_COMMANDS_TOPIC
    assert spec['object_workspace'] == stage.object_workspace(CAN_ON_TABLE)
    assert [c['role'] for c in spec['cameras']] == ['chest', 'wrist_right']
    with open(spec['robot']['urdf_path']) as urdf:
        assert '/share/probe_meshes/meshes/arm.stl' in urdf.read()


def test_stage_spec_refuses_another_schema_version():
    world = stage.generate(CAN_ON_TABLE, 0)
    world['schema_version'] = 2
    with pytest.raises(stage.StageError, match='schema_version'):
        stage.stage_spec(world, PROBE_URDF, '/r.urdf', {})


def _openarm_description():
    xacro = pytest.importorskip('xacro')
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        share = get_package_share_directory('fm_control')
    except PackageNotFoundError:
        pytest.skip('fm_control (fm-robot) is not installed')
    return xacro.process_file(
        os.path.join(share, 'urdf', 'openarm.sim.urdf.xacro'),
        mappings={
            'robot_preset': 'default_bimanual', 'emit_grasp_frame': 'true',
            'sim_backend': 'isaac',
        },
    ).toxml()


def test_openarm_stage_matches_its_description_and_cameras():
    with open(CAMERAS_YAML) as cameras_file:
        cameras = stage.check_cameras(yaml.safe_load(cameras_file))
    spec = stage.stage_spec(
        stage.generate(CAN_ON_TABLE, 0), _openarm_description(), '/r.urdf', cameras
    )
    by_role = {c['role']: c for c in spec['cameras']}
    assert set(by_role) == {'chest', 'wrist_right'}
    assert by_role['chest']['parent_prim'] == stage.CAMERAS_PRIM
    # The importer puts the robot at /<robot name>; links sit flat under it.
    robot_prim = spec['robot']['prim_path']
    assert robot_prim == '/openarm_v20'
    # The wrist camera rides the link joint7 moves; the loader hand's fixed
    # ee_base_link and hand merge into it.
    assert by_role['wrist_right']['parent_prim'] == f'{robot_prim}/openarm_right_link7'
    assert by_role['chest']['intrinsics']['fx'] == 740.244
    assert by_role['chest']['frame_id'] == 'openarm_cam_chest_optical'
    # The chest camera is about 56 cm up and 9 cm forward, looking forward and down.
    x, _, z = by_role['chest']['translate']
    assert 0.05 < x < 0.12 and 0.5 < z < 0.6
    forward = stage.quat_rotate(by_role['chest']['orient_wxyz'], (0.0, 0.0, -1.0))
    assert forward[0] > 0 and forward[2] < 0
    values = dict(spec['graph']['values'])
    assert values['joint_state.inputs:topicName'] == '/isaac_joint_states'
    assert values['camera_chest.inputs:topicName'] == '/cam_chest/image_raw'
