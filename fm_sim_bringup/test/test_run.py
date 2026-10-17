"""The rules run.launch.py applies to the four catalog entries."""

import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from fm_sim_bringup import catalog, run
from fm_sim_cameras import cameras_file
from fm_sim_models.models import mjcf_path
from fm_sim_scenarios import isaac_stage, scenario as sc
import numpy as np
import pytest
import yaml

ROBOT = {
    'name': 'arm',
    'description_args': {'robot_preset': 'right_arm'},
    'controller_roles': {
        'broadcasters': ['joint_state_broadcaster'],
        'arm_trajectory': ['arm_controller'],
        'arm_forward_position': ['forward_position_controller'],
    },
    'launch_roles': {
        'servo': {
            'requires': ['arm_trajectory'],
            'launch': {'package': 'servo_pkg', 'path': 'launch/servo.launch.py'},
        },
    },
}


def _controller(*roles):
    return {'name': 'test', 'activate': list(roles)}


def test_backend_args_fill_placeholders_over_robot_args():
    backend = {'description_args': {
        'sim_backend': 'mujoco',
        'mujoco_model': '{mjcf_path}',
        'gazebo_controllers_file': '{controllers_file}',
    }}
    assert run.description_args(backend, ROBOT, '/m.xml', '/c.yaml') == {
        'robot_preset': 'right_arm',
        'sim_backend': 'mujoco',
        'mujoco_model': '/m.xml',
        'gazebo_controllers_file': '/c.yaml',
    }


def test_idle_activates_broadcasters_only():
    active, inactive = run.controller_sets(ROBOT, _controller())
    assert active == ['joint_state_broadcaster']
    assert inactive == ['arm_controller', 'forward_position_controller']


def test_controller_activates_its_roles():
    active, inactive = run.controller_sets(ROBOT, _controller('arm_forward_position'))
    assert active == ['joint_state_broadcaster', 'forward_position_controller']
    assert inactive == ['arm_controller']


def test_unknown_role_names_the_robot_roles():
    with pytest.raises(catalog.CatalogError, match='gripper.*arm_forward_position'):
        run.controller_sets(ROBOT, _controller('gripper'))


def test_unknown_role_names_the_launch_roles_too():
    with pytest.raises(catalog.CatalogError, match='its roles: .*servo'):
        run.controller_sets(ROBOT, _controller('teleop'))
    with pytest.raises(catalog.CatalogError, match='activates teleop'):
        run.role_launches(ROBOT, _controller('teleop'))


def test_launch_role_activates_what_it_requires():
    active, inactive = run.controller_sets(ROBOT, _controller('servo'))
    assert active == ['joint_state_broadcaster', 'arm_controller']
    assert inactive == ['forward_position_controller']
    assert run.role_launches(ROBOT, _controller('servo')) == [ROBOT['launch_roles']['servo']]


def test_launch_role_and_its_requires_activate_once():
    active, _ = run.controller_sets(ROBOT, _controller('arm_trajectory', 'servo'))
    assert active == ['joint_state_broadcaster', 'arm_controller']


def test_launch_role_listed_twice_starts_once():
    assert len(run.role_launches(ROBOT, _controller('servo', 'servo'))) == 1


def test_controller_roles_start_no_launch():
    assert run.role_launches(ROBOT, _controller('arm_trajectory')) == []


def test_launch_role_requiring_a_missing_role_fails():
    robot = dict(ROBOT, launch_roles={'servo': dict(
        ROBOT['launch_roles']['servo'], requires=['gripper'],
    )})
    with pytest.raises(catalog.CatalogError, match="'servo' requires gripper"):
        run.controller_sets(robot, _controller('servo'))


def test_role_named_twice_fails():
    robot = dict(ROBOT, launch_roles={'arm_trajectory': ROBOT['launch_roles']['servo']})
    with pytest.raises(catalog.CatalogError, match='both a controller role and a launch'):
        run.controller_sets(robot, _controller())


def test_openarm_servo_role_starts_fm_control_servo():
    robot = run.resolve('mujoco', 'openarm', 'empty', 'idle')['robot']
    active, _ = run.controller_sets(robot, _controller('servo'))
    assert active == [
        'joint_state_broadcaster', 'openarm_left_arm_controller', 'openarm_right_arm_controller',
    ]
    (servo,) = run.role_launches(robot, _controller('servo'))
    assert servo['launch'] == {
        'package': 'fm_control', 'path': 'launch/openarm_sim_servo.launch.py',
    }
    assert servo['launch_args']['sim_only'] == 'true'
    assert os.path.isfile(catalog.share_file(servo['launch']))


def test_resolve_refuses_an_unknown_backend():
    with pytest.raises(catalog.CatalogError, match="unknown backend 'bullet'"):
        run.resolve('bullet', 'openarm', 'empty', 'idle')


@pytest.mark.parametrize('robot,controllers', [('openarm', 6), ('so101', 2), ('axol', 2)])
def test_idle_loads_every_controller_once(robot, controllers):
    entries = run.resolve('mujoco', robot, 'empty', 'idle')
    active, inactive = run.controller_sets(entries['robot'], entries['controller'])
    assert active == ['joint_state_broadcaster']
    assert len(inactive) == len(set(inactive)) == controllers


@pytest.mark.parametrize('robot,arm_controllers', [
    ('openarm', ['openarm_left_arm_controller', 'openarm_right_arm_controller']),
    ('so101', ['so101_arm_controller']),
    ('axol', ['axol_left_arm_controller', 'axol_right_arm_controller']),
])
def test_teleop_activates_the_arm_trajectory_controllers(robot, arm_controllers):
    entries = run.resolve('mujoco', robot, 'empty', 'teleop')
    active, inactive = run.controller_sets(entries['robot'], entries['controller'])
    assert active == ['joint_state_broadcaster', *arm_controllers]
    assert not set(active) & set(inactive)


def test_resolve_refuses_a_launch_controller_on_a_robot_without_profile(monkeypatch):
    load = catalog.require_available

    def idle_with_a_launch(kind, name):
        entry = load(kind, name)
        if kind == 'controllers':
            entry['launch'] = {'package': 'fake_pkg', 'path': 'launch/run.launch.py'}
        return entry

    monkeypatch.setattr(catalog, 'require_available', idle_with_a_launch)
    with pytest.raises(
        catalog.CatalogError, match="'idle' needs a robot profile; robot 'so101' has none"
    ):
        run.resolve('mujoco', 'so101', 'empty', 'idle')
    assert run.resolve('mujoco', 'openarm', 'empty', 'idle')['robot']['robot_profile']


def test_resolve_refuses_a_backend_the_robot_does_not_support(monkeypatch):
    load = catalog.require_available

    def mujoco_only_robot(kind, name):
        entry = load(kind, name)
        if kind == 'robots':
            entry['supported_backends'] = ['mujoco']
        return entry

    monkeypatch.setattr(catalog, 'require_available', mujoco_only_robot)
    with pytest.raises(
        catalog.CatalogError, match="not supported on backend 'gazebo' yet; supported: mujoco$"
    ):
        run.resolve('gazebo', 'openarm', 'empty', 'idle')


@pytest.mark.parametrize('text, seed', [('0', 0), ('42', 42), ('4294967295', 2 ** 32 - 1)])
def test_seed_parses(text, seed):
    assert run.parse_seed(text) == seed


@pytest.mark.parametrize('text', ['', '-1', '4294967296', '1.5', 'x'])
def test_seed_refuses_non_uint32(text):
    with pytest.raises(catalog.CatalogError, match='seed must be an integer'):
        run.parse_seed(text)


def test_timeout_empty_keeps_the_scenario_one():
    assert run.parse_timeout('') == 0.0
    assert run.parse_timeout('12.5') == 12.5


@pytest.mark.parametrize('text', ['0', '-3', 'x', 'nan', 'inf', '1e400'])
def test_timeout_refuses_non_positive(text):
    with pytest.raises(catalog.CatalogError, match='timeout_s'):
        run.parse_timeout(text)


def _entries(backend, scenario):
    return run.resolve(backend, 'openarm', scenario, 'idle')


def test_backend_without_generator_runs_the_robot_model_for_empty():
    entries = _entries('gazebo', 'empty')
    world = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 0, '/robot.xml', '/unused'
    )
    assert world is None


def test_backend_without_generator_refuses_objects():
    entries = _entries('gazebo', 'can_on_table')
    with pytest.raises(catalog.CatalogError, match="backend 'gazebo' has no scenario generator"):
        run.world_model(
            entries['backend'], entries['robot'], entries['scenario'], 0, '/robot.xml', '/unused'
        )


@pytest.mark.skipif(
    not os.path.isfile(mjcf_path('openarm')), reason='openarm_mujoco external (vcs import)'
)
def test_mujoco_builds_the_openarm_world(tmp_path):
    entries = _entries('mujoco', 'can_on_table')
    world = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 3, mjcf_path('openarm'),
        str(tmp_path),
    )
    assert world == str(tmp_path / 'can_on_table_3.xml')
    with open(world) as world_file:
        text = world_file.read()
    assert f'file="{mjcf_path("openarm")}"' in text
    assert '<frame name="robot_mount" pos="0 0 0.698">' in text
    (x, y, z), _ = sc.spawn_poses(entries['scenario'], 3)['can']
    assert f'<body name="can" pos="{x:.9g} {y:.9g} {z:.9g}"' in text


def test_isaac_writes_the_scenario_half_of_its_stage(tmp_path):
    entries = _entries('isaac', 'can_on_table')
    world = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 3, '/robot.xml',
        str(tmp_path),
    )
    assert world == str(tmp_path / 'can_on_table_3.isaac.json')
    with open(world) as world_file:
        spec = json.load(world_file)
    assert spec['seed'] == 3
    (can,) = [o for o in spec['objects'] if o['name'] == 'can']
    assert can['translate'] == list(sc.spawn_poses(entries['scenario'], 3)['can'][0])


@pytest.mark.parametrize('backend', ['mujoco', 'gazebo'])
def test_simulators_get_only_their_own_launch_arguments(backend):
    entries = _entries(backend, 'can_on_table' if backend == 'mujoco' else 'empty')
    arguments = run.backend_launch_arguments(
        entries['backend'], entries['robot'], '<robot/>', '/c.yaml', '/w/world.xml'
    )
    assert arguments == {
        **entries['backend'].get('launch_args', {}),
        'robot_description': '<robot/>',
        'controllers_file': '/c.yaml',
    }


def test_isaac_gets_the_world_and_the_robot_cameras():
    entries = _entries('isaac', 'can_on_table')
    arguments = run.backend_launch_arguments(
        entries['backend'], entries['robot'], '<robot/>', '/c.yaml', '/w/stage.isaac.json'
    )
    assert arguments['world'] == '/w/stage.isaac.json'
    assert arguments['cameras_file'] == catalog.share_file(entries['robot']['cameras_file'])
    assert arguments['mesh_roots'] == '[]'
    assert run.backend_launch_arguments(
        entries['backend'], {'name': 'bare'}, '<robot/>', '/c.yaml', None
    )['world'] == ''


def test_mesh_roots_resolve_to_shares():
    assert run.mesh_roots(catalog.load('robots', 'openarm')) == []
    ((prefix, directory),) = run.mesh_roots(catalog.load('robots', 'axol'))
    assert prefix == 'package://assembly/'
    assert directory.endswith('/fm_description/axol_description')
    ((prefix, directory),) = run.mesh_roots(catalog.load('robots', 'so101'))
    assert prefix == 'assets/'
    assert directory.endswith('/fm_description/so101_description/assets')
    entries = _entries('isaac', 'empty')
    entries['robot'] = catalog.load('robots', 'axol')
    arguments = run.backend_launch_arguments(
        entries['backend'], entries['robot'], '<robot/>', '/c.yaml', None
    )
    assert json.loads(arguments['mesh_roots']) == run.mesh_roots(entries['robot'])


def test_openarm_cameras_file_is_shipped():
    robot = catalog.load('robots', 'openarm')
    assert os.path.isfile(catalog.share_file(robot['cameras_file']))


def test_scenario_node_runs_only_with_objects():
    empty = _entries('mujoco', 'empty')
    assert run.scenario_node_parameters(empty['backend'], empty['scenario'], 0, 0.0) is None
    entries = _entries('mujoco', 'can_on_table')
    parameters = run.scenario_node_parameters(entries['backend'], entries['scenario'], 7, 30.0)
    assert parameters['backend'] == 'mujoco'
    assert parameters['seed'] == 7
    assert parameters['timeout_s'] == 30.0
    assert parameters['use_sim_time'] is True
    assert json.loads(parameters['scenario']) == entries['scenario']


def test_backend_without_camera_renders_has_no_cameras():
    entries = _entries('gazebo', 'empty')
    assert run.sim_cameras(entries['backend'], entries['robot'], '<robot/>', '/r.xml') == []


def test_robot_without_cameras_file_has_no_cameras():
    entries = _entries('mujoco', 'empty')
    robot = {k: v for k, v in entries['robot'].items() if k != 'cameras_file'}
    assert run.sim_cameras(entries['backend'], robot, '<robot/>', '/r.xml') == []


def test_mujoco_camera_render_is_the_plugin_default_topic():
    # CameraPlugin publishes a camera's color image on <plugin key>/<camera>/color.
    with open(os.path.join(
        get_package_share_directory('fm_sim_backends'), 'config', 'mujoco_plugins.yaml'
    )) as plugins_file:
        plugins = yaml.safe_load(plugins_file)
    plugins = plugins['mujoco_ros2_control_node']['ros__parameters']['mujoco_plugins']
    (key,) = [k for k, v in plugins.items()
              if v['type'] == 'mujoco_ros2_control_plugins/CameraPlugin']
    assert run.RENDERED_CAMERAS['mujoco'][1] == f'/{key}/cam_{{role}}/color'
    # The bridge caps cameras at 10 Hz; rendering faster only burns the engine's CPU.
    assert plugins[key]['camera_publish_rate'] <= 10.0


def _openarm_description(entries, world):
    import xacro
    robot = entries['robot']
    return xacro.process_file(
        catalog.share_file(robot['description_file']),
        mappings=run.description_args(
            entries['backend'], robot, world, catalog.share_file(robot['controllers_file'])
        ),
    ).toxml()


needs_openarm = pytest.mark.skipif(
    not os.path.isfile(mjcf_path('openarm')), reason='openarm_mujoco external (vcs import)'
)


@needs_openarm
def test_openarm_cameras_mount_on_the_world_and_the_right_hand(tmp_path):
    entries = _entries('mujoco', 'can_on_table')
    world = run.world_file(entries['backend'], entries['scenario'], 0, str(tmp_path))
    cameras = run.sim_cameras(
        entries['backend'], entries['robot'], _openarm_description(entries, world),
        mjcf_path('openarm'),
    )
    by_role = {camera.role: camera for camera in cameras}
    assert sorted(by_role) == ['chest', 'wrist_right']
    assert by_role['chest'].model.body is None
    assert by_role['wrist_right'].model.body == 'openarm_right_ee_base_link'
    assert by_role['chest'].render_topic == '/mujoco_cameras/cam_chest/color'
    parameters = run.camera_node_parameters(by_role['wrist_right'])
    assert parameters == {
        'role': 'wrist_right',
        'cameras_file': catalog.share_file(entries['robot']['cameras_file']),
        'render_topic': '/mujoco_cameras/cam_wrist_right/color',
        'use_sim_time': True,
    }
    path = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 0, mjcf_path('openarm'),
        str(tmp_path), cameras,
    )
    assert path == world


@needs_openarm
def test_openarm_camera_refused_off_a_shared_body(monkeypatch, tmp_path):
    entries = _entries('mujoco', 'empty')
    description = _openarm_description(entries, '/unused.xml')
    monkeypatch.setattr(run.mujoco_world, 'body_names', lambda robot_mjcf: set())
    with pytest.raises(catalog.CatalogError, match='moves with joint'):
        run.sim_cameras(entries['backend'], entries['robot'], description, mjcf_path('openarm'))


def _fk(urdf, link, positions):
    """Return the pose of ``link`` in the URDF root at joint ``positions`` (revolute only)."""
    import xml.etree.ElementTree as ET
    from fm_sim_cameras import mount
    root = ET.fromstring(urdf)
    parent_joint = {j.find('child').get('link'): j for j in root.findall('joint')}
    transform = np.eye(4)
    while link in parent_joint:
        joint = parent_joint[link]
        step = mount._origin(joint)
        if joint.get('type') in ('revolute', 'continuous'):
            axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
            angle = positions.get(joint.get('name'), 0.0)
            k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            turn = np.eye(4)
            turn[:3, :3] = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k
            step = step @ turn
        elif joint.get('type') != 'fixed':
            raise AssertionError(f'unexpected joint type {joint.get("type")}')
        transform = step @ transform
        link = joint.find('parent').get('link')
    return transform


@needs_openarm
def test_openarm_mujoco_cameras_sit_on_the_description_frames(tmp_path):
    # The real wiring: description -> mounts -> generated world -> MuJoCo, compared
    # with the description's own forward kinematics at an arbitrary arm pose.
    mujoco = pytest.importorskip('mujoco', reason='mujoco Python binding (requirements-sim.txt)')
    entries = _entries('mujoco', 'can_on_table')
    world = run.world_file(entries['backend'], entries['scenario'], 4, str(tmp_path))
    description = _openarm_description(entries, world)
    cameras = run.sim_cameras(
        entries['backend'], entries['robot'], description, mjcf_path('openarm')
    )
    run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 4, mjcf_path('openarm'),
        str(tmp_path), cameras,
    )
    model = mujoco.MjModel.from_xml_path(world)
    data = mujoco.MjData(model)
    rng = np.random.default_rng(8)
    positions = {}
    for index in range(1, 8):
        name = f'openarm_right_joint{index}'
        low, high = model.joint(name).range
        positions[name] = float(rng.uniform(low, high))
        data.qpos[model.joint(name).qposadr[0]] = positions[name]
    mujoco.mj_forward(model, data)
    specs = cameras_file.load(catalog.share_file(entries['robot']['cameras_file']))
    for camera in cameras:
        expected = _fk(description, specs[camera.role]['frame'], positions)
        rotation = data.cam(camera.model.name).xmat.reshape(3, 3)
        assert data.cam(camera.model.name).xpos == pytest.approx(expected[:3, 3], abs=1e-6)
        # MuJoCo's camera x is optical x; its y and z are optical -y and -z.
        assert rotation @ np.diag([1.0, -1.0, -1.0]) == pytest.approx(expected[:3, :3], abs=1e-6)
        assert model.cam(camera.model.name).fovy[0] == pytest.approx(camera.model.fovy_deg)


def test_world_file_names_the_world_only_where_it_carries_cameras(tmp_path):
    mujoco = _entries('mujoco', 'can_on_table')
    assert run.world_file(mujoco['backend'], mujoco['scenario'], 2, str(tmp_path)) == str(
        tmp_path / 'can_on_table_2.xml'
    )
    isaac = _entries('isaac', 'can_on_table')
    assert run.world_file(isaac['backend'], isaac['scenario'], 2, str(tmp_path)) is None
    assert run.sim_cameras(isaac['backend'], isaac['robot'], '<robot/>', '/r.xml') == []


# --- One cameras file, one shared verdict, explicit backend refusals ----------

OPENARM_CAMERAS_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'openarm', 'cameras.yaml'
)


def _openarm_cameras(change=None):
    with open(OPENARM_CAMERAS_YAML) as openarm_file:
        data = yaml.safe_load(openarm_file)
    if change:
        change(data)
    return data


@pytest.fixture
def mujoco_verdict(monkeypatch, tmp_path):
    """Return MuJoCo's verdict on a cameras file: ``run.sim_cameras`` on the real OpenArm."""
    entries = _entries('mujoco', 'empty')
    description = _openarm_description(entries, '/unused.xml')
    share_file = catalog.share_file

    def verdict(data):
        path = tmp_path / 'cameras.yaml'
        path.write_text(yaml.safe_dump(data))
        monkeypatch.setattr(
            run.catalog, 'share_file',
            lambda ref: str(path) if ref == entries['robot']['cameras_file'] else share_file(ref),
        )
        cameras = run.sim_cameras(
            entries['backend'], entries['robot'], description, mjcf_path('openarm')
        )
        assert sorted(camera.role for camera in cameras) == sorted(data['cameras'])
        return True
    return verdict


def _isaac_verdict(data):
    isaac_stage.check_cameras(data)
    return True


@needs_openarm
def test_the_shipped_cameras_file_passes_both_backends(mujoco_verdict):
    data = _openarm_cameras()
    assert mujoco_verdict(data) and _isaac_verdict(data)


@pytest.mark.parametrize('change', [
    lambda d: d['cameras'].update(head=d['cameras']['chest']),
    lambda d: d['cameras']['chest'].update(rate_hz=-1),
    lambda d: d['cameras']['chest'].update(intrinsics=dict(
        d['cameras']['chest']['intrinsics'], width=3840, height=2160)),
])
@needs_openarm
def test_the_shared_loader_refuses_for_both_backends(mujoco_verdict, change):
    with pytest.raises(catalog.CatalogError):
        mujoco_verdict(_openarm_cameras(change))
    with pytest.raises(isaac_stage.StageError):
        _isaac_verdict(_openarm_cameras(change))


@needs_openarm
def test_plumb_bob_renders_on_isaac_but_not_through_the_mujoco_warp(mujoco_verdict):
    def plumb_bob(d):
        d['cameras']['chest']['intrinsics'] = dict(
            d['cameras']['chest']['intrinsics'], distortion_model='plumb_bob',
            distortion=[0.0, 0.0, 0.0, 0.0, 0.0],
        )
    data = _openarm_cameras(plumb_bob)
    assert _isaac_verdict(data)
    with pytest.raises(catalog.CatalogError, match='cannot be rendered by the fisheye warp'):
        mujoco_verdict(data)


@needs_openarm
def test_a_rate_isaac_cannot_divide_still_renders_on_mujoco(mujoco_verdict):
    def seven_hz(d):
        d['cameras']['chest']['rate_hz'] = 7
    data = _openarm_cameras(seven_hz)
    assert mujoco_verdict(data)
    with pytest.raises(isaac_stage.StageError, match='divide the 60 Hz'):
        _isaac_verdict(data)
