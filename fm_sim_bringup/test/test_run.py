"""The rules run.launch.py applies to the four catalog entries."""

import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from description_model import fk, needs_openarm
from fm_sim_bringup import catalog, run
from fm_sim_cameras import cameras_file
from fm_sim_models.models import mjcf_path
from fm_sim_scenarios import gazebo_world, isaac_stage, scenario as sc
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


@needs_openarm
def test_openarm_servo_role_starts_fm_control_servo_under_the_loader_names():
    robot = run.resolve('mujoco', 'openarm', 'empty', 'idle')['robot']
    active, _ = run.controller_sets(robot, _controller('servo'))
    assert active == ['joint_state_broadcaster', 'follower_l_jtc', 'follower_r_jtc']
    (servo,) = run.role_launches(robot, _controller('servo'))
    assert servo['launch'] == {
        'package': 'fm_control', 'path': 'launch/openarm_sim_servo.launch.py',
    }
    assert servo['launch_args'] == {
        'sim_only': 'true', 'command_type': 'joint_jog', 'names': 'loader',
    }
    assert os.path.isfile(catalog.share_file(servo['launch']))


def test_resolve_refuses_an_unknown_backend():
    with pytest.raises(catalog.CatalogError, match="unknown backend 'bullet'"):
        run.resolve('bullet', 'so101', 'empty', 'idle')


@pytest.mark.parametrize('robot,controllers', [
    pytest.param('openarm', 4, marks=needs_openarm), ('so101', 2), ('axol', 2),
])
def test_idle_loads_every_controller_once(robot, controllers):
    entries = run.resolve('mujoco', robot, 'empty', 'idle')
    active, inactive = run.controller_sets(entries['robot'], entries['controller'])
    assert active == ['joint_state_broadcaster']
    assert len(inactive) == len(set(inactive)) == controllers


@pytest.mark.parametrize('robot,arm_controllers', [
    pytest.param('openarm', ['follower_l_jtc', 'follower_r_jtc'], marks=needs_openarm),
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
    assert catalog.load('robots', 'openarm')['robot_profile'] == 'loader_openarm_v2'


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
        run.resolve('gazebo', 'so101', 'empty', 'idle')


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


def _entries(backend, scenario, robot='so101'):
    return run.resolve(backend, robot, scenario, 'idle')


def test_backend_without_generator_runs_the_robot_model_for_empty(monkeypatch):
    monkeypatch.delitem(run.WORLD_GENERATORS, 'gazebo')
    entries = _entries('gazebo', 'empty')
    world = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 0, '/robot.xml', '/unused'
    )
    assert world is None


def test_backend_without_generator_refuses_objects(monkeypatch):
    monkeypatch.delitem(run.WORLD_GENERATORS, 'gazebo')
    entries = _entries('gazebo', 'can_on_table')
    with pytest.raises(catalog.CatalogError, match="backend 'gazebo' has no scenario generator"):
        run.world_model(
            entries['backend'], entries['robot'], entries['scenario'], 0, '/robot.xml', '/unused'
        )


def test_gazebo_writes_the_scenario_world(tmp_path):
    entries = _entries('gazebo', 'can_on_table')
    world = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 3, '/robot.xml',
        str(tmp_path),
    )
    assert world == str(tmp_path / 'can_on_table_3.sdf')
    with open(world) as world_file:
        assert gazebo_world.free_objects(world_file.read()) == ['can']


@needs_openarm
def test_mujoco_builds_the_openarm_world(tmp_path):
    entries = _entries('mujoco', 'can_on_table', 'openarm')
    robot_mjcf = mjcf_path(entries['robot']['mjcf'])
    world = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 3, robot_mjcf,
        str(tmp_path),
    )
    assert world == str(tmp_path / 'can_on_table_3.xml')
    with open(world) as world_file:
        text = world_file.read()
    assert f'file="{robot_mjcf}"' in text
    # The model's origin is the description's root, so the mount is the default.
    assert '<frame name="robot_mount" pos="0 0 0">' in text
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


def test_run_values_win_over_controller_launch_args():
    arguments = run.controller_launch_arguments(
        # The loader refuses use_sim here; this is the backstop for an entry it never saw.
        {'launch_args': {'perception': 'model', 'use_sim': 'false'}},
        {'robot_profile': 'loader_openarm_v2'}, 'can_on_table', 'true',
    )
    assert arguments == {
        'perception': 'model',
        'use_sim': 'true',
        'robot_profile': 'loader_openarm_v2',
        'scenario': 'can_on_table',
    }


def test_controller_launch_args_are_optional():
    arguments = run.controller_launch_arguments(
        {}, {'robot_profile': 'loader_openarm_v2'}, 'empty', 'true'
    )
    assert set(arguments) == {'use_sim', 'robot_profile', 'scenario'}


def test_mujoco_gets_only_its_own_launch_arguments():
    entries = _entries('mujoco', 'can_on_table')
    arguments = run.backend_launch_arguments(
        entries['backend'], entries['robot'], '<robot/>', '/c.yaml', '/w/world.xml'
    )
    assert arguments == {
        **entries['backend'].get('launch_args', {}),
        'robot_description': '<robot/>',
        'controllers_file': '/c.yaml',
    }


@pytest.mark.parametrize('backend', ['gazebo', 'isaac'])
def test_stage_backends_get_the_world_and_the_robot_cameras(backend):
    entries = _entries(backend, 'can_on_table')
    cameras = catalog.load('robots', 'openarm')['cameras_file']
    robot = dict(entries['robot'], cameras_file=cameras)
    arguments = run.backend_launch_arguments(
        entries['backend'], robot, '<robot/>', '/c.yaml', '/w/stage.isaac.json'
    )
    assert arguments['world'] == '/w/stage.isaac.json'
    assert arguments['cameras_file'] == catalog.share_file(cameras)
    assert json.loads(arguments['mesh_roots']) == run.mesh_roots(robot)
    assert arguments['robot'] == 'so101'
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


@needs_openarm
def test_openarm_cameras_mount_on_the_description_camera_links(tmp_path):
    entries = _entries('mujoco', 'can_on_table', 'openarm')
    world = run.world_file(entries['backend'], entries['scenario'], 0, str(tmp_path))
    robot_mjcf = mjcf_path(entries['robot']['mjcf'])
    cameras = run.sim_cameras(
        entries['backend'], entries['robot'], _openarm_description(entries, world), robot_mjcf,
    )
    by_role = {camera.role: camera for camera in cameras}
    assert sorted(by_role) == ['chest', 'wrist_right']
    assert by_role['chest'].model.body == 'chest_cam'
    assert by_role['wrist_right'].model.body == 'right_wrist_cam'
    assert by_role['chest'].render_topic == '/mujoco_cameras/cam_chest/color'
    parameters = run.camera_node_parameters(by_role['wrist_right'])
    assert parameters == {
        'role': 'wrist_right',
        'cameras_file': catalog.share_file(entries['robot']['cameras_file']),
        'render_topic': '/mujoco_cameras/cam_wrist_right/color',
        'use_sim_time': True,
    }
    path = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 0, robot_mjcf,
        str(tmp_path), cameras,
    )
    assert path == world


@needs_openarm
def test_openarm_camera_refused_off_a_shared_body(monkeypatch, tmp_path):
    entries = _entries('mujoco', 'empty', 'openarm')
    description = _openarm_description(entries, '/unused.xml')
    monkeypatch.setattr(run.mujoco_world, 'body_names', lambda robot_mjcf: set())
    with pytest.raises(catalog.CatalogError, match='moves with joint'):
        run.sim_cameras(entries['backend'], entries['robot'], description, '/unused.xml')


@needs_openarm
def test_openarm_mujoco_cameras_sit_on_the_description_frames(tmp_path):
    # The real wiring: description -> mounts -> generated world -> MuJoCo, compared
    # with the description's own forward kinematics at an arbitrary arm pose.
    mujoco = pytest.importorskip('mujoco', reason='mujoco Python binding (requirements-sim.txt)')
    entries = _entries('mujoco', 'can_on_table', 'openarm')
    world = run.world_file(entries['backend'], entries['scenario'], 4, str(tmp_path))
    description = _openarm_description(entries, world)
    robot_mjcf = mjcf_path(entries['robot']['mjcf'])
    cameras = run.sim_cameras(entries['backend'], entries['robot'], description, robot_mjcf)
    run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 4, robot_mjcf,
        str(tmp_path), cameras,
    )
    model = mujoco.MjModel.from_xml_path(world)
    data = mujoco.MjData(model)
    rng = np.random.default_rng(8)
    positions = {}
    for index in range(1, 8):
        name = f'follower_r_joint{index}'
        low, high = model.joint(name).range
        positions[name] = float(rng.uniform(low, high))
        data.qpos[model.joint(name).qposadr[0]] = positions[name]
    mujoco.mj_forward(model, data)
    specs = cameras_file.load(catalog.share_file(entries['robot']['cameras_file']))
    for camera in cameras:
        expected = fk(ET.fromstring(description), specs[camera.role]['frame'], positions)
        rotation = data.cam(camera.model.name).xmat.reshape(3, 3)
        # The compiled model is written to six significant digits, hence the tolerance.
        assert data.cam(camera.model.name).xpos == pytest.approx(expected[:3, 3], abs=1e-5)
        # MuJoCo's camera x is optical x; its y and z are optical -y and -z.
        assert rotation @ np.diag([1.0, -1.0, -1.0]) == pytest.approx(expected[:3, :3], abs=1e-5)
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
    entries = _entries('mujoco', 'empty', 'openarm')
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
            entries['backend'], entries['robot'], description,
            mjcf_path(entries['robot']['mjcf']),
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


COMBINATION_CASES = json.loads(
    (Path(__file__).parent / 'combination_cases.json').read_text()
)['cases']


@pytest.mark.parametrize('case', COMBINATION_CASES, ids=lambda case: case['name'])
def test_resolve_matches_the_shared_combination_cases(monkeypatch, case):
    """The sim agent refuses from the same table; the two must agree."""
    entries = {
        'backends': {'name': case['backend']},
        'robots': {'name': 'robot', **case['robot']},
        'scenarios': catalog.load('scenarios', case.get('scenario', 'empty')),
        'controllers': {'name': 'controller', **case['controller']},
    }
    monkeypatch.setattr(catalog, 'require_available', lambda kind, name: entries[kind])
    try:
        resolved = run.resolve(case['backend'], 'robot', case.get('scenario', 'empty'), 'controller')
        run.controller_sets(resolved['robot'], resolved['controller'])
    except catalog.CatalogError as error:
        refusal = str(error)
    else:
        refusal = None
    assert (refusal is not None) == case['refused']
    assert case.get('reason', '') in (refusal or '')
