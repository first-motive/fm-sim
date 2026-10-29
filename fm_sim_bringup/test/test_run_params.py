"""Run params in run.launch.py: overrides merged into the entries, the params file and its guard.

Precedence is resolved through rcl (fm_sim_backends.params_check), from the
Node actions the launch builds, not read off their parameter lists.
"""

import importlib.util
import json
import os
import re

from ament_index_python.packages import get_package_share_directory
from fm_sim_backends.params_check import (
    assert_guard_holds, assert_params_file_wins, resolved_parameters,
)
from fm_sim_bringup import catalog, params, run
from fm_sim_models.models import mjcf_path
from fm_sim_scenarios import mujoco_world
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch_ros.actions import Node
import pytest
import yaml

CAN_MASS = {
    'label': 'Can mass', 'type': 'float', 'default': 0.35, 'min': 0.05, 'max': 2.0,
    'target': {'kind': 'entry', 'path': 'objects.can.mass'},
}
SPAWN_X = {
    'label': 'Can spawn x', 'type': 'array', 'items': 'float', 'length': 2,
    'default': [-0.03, 0.03], 'min': -0.1, 'max': 0.1,
    'target': {'kind': 'entry', 'path': 'objects.can.spawn.x'},
}
PLUGIN_RATE = {
    'label': 'Object pose rate', 'type': 'float', 'default': 30.0, 'min': 1.0, 'max': 100.0,
    'backends': ['mujoco'],
    'target': {'kind': 'ros_param', 'node': '/mujoco_ros2_control_node',
               'param': 'mujoco_plugins.object_poses.publish_rate'},
}


def _entries(backend='mujoco', **tunables):
    entries = run.resolve(backend, 'so101', 'can_on_table', 'idle')
    entries['scenario']['params'] = tunables
    return entries


def test_an_entry_override_changes_the_generated_world(tmp_path):
    entries = run.apply_overrides(_entries(can_mass=CAN_MASS), {'scenario.can_mass': 1})
    assert entries['scenario']['objects'][2]['mass'] == 1.0
    world = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 0,
        mjcf_path('so101'), str(tmp_path),
    )
    with open(world) as world_file:
        assert re.search(r'<geom name="can"[^>]* mass="1"', world_file.read())


def test_overrides_leave_the_resolved_entries_alone():
    entries = _entries(can_mass=CAN_MASS)
    run.apply_overrides(entries, {'scenario.can_mass': 1.0})
    assert entries['scenario']['objects'][2]['mass'] == 0.35


def test_an_override_the_entry_would_refuse_is_refused():
    with pytest.raises(catalog.CatalogError, match='spawn'):
        run.apply_overrides(_entries(spawn_x=SPAWN_X), {'scenario.spawn_x': [0.05, -0.05]})


@pytest.mark.parametrize('overrides, match', [
    ({'scenario.nope': 1.0}, 'not a declared tunable'),
    ({'planner.can_mass': 1.0}, 'not a declared tunable'),
    ({'scenario.can_mass': 9.0}, 'must be between'),
    ({'scenario.can_mass': True}, 'must be a number'),
])
def test_apply_overrides_refuses(overrides, match):
    with pytest.raises(catalog.CatalogError, match=match):
        run.apply_overrides(_entries(can_mass=CAN_MASS), overrides)


def test_a_ros_param_override_is_checked_and_left_to_the_params_file():
    entries = _entries(rate=PLUGIN_RATE)
    assert run.apply_overrides(entries, {'scenario.rate': 10.0}) == entries
    with pytest.raises(catalog.CatalogError, match='applies only on mujoco'):
        run.apply_overrides(_entries('gazebo', rate=PLUGIN_RATE), {'scenario.rate': 10.0})


@pytest.mark.parametrize('content', ['[1, 2]', 'not json'])
def test_a_malformed_overrides_file_is_refused(tmp_path, content):
    path = tmp_path / 'params.json'
    path.write_text(content)
    with pytest.raises(catalog.CatalogError, match='overrides_file'):
        run.load_overrides(str(path))


# -- the launch's own nodes --------------------------------------------------


def _launch_setup(monkeypatch, tmp_path, params_file):
    """Run run.launch.py's setup for so101 x mujoco x can_on_table x idle."""
    path = os.path.join(get_package_share_directory('fm_sim_bringup'), 'launch', 'run.launch.py')
    spec = importlib.util.spec_from_file_location('run_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    world_dir = tmp_path / 'world'
    world_dir.mkdir()
    monkeypatch.setattr(module.tempfile, 'mkdtemp', lambda prefix: str(world_dir))
    context = LaunchContext()
    context.launch_configurations.update({
        'backend': 'mujoco', 'robot': 'so101', 'scenario': 'can_on_table',
        'controller': 'idle', 'use_sim': 'true', 'seed': '4', 'timeout_s': '',
        'params_file': params_file, 'overrides_file': '',
    })
    return module._launch_setup(context), context


def _params_file(tmp_path, document):
    path = tmp_path / 'params.yaml'
    path.write_text(yaml.safe_dump(document))
    return str(path)


def _scenario_node(actions):
    (node,) = [a for a in actions if isinstance(a, Node) and a.node_executable == 'scenario_node']
    return node


def test_params_file_wins_on_the_scenario_node_and_the_guard_holds(monkeypatch, tmp_path):
    params_file = _params_file(tmp_path, {'/sim_scenario': {'ros__parameters': {
        'frame_id': 'map', 'seed': 9, 'timeout_s': 1.0, 'use_sim_time': False,
        'backend': 'gazebo', 'scenario': '{}',
    }}})
    actions, context = _launch_setup(monkeypatch, tmp_path, params_file)
    node = _scenario_node(actions)
    assert_params_file_wins(node, context, {'frame_id': 'map'})
    resolved = resolved_parameters(node, context, ['scenario'])
    assert json.loads(resolved['scenario'])['name'] == 'can_on_table'
    assert_guard_holds(node, context, {
        'seed': 4, 'timeout_s': 0.0, 'use_sim_time': True, 'backend': 'mujoco',
    })


def test_without_a_params_file_the_nodes_are_unchanged(monkeypatch, tmp_path):
    actions, context = _launch_setup(monkeypatch, tmp_path, '')
    node = _scenario_node(actions)
    assert resolved_parameters(node, context, ['frame_id', 'seed']) == {'seed': 4}
    assert os.listdir(tmp_path / 'world') == ['can_on_table_4.xml']
    spawners = [a for a in actions if isinstance(a, Node) and a.node_executable == 'spawner']
    for spawner in spawners:
        assert list(spawner._Node__arguments).count('--param-file') == 1


def test_params_file_reaches_spawners_and_included_launches(monkeypatch, tmp_path):
    params_file = _params_file(tmp_path, {'/so101_arm_controller': {'ros__parameters': {
        'allow_nonzero_velocity_at_trajectory_end': False,
    }}})
    actions, _ = _launch_setup(monkeypatch, tmp_path, params_file)
    spawners = [a for a in actions if isinstance(a, Node) and a.node_executable == 'spawner']
    assert len(spawners) == 2
    for spawner in spawners:
        arguments = list(spawner._Node__arguments)
        assert arguments[arguments.index(params_file) - 1] == '--param-file'
        controllers = arguments.index('--param-file')
        assert controllers < arguments.index(params_file)
    (backend,) = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert dict(backend.launch_arguments)['params_file'] == params_file


def test_a_controllers_params_file_beats_the_controllers_file(tmp_path):
    # The spawner hands both files to the controller in this order; the
    # controllers file keys it relative, the params file fully qualified.
    params_file = _params_file(tmp_path, {'/so101_arm_controller': {'ros__parameters': {
        'allow_nonzero_velocity_at_trajectory_end': False,
    }}})
    controllers_file = catalog.share_file(
        catalog.load('robots', 'so101')['controllers_file']
    )
    node = Node(package='controller_manager', executable='spawner', name='so101_arm_controller',
                parameters=[controllers_file, params_file])
    assert_params_file_wins(
        node, LaunchContext(), {'allow_nonzero_velocity_at_trajectory_end': False}
    )


def test_a_missing_params_file_is_refused(monkeypatch, tmp_path):
    with pytest.raises(catalog.CatalogError, match='params_file is not a file'):
        _launch_setup(monkeypatch, tmp_path, str(tmp_path / 'absent.yaml'))


def test_own_node_guards_are_what_params_reserves():
    camera = run.CameraPlacement(
        'chest', '/c/cameras.yaml', '/mujoco_cameras/cam_chest/color',
        mujoco_world.Camera('cam_chest', None, (0, 0, 0), (1, 0, 0, 0), 60.0, (64, 48)),
    )
    entries = run.resolve('mujoco', 'so101', 'can_on_table', 'idle')
    scenario = run.scenario_node_parameters(entries['backend'], entries['scenario'], 0, 0.0)
    guards = run.own_node_guards([camera], scenario)
    assert set(guards['/sim_camera_chest']) == params.CAMERA_OWNED
    assert set(guards['/sim_scenario']) == params.SCENARIO_OWNED


def _with_camera(monkeypatch):
    """so101 given a chest camera; its world is not built (it has no camera body)."""
    camera = run.CameraPlacement(
        'chest', '/c/cameras.yaml', '/mujoco_cameras/cam_chest/color',
        mujoco_world.Camera('cam_chest', None, (0, 0, 0), (1, 0, 0, 0), 60.0, (64, 48)),
    )
    monkeypatch.setattr(run, 'sim_cameras', lambda *args: [camera])
    monkeypatch.setattr(run, 'world_model', lambda *args: '/world.xml')


def test_the_camera_guard_holds_and_its_tunables_win(monkeypatch, tmp_path):
    _with_camera(monkeypatch)
    params_file = _params_file(tmp_path, {'/sim_camera_chest': {'ros__parameters': {
        'jpeg_quality': 50, 'role': 'wrist_right', 'render_topic': '/x', 'use_sim_time': False,
    }}})
    actions, context = _launch_setup(monkeypatch, tmp_path, params_file)
    (camera,) = [a for a in actions if isinstance(a, Node) and a.node_executable == 'sim_camera']
    assert_params_file_wins(camera, context, {'jpeg_quality': 50})
    assert_guard_holds(camera, context, {
        'role': 'chest', 'render_topic': '/mujoco_cameras/cam_chest/color',
        'use_sim_time': True, 'cameras_file': '/c/cameras.yaml',
    })


def _with_launches(monkeypatch):
    """so101 given OpenArm's servo role and a controller with a launch that activates it."""
    resolve = run.resolve

    def resolve_with(**choice):
        entries = resolve(**choice)
        entries['robot']['launch_roles'] = catalog.load('robots', 'openarm')['launch_roles']
        entries['robot']['robot_profile'] = 'probe_profile'
        entries['controller'] = dict(entries['controller'], activate=['servo'], launch=(
            entries['robot']['launch_roles']['servo']['launch']
        ))
        return entries
    monkeypatch.setattr(run, 'resolve', resolve_with)


@pytest.mark.parametrize('params_file', ['set', ''])
def test_params_file_is_forwarded_to_role_and_controller_launches(
        monkeypatch, tmp_path, params_file):
    _with_launches(monkeypatch)
    path = _params_file(tmp_path, {}) if params_file else ''
    actions, _ = _launch_setup(monkeypatch, tmp_path, path)
    (controller,) = [
        a for a in actions if isinstance(a, IncludeLaunchDescription)
        and 'robot_profile' in dict(a.launch_arguments)
    ]
    assert dict(controller.launch_arguments).get('params_file') == (path or None)
    (handler,) = [a.event_handler for a in actions if isinstance(a, RegisterEventHandler)
                  and isinstance(a.event_handler, OnProcessExit)]
    spawner = next(a for a in actions if isinstance(a, Node))
    exited = ProcessExited(action=spawner, name='spawner', cmd=['spawner'], cwd=None, env=None,
                           pid=1, returncode=0)
    (role,) = handler.handle(exited, LaunchContext())
    assert dict(role.launch_arguments).get('params_file') == (path or None)


# -- description overrides ---------------------------------------------------

DAMPING = {
    'label': 'Shoulder pan damping', 'type': 'float', 'default': 0.6, 'min': 0.0, 'max': 10.0,
    'backends': ['mujoco'],
    'target': {'kind': 'description',
               'mjcf': {'element': 'joint', 'name': 'shoulder_pan', 'attribute': 'damping'}},
}
XACRO = {
    'label': 'Hand', 'type': 'enum', 'enum': ['loader', 'pinch'], 'default': 'loader',
    'target': {'kind': 'description', 'xacro_arg': 'hand'},
}


def _robot_entries(**tunables):
    entries = run.resolve('mujoco', 'so101', 'can_on_table', 'idle')
    entries['robot']['params'] = tunables
    return entries


def test_a_description_override_changes_the_compiled_mjcf(tmp_path):
    mujoco = pytest.importorskip('mujoco')
    entries = _robot_entries(damping=DAMPING)
    overrides = {'robot.damping': 4}
    described = run.description_overrides(run.apply_overrides(entries, overrides), overrides)
    patch = ('joint', 'shoulder_pan', 'damping', '4.0')
    assert described == run.DescriptionOverrides({}, (patch,))
    patched = run.patched_mjcf(mjcf_path('so101'), described.mjcf, str(tmp_path))
    assert patched == str(tmp_path / 'robot_patched.xml')

    def damping(path):
        model = mujoco.MjModel.from_xml_path(path)
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'shoulder_pan')
        return model.dof_damping[model.jnt_dofadr[joint]]

    assert damping(mjcf_path('so101')) != 4.0
    assert damping(patched) == 4.0
    world = run.world_model(
        entries['backend'], entries['robot'], entries['scenario'], 0, patched, str(tmp_path)
    )
    assert damping(world) == 4.0


def test_no_mjcf_patch_keeps_the_registered_model(tmp_path):
    assert run.patched_mjcf('/models/robot.xml', (), str(tmp_path)) == '/models/robot.xml'


def test_a_patch_naming_no_element_is_refused(tmp_path):
    with pytest.raises(catalog.CatalogError, match="0 <joint> named 'nope'"):
        run.patched_mjcf(mjcf_path('so101'), (('joint', 'nope', 'damping', '1'),), str(tmp_path))


def test_a_xacro_override_becomes_a_description_arg():
    entries = _robot_entries(hand=XACRO)
    overrides = {'robot.hand': 'pinch'}
    described = run.description_overrides(run.apply_overrides(entries, overrides), overrides)
    assert described == run.DescriptionOverrides({'hand': 'pinch'}, ())


def test_an_mjcf_override_off_mujoco_is_refused():
    entries = run.resolve('gazebo', 'so101', 'can_on_table', 'idle')
    entries['robot']['params'] = {'damping': DAMPING}
    with pytest.raises(catalog.CatalogError, match='applies only on mujoco'):
        run.apply_overrides(entries, {'robot.damping': 4.0})
