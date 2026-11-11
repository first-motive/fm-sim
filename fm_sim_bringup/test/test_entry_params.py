"""The tunables fm-sim's own entries declare: their defaults are what the run uses.

Each default is checked against the file or model its target overrides, so the
app never shows a value the run does not start with, and no tunable names a
parameter the launch owns: neither one ``params.GUARDED_PARAMS`` reserves nor
one the launch that starts the tunable's node guards.
"""

import importlib.util
import json
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from description_model import needs_openarm
from fm_sim_backends.params_check import assert_params_file_wins
from fm_sim_bringup import catalog, params
from fm_sim_cameras import cameras_file
from fm_sim_models.models import mjcf_path
from launch import LaunchContext
from launch_ros.actions import Node
import pytest
import yaml

from test_run_params import _launch_setup

# Every node an fm-sim entry tunes, and the file it otherwise reads that param from.
PLUGINS_FILE = ('fm_sim_backends', 'config/mujoco_plugins.yaml')
# The servo role's own file (fm-robot), which the openarm servo tunables default to.
SERVO_FILE = ('fm_control', 'config/openarm/servo_sim.yaml')
# The sim_camera node's own default for a param no file sets.
CAMERA_DEFAULTS = {'jpeg_quality': 90, 'depth_sensor': False}
# The launches that start a node an fm-sim tunable targets, and guard it.
ENGINE_LAUNCH = ('fm_sim_backends', 'launch/mujoco.launch.py')
SERVO_LAUNCH = ('fm_control', 'launch/openarm_sim_servo.launch.py')
# Just the right_arm chain the servo launch checks a passed description for.
SERVO_CHAIN_URDF = (
    '<robot name="probe"><link name="openarm_body_link0"/>'
    '<link name="openarm_right_grasp_frame"/></robot>'
)


def _tunables(kind, name):
    entry = catalog.load(kind, name)
    return entry, entry.get('params', {})


def _node_yaml_value(document, node, param):
    """``param`` of ``node`` in a ROS params file keyed relative or fully qualified."""
    for key in (node, node.lstrip('/')):
        if key in document:
            value = document[key]['ros__parameters']
            for segment in param.split('.'):
                value = value[segment]
            return value
    raise KeyError(node)


def _share_yaml(package, path):
    with open(f'{get_package_share_directory(package)}/{path}') as yaml_file:
        return yaml.safe_load(yaml_file)


ENTRIES = [
    ('backends', 'mujoco'), ('robots', 'so101'), ('robots', 'axol'), ('robots', 'openarm'),
    ('robots', 'panda'),
    ('scenarios', 'can_on_table'),
]


@pytest.mark.parametrize('kind, name', ENTRIES)
def test_ros_param_defaults_are_what_the_nodes_read(kind, name):
    entry, tunables = _tunables(kind, name)
    assert tunables or kind == 'scenarios'
    for tunable_name, tunable in tunables.items():
        target = tunable['target']
        if target['kind'] != 'ros_param':
            continue
        node, param = target['node'], target['param']
        if node.startswith('/sim_camera_'):
            role = node.removeprefix('/sim_camera_')
            cameras = _share_yaml(**entry['cameras_file'])['cameras']
            # The camera publishes no faster than the engine renders.
            render = _node_yaml_value(
                _share_yaml(*PLUGINS_FILE), '/mujoco_ros2_control_node',
                'mujoco_plugins.mujoco_cameras.camera_publish_rate',
            )
            expected = (
                min(cameras[role][param], render) if param == 'rate_hz'
                else CAMERA_DEFAULTS[param]
            )
        elif node == '/mujoco_ros2_control_node':
            expected = _node_yaml_value(_share_yaml(*PLUGINS_FILE), node, param)
        elif node == '/servo_node':
            expected = _node_yaml_value(_share_yaml(*SERVO_FILE), node, param)
        else:
            controllers = _share_yaml(**entry['controllers_file'])
            expected = _node_yaml_value(controllers, node, param)
        assert tunable['default'] == expected, tunable_name


def _compiled(path, element, name, attribute):
    mujoco = pytest.importorskip('mujoco')
    model = mujoco.MjModel.from_xml_path(path)
    if element == 'joint':
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert attribute == 'damping'
        return model.dof_damping[model.jnt_dofadr[joint]]
    actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    assert actuator >= 0, name
    if attribute == 'kp':
        return model.actuator_gainprm[actuator][0]
    assert attribute == 'forcerange'
    return list(model.actuator_forcerange[actuator])


def _check_mjcf_defaults(robot):
    entry, tunables = _tunables('robots', robot)
    path = mjcf_path(entry['mjcf'])
    mjcf = {n: t for n, t in tunables.items() if 'mjcf' in t['target']}
    assert mjcf
    for tunable_name, tunable in mjcf.items():
        patch = tunable['target']['mjcf']
        value = _compiled(path, patch['element'], patch['name'], patch['attribute'])
        assert value == pytest.approx(tunable['default']), tunable_name


def test_so101_mjcf_defaults_are_the_compiled_model():
    _check_mjcf_defaults('so101')


def test_panda_mjcf_defaults_are_the_compiled_model():
    _check_mjcf_defaults('panda')


@needs_openarm
def test_openarm_mjcf_defaults_are_the_compiled_model():
    _check_mjcf_defaults('openarm')


@pytest.mark.parametrize('kind, name', ENTRIES)
def test_no_tunable_names_what_the_launch_owns(kind, name):
    for tunable in _tunables(kind, name)[1].values():
        target = tunable['target']
        if target['kind'] == 'ros_param':
            assert not params.guarded(target['node'], target['param'])


def _launch_module(package, path):
    """Load the launch file ``path`` in ``package``'s share as a module."""
    spec = importlib.util.spec_from_file_location(
        path.replace('/', '_').removesuffix('.py').replace('.', '_'),
        f'{get_package_share_directory(package)}/{path}',
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _ros_param_tunables(kind, name, node):
    """``{param: tunable}`` for the tunables of entry ``name`` that target ``node``."""
    return {
        tunable['target']['param']: tunable
        for tunable in _tunables(kind, name)[1].values()
        if tunable['target']['kind'] == 'ros_param' and tunable['target']['node'] == node
    }


def _covers(guard, param):
    """Whether the nested ``guard`` holds ``param`` (dotted) or a key above it."""
    for segment in param.split('.'):
        if not isinstance(guard, dict):
            return True
        if segment not in guard:
            return False
        guard = guard[segment]
    return True


def test_the_engine_launch_guards_no_engine_tunable():
    engine = _launch_module(*ENGINE_LAUNCH)
    plugins_file = f'{get_package_share_directory(PLUGINS_FILE[0])}/{PLUGINS_FILE[1]}'
    guards = engine._guards('', plugins_file)
    tunables = _ros_param_tunables('backends', 'mujoco', engine.HARDWARE_NODE)
    assert tunables
    for param in tunables:
        assert not _covers(guards[engine.HARDWARE_NODE], param), param


def test_a_params_file_wins_for_every_servo_tunable(monkeypatch, tmp_path):
    """Each servo tunable, set in a params file, resolves through rcl to the file's value.

    Judged on the Node the servo launch (fm-robot) builds, so the check follows
    whatever its guard file holds, not a copy of its keys here.
    """
    tunables = _ros_param_tunables('robots', 'openarm', '/servo_node')
    assert tunables
    # A step off each default, so a value that resolves to it came from the file.
    expected = {param: tunable['default'] + tunable['step'] for param, tunable in tunables.items()}
    document = {}
    for param, value in expected.items():
        *parents, leaf = param.split('.')
        level = document
        for segment in parents:
            level = level.setdefault(segment, {})
        level[leaf] = value
    params_file = tmp_path / 'params.yaml'
    params_file.write_text(yaml.safe_dump({'/servo_node': {'ros__parameters': document}}))
    # The launch refuses to start unless ROS discovery is host-local.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    # The launch writes its guard file to the temp dir; keep it in this test's.
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update({
        'sim_only': 'true', 'command_type': 'joint_jog', 'names': 'openarm',
        'robot_description': SERVO_CHAIN_URDF, 'params_file': str(params_file),
    })
    actions = _launch_module(*SERVO_LAUNCH)._launch_setup(context)
    (servo,) = [action for action in actions if isinstance(action, Node)]
    assert_params_file_wins(servo, context, expected)


def test_the_camera_rate_never_exceeds_what_is_rendered():
    _, mujoco = _tunables('backends', 'mujoco')
    _, openarm = _tunables('robots', 'openarm')
    render_max = mujoco['camera_render_rate']['max']
    for role in cameras_file.ROLES:
        assert openarm[f'{role}_camera_rate']['max'] <= render_max


def test_can_size_bounds_keep_the_spawn_out_of_what_it_stands_on():
    entry, tunables = _tunables('scenarios', 'can_on_table')
    objects = {obj['name']: obj for obj in entry['objects']}
    can, size = objects['can'], tunables['can_size']
    assert can['shape'] == 'cylinder' and size['target']['path'] == 'objects.can.size'
    tallest = size['max']  # the height is the second element; both share the bounds
    base = can['pose']['xyz'][2] + can['spawn'].get('z', [0.0, 0.0])[0] - tallest / 2
    tops = [
        obj['pose']['xyz'][2] + obj['size'][2] / 2
        for obj in objects.values() if obj.get('static') and obj['shape'] == 'box'
    ]
    assert base >= max(tops) - 1e-9


def test_entry_tunables_change_the_world_through_the_launch(monkeypatch, tmp_path):
    overrides = tmp_path / 'params.json'
    overrides.write_text(json.dumps({
        'scenario.can_mass': 0.9, 'scenario.timeout': 30, 'robot.shoulder_pan_damping': 2.5,
    }))
    actions, _ = _launch_setup(monkeypatch, tmp_path, '', str(overrides))
    world_dir = tmp_path / 'world'
    with open(world_dir / 'can_on_table_4.xml') as world_file:
        world = world_file.read()
    assert re.search(r'<geom name="can"[^>]* mass="0.9"', world)
    assert 'robot_patched.xml' in world
    assert _compiled(str(world_dir / 'can_on_table_4.xml'), 'joint', 'shoulder_pan',
                     'damping') == 2.5
