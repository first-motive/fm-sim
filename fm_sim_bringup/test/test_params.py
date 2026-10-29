"""Tunables: the loader's checks of a declaration, and the value check a run shares."""

import copy
import json
from pathlib import Path

from fm_sim_bringup import catalog, params
import pytest

from test_catalog import extra_prefix, PLACEHOLDER  # noqa: F401 - the fixture

CASES = json.loads((Path(__file__).parent / 'param_cases.json').read_text())['cases']

SPEED = {
    'label': 'Arm speed',
    'type': 'float',
    'default': 0.5,
    'min': 0.1,
    'max': 2.0,
    'step': 0.05,
    'unit': 'rad/s',
    'target': {'kind': 'ros_param', 'node': '/motion_server', 'param': 'max_joint_speed'},
}


@pytest.mark.parametrize('case', CASES, ids=[case['name'] for case in CASES])
def test_check_value_matches_the_shared_cases(case):
    if case['accepted']:
        value = params.check_value(case['param'], case['value'])
        assert value == case['normalised']
        assert json.dumps(value) == json.dumps(case['normalised'])  # 1.0 stays 1.0
    else:
        with pytest.raises(params.ParamError, match=case['reason']):
            params.check_value(case['param'], case['value'])


def _controller(**tunables):
    return dict(PLACEHOLDER, available=True, params=tunables)


def test_a_declared_tunable_loads(extra_prefix):  # noqa: F811
    extra_prefix('wam', _controller(max_joint_speed=SPEED))
    assert catalog.load('controllers', 'wam')['params']['max_joint_speed'] == SPEED


def _with(**changes):
    tunable = copy.deepcopy(SPEED)
    for key, value in changes.items():
        if value is None:
            del tunable[key]
        else:
            tunable[key] = value
    return tunable


@pytest.mark.parametrize('tunable, match', [
    (_with(default=3.0), 'default must be between'),
    (_with(default='fast'), 'default must be a number'),
    (_with(min=3.0), 'min is above max'),
    (_with(max=None), "'max' is a required property"),
    (_with(type='int', min=0, max=2), 'default must be an integer'),
    (_with(type='enum', min=None, max=None, step=None, enum=['a'], default='b'),
     'default must be one of a'),
    (_with(type='array', items='float', length=2, default=[0.5]), 'default must be a list of 2'),
    (_with(target={'kind': 'service', 'node': '/x'}), "'service' is not one of"),
    (_with(target={'kind': 'ros_param', 'node': 'motion_server', 'param': 'x'}),
     'does not match'),
    (_with(target={'kind': 'ros_param', 'node': '/**', 'param': 'x'}), 'does not match'),
    (_with(target={'kind': 'ros_param', 'node': '/**/motion_server', 'param': 'x'}),
     'does not match'),
    (_with(target={'kind': 'ros_param', 'node': '/x'}), "'param' is a required property"),
    (_with(target={'kind': 'description', 'xacro_arg': 'a', 'mjcf': {
        'element': 'joint', 'name': 'j', 'attribute': 'damping'}}), 'valid under each'),
    (_with(target={'kind': 'description', 'mjcf': {
        'element': 'joint', 'name': 'j', 'attribute': 'damping'}}), 'mjcf target needs backends'),
    (_with(target={'kind': 'ros_param', 'node': '/x', 'param': 'use_sim_time'}),
     'use_sim_time is launch-owned'),
    (_with(target={'kind': 'ros_param', 'node': '/x', 'param': 'command_out_topic'}),
     'command_out_topic is launch-owned'),
    (_with(target={'kind': 'ros_param', 'node': '/policy', 'param': 'io.checkpoint'}),
     'io.checkpoint is launch-owned'),
    (_with(target={'kind': 'ros_param', 'node': '/x', 'param': 'robot_description_semantic'}),
     'robot_description_semantic is launch-owned'),
    (_with(target={'kind': 'ros_param', 'node': '/sim_scenario', 'param': 'seed'}),
     'seed is launch-owned'),
    (_with(target={'kind': 'ros_param', 'node': '/sim_camera_chest', 'param': 'role'}),
     'role is launch-owned'),
    (_with(target={'kind': 'ros_param', 'node': '/_x', 'param': 'p'}), 'does not match'),
    (_with(target={'kind': 'description', 'mjcf': {
        'element': 'joint', 'name': 'j', 'attribute': 'damping'}}, backends=['mujoco', 'gazebo']),
     'mjcf target needs backends'),
    (_with(target={'kind': 'description', 'mjcf': {
        'element': 'motor', 'name': 'j', 'attribute': 'damping'}}, backends=['mujoco']),
     "'motor' is not one of"),
    (_with(target={'kind': 'description', 'mjcf': {
        'element': 'joint', 'name': 'j', 'attribute': 'armature'}}, backends=['mujoco']),
     "'armature' is not one of"),
    (_with(type='int', default=5, min=0.5, max=10), 'min and max must be integers'),
    (_with(ordered=True), 'should not be valid'),
    (_with(label=None), "'label' is a required property"),
    (_with(colour='red'), 'Additional properties'),
], ids=[
    'default out of bounds', 'default of the wrong type', 'min above max', 'no max',
    'float default for an int', 'enum default not a member', 'array default too short',
    'unknown target kind', 'relative node name', 'wildcard node', 'wildcard namespace',
    'ros_param without param',
    'description with two targets', 'mjcf without backends', 'guarded ros param',
    'guarded topic', 'guarded path param', 'guarded description', 'scenario node param',
    'camera node param',
    'node starting with an underscore', 'mjcf on gazebo too', 'mjcf element not patched',
    'mjcf attribute not patched', 'int tunable with float bounds', 'ordered on a float',
    'no label', 'unknown field',
])
def test_loader_rejects_a_bad_tunable(extra_prefix, tunable, match):  # noqa: F811
    extra_prefix('wam', _controller(speed=tunable))
    with pytest.raises(catalog.CatalogError, match=match):
        catalog.load('controllers', 'wam')


def test_moveit_joint_limits_are_tunable():
    # fm-skills tunes these; only the description itself is launch-owned.
    assert not params.guarded(
        '/fm_skills_moveit_py',
        'robot_description_planning.joint_limits.follower_r_joint1.max_velocity',
    )
    assert params.guarded('/fm_skills_moveit_py', 'robot_description_kinematics.right_arm.x')


def test_tunable_name_must_be_a_name(extra_prefix):  # noqa: F811
    extra_prefix('wam', _controller(**{'Max-Speed': SPEED}))
    with pytest.raises(catalog.CatalogError, match='does not match'):
        catalog.load('controllers', 'wam')


def _scenario_with(tunable):
    entry = catalog.load('scenarios', 'can_on_table')
    del entry['package']
    entry['params'] = {'can_mass': tunable}
    return entry


CAN_MASS = {
    'label': 'Can mass', 'type': 'float', 'default': 0.35, 'min': 0.05, 'max': 2.0,
    'unit': 'kg', 'target': {'kind': 'entry', 'path': 'objects.can.mass'},
}


def test_entry_target_must_hold_the_default(extra_prefix):  # noqa: F811
    extra_prefix('cans', dict(_scenario_with(CAN_MASS), name='cans'), kind='scenarios')
    assert catalog.load('scenarios', 'cans')['params']['can_mass']['default'] == 0.35
    extra_prefix('cans', dict(_scenario_with(dict(CAN_MASS, default=0.5)), name='cans'),
                 kind='scenarios')
    with pytest.raises(catalog.CatalogError, match='default is not the value at objects.can'):
        catalog.load('scenarios', 'cans')


def test_entry_target_path_must_name_something(extra_prefix):  # noqa: F811
    tunable = dict(CAN_MASS, target={'kind': 'entry', 'path': 'objects.bottle.mass'})
    extra_prefix('cans', dict(_scenario_with(tunable), name='cans'), kind='scenarios')
    with pytest.raises(catalog.CatalogError, match="segment 'bottle' names nothing"):
        catalog.load('scenarios', 'cans')


def test_entry_value_addresses_list_items_by_name():
    entry = {'objects': [{'name': 'can', 'spawn': {'x': [-0.03, 0.03]}}]}
    assert params.entry_value(entry, 'objects.can.spawn.x') == [-0.03, 0.03]
    params.set_entry_value(entry, 'objects.can.spawn.x', [0.0, 0.01])
    assert entry['objects'][0]['spawn']['x'] == [0.0, 0.01]
    with pytest.raises(params.ParamError, match="'y' names nothing"):
        params.set_entry_value(entry, 'objects.can.spawn.y', [0.0, 0.0])


@pytest.mark.parametrize('second', [
    {'kind': 'ros_param', 'node': '/motion_server', 'param': 'max_joint_speed'},
    {'kind': 'ros_param', 'node': '/motion_server', 'param': 'max_joint_speed.arm'},
], ids=['the same param', 'a param inside it'])
def test_two_tunables_setting_one_ros_param_are_refused(extra_prefix, second):  # noqa: F811
    extra_prefix('wam', _controller(speed=SPEED, other=_with(target=second)))
    with pytest.raises(catalog.CatalogError, match='sets the same ROS param as'):
        catalog.load('controllers', 'wam')


def test_one_node_two_params_load(extra_prefix):  # noqa: F811
    other = _with(target={'kind': 'ros_param', 'node': '/motion_server', 'param': 'max_accel'})
    extra_prefix('wam', _controller(speed=SPEED, accel=other))
    assert set(catalog.load('controllers', 'wam')['params']) == {'speed', 'accel'}


def test_entry_target_must_not_end_on_an_identity_field(extra_prefix):  # noqa: F811
    tunable = {'label': 'Shape', 'type': 'enum', 'enum': ['box', 'cylinder'],
               'default': 'cylinder', 'target': {'kind': 'entry', 'path': 'objects.can.shape'}}
    extra_prefix('cans', dict(_scenario_with(tunable), name='cans'), kind='scenarios')
    with pytest.raises(catalog.CatalogError, match='objects.can.shape names an identity field'):
        catalog.load('scenarios', 'cans')
