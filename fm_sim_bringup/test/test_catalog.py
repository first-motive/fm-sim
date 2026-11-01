"""Catalog lookup, validation and the catalog --json contract."""

import json
import os

from fm_sim_bringup import catalog
import jsonschema
import pytest
import yaml

PLACEHOLDER = {
    'name': 'wam',
    'description': 'World-action model; not built yet',
    'available': False,
    'activate': [],
    'launch': None,
}


@pytest.fixture
def extra_prefix(tmp_path, monkeypatch):
    """Return a function that registers an entry (default: a controller) in a temp prefix."""
    monkeypatch.setenv(
        'AMENT_PREFIX_PATH', f'{tmp_path}{os.pathsep}{os.environ["AMENT_PREFIX_PATH"]}'
    )

    def register(name, entry, marker_content=None, kind='controllers'):
        marker = tmp_path / f'share/ament_index/resource_index/fm_sim.{kind}' / name
        marker.parent.mkdir(parents=True, exist_ok=True)
        relative = f'fake_pkg/catalog/{kind}/{name}.yaml'
        marker.write_text((marker_content or relative) + '\n')
        entry_file = tmp_path / 'share' / relative
        entry_file.parent.mkdir(parents=True, exist_ok=True)
        entry_file.write_text(yaml.safe_dump(entry))
        return entry_file

    return register


def test_schema_is_valid_json_schema():
    jsonschema.Draft7Validator.check_schema(catalog.load_schema())


def test_registered_entries():
    assert {'mujoco', 'gazebo', 'isaac'} <= set(catalog.names('backends'))
    assert {'openarm', 'so101', 'axol'} <= set(catalog.names('robots'))
    assert {'empty', 'can_on_table', 'table_only'} <= set(catalog.names('scenarios'))
    assert {'idle', 'teleop'} <= set(catalog.names('controllers'))


def test_load_adds_the_registering_package():
    assert catalog.load('backends', 'mujoco')['package'] == 'fm_sim_backends'
    assert catalog.load('robots', 'openarm')['package'] == 'fm_sim_bringup'


def test_unknown_name_lists_the_registered_ones():
    with pytest.raises(catalog.CatalogError, match='unknown robot .nope.; registered: .*openarm'):
        catalog.load('robots', 'nope')


def test_catalog_json_matches_the_schema(capsys):
    assert catalog.main(['--json']) == 0
    output = json.loads(capsys.readouterr().out)
    jsonschema.validate(output, catalog.load_schema())
    assert output['schema_version'] == catalog.SCHEMA_VERSION
    assert [entry['name'] for entry in output['scenarios']] == catalog.names('scenarios')
    for robot in output['robots']:
        assert robot['supported_backends'] == ['mujoco', 'gazebo', 'isaac'], robot['name']


def test_unavailable_entry_is_listed_but_refused(extra_prefix):
    extra_prefix('wam', PLACEHOLDER)
    assert catalog.load('controllers', 'wam')['available'] is False
    with pytest.raises(catalog.CatalogError, match='not available'):
        catalog.require_available('controllers', 'wam')


def test_entry_needing_a_missing_package_is_listed_but_refused(extra_prefix, capsys):
    extra_prefix('wam', dict(PLACEHOLDER, available=True,
                             requires_packages=['fm_sim_bringup', 'no_such_package']))
    entry = catalog.load('controllers', 'wam')
    assert entry['available'] is False
    assert entry['missing_packages'] == ['no_such_package']
    with pytest.raises(catalog.CatalogError, match='needs no_such_package, which is not inst'):
        catalog.require_available('controllers', 'wam')
    assert catalog.main([]) == 0
    (listed,) = [line for line in capsys.readouterr().out.splitlines()
                 if line.split()[0] == 'wam']
    assert listed.endswith('(needs no_such_package, which is not installed)')
    assert catalog.main(['--json']) == 0
    jsonschema.validate(json.loads(capsys.readouterr().out), catalog.load_schema())


def test_entry_whose_packages_are_installed_is_available(extra_prefix):
    extra_prefix('wam', dict(PLACEHOLDER, available=True, requires_packages=['fm_sim_bringup']))
    entry = catalog.require_available('controllers', 'wam')
    assert 'missing_packages' not in entry


def test_requires_packages_cannot_make_a_placeholder_available(extra_prefix):
    extra_prefix('wam', dict(PLACEHOLDER, requires_packages=['fm_sim_bringup']))
    with pytest.raises(catalog.CatalogError, match='not available yet'):
        catalog.require_available('controllers', 'wam')


def test_malformed_entry_is_rejected(extra_prefix):
    broken = dict(PLACEHOLDER, name='broken')
    del broken['activate']
    extra_prefix('broken', broken)
    with pytest.raises(catalog.CatalogError, match="'activate' is a required property"):
        catalog.load('controllers', 'broken')


def test_controller_scenarios_must_be_registered(extra_prefix):
    extra_prefix('listed', dict(PLACEHOLDER, name='listed', scenarios=['can_on_table']))
    assert catalog.load('controllers', 'listed')['scenarios'] == ['can_on_table']
    extra_prefix('misspelt', dict(PLACEHOLDER, name='misspelt', scenarios=['can_on_tabel']))
    with pytest.raises(catalog.CatalogError, match='unknown scenarios can_on_tabel'):
        catalog.load('controllers', 'misspelt')


def test_controller_scenarios_cannot_be_empty(extra_prefix):
    extra_prefix('none', dict(PLACEHOLDER, name='none', scenarios=[]))
    with pytest.raises(catalog.CatalogError, match='is too short|should be non-empty'):
        catalog.load('controllers', 'none')


def test_controller_carries_its_launch_args(extra_prefix):
    launch_args = {'perception': 'model', 'pack': 'can_paper_seg'}
    extra_prefix('wam', dict(PLACEHOLDER, launch_args=launch_args))
    assert catalog.load('controllers', 'wam')['launch_args'] == launch_args


@pytest.mark.parametrize('launch_args, match', [
    ({'Bad-Key': 'x'}, 'does not match'),
    ({'perception': 1}, 'is not of type'),
], ids=['key_name', 'not_a_string'])
def test_malformed_controller_launch_args_are_rejected(extra_prefix, launch_args, match):
    extra_prefix('wam', dict(PLACEHOLDER, launch_args=launch_args))
    with pytest.raises(catalog.CatalogError, match=match):
        catalog.load('controllers', 'wam')


@pytest.mark.parametrize('owned', sorted(catalog.CONTROLLER_LAUNCH_OWNED))
def test_controller_launch_args_cannot_set_what_the_run_owns(extra_prefix, owned):
    extra_prefix('wam', dict(PLACEHOLDER, launch_args={'pack': 'x', owned: 'y'}))
    with pytest.raises(catalog.CatalogError, match=f'launch_args may not set {owned}; '):
        catalog.load('controllers', 'wam')


def test_entry_must_name_itself(extra_prefix):
    extra_prefix('alias', PLACEHOLDER)
    with pytest.raises(catalog.CatalogError, match="names itself 'wam'"):
        catalog.load('controllers', 'alias')


@pytest.mark.parametrize('robot', catalog.names('robots'))
def test_robot_roles_name_controllers_its_file_declares(robot):
    entry = catalog.load('robots', robot)
    with open(catalog.share_file(entry['controllers_file'])) as controllers_file:
        declared = yaml.safe_load(controllers_file)['controller_manager']['ros__parameters']
    for names in entry['controller_roles'].values():
        for name in names:
            assert 'type' in declared.get(name, {}), f'{robot}: {name} has no type'


@pytest.mark.parametrize('marker', ['../escape.yaml', 'fake_pkg/../../escape.yaml', '/abs.yaml'])
def test_marker_must_stay_inside_its_package(extra_prefix, marker):
    extra_prefix('wam', PLACEHOLDER, marker_content=marker)
    with pytest.raises(catalog.CatalogError, match='marker'):
        catalog.load('controllers', 'wam')


@pytest.mark.parametrize('path', ['../fm_sim_backends/package.xml', '/etc/hosts', 'a/../../b'])
def test_share_file_must_stay_inside_its_package(path):
    with pytest.raises(catalog.CatalogError, match='share file'):
        catalog.share_file({'package': 'fm_sim_bringup', 'path': path})


@pytest.mark.parametrize('path', ['../x.yaml', '/x.yaml', 'a/../b.yaml'])
def test_schema_rejects_escaping_paths(path):
    schema = catalog.load_schema()
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(
            {'package': 'p', 'path': path}, {**schema, '$ref': '#/definitions/share_file'}
        )


def _openarm_with_servo(servo):
    robot = catalog.load('robots', 'openarm')
    robot['launch_roles'] = {'servo': servo}
    return robot


def _validate_robot(robot):
    schema = catalog.load_schema()
    jsonschema.validate(robot, {**schema, '$ref': '#/definitions/robot'})


def test_schema_accepts_a_launch_role():
    _validate_robot(_openarm_with_servo({
        'requires': ['arm_trajectory'],
        'launch': {'package': 'p', 'path': 'x.launch.py'},
        'launch_args': {'sim_only': 'true'},
    }))


@pytest.mark.parametrize('servo', [
    {'requires': ['arm_trajectory']},
    {'requires': [], 'launch': {'package': 'p', 'path': 'x.launch.py'}},
    {'requires': ['arm_trajectory'], 'launch': {'package': 'p', 'path': 'x.launch.py'},
     'command_type': 'twist'},
], ids=['no_launch', 'empty_requires', 'unknown_key'])
def test_schema_rejects_a_malformed_launch_role(servo):
    with pytest.raises(jsonschema.ValidationError):
        _validate_robot(_openarm_with_servo(servo))


def _can_on_table():
    return catalog.load('scenarios', 'can_on_table')


def test_can_on_table_uses_fm_skills_geometry():
    entry = _can_on_table()
    objects = {obj['name']: obj for obj in entry['objects']}
    table, paper, can = objects['table'], objects['paper'], objects['can']
    # specs.yaml: table_z 0.207, can 0.033 x 0.135, A4 paper.
    assert table['pose']['xyz'][2] + table['size'][2] / 2 == pytest.approx(0.207)
    assert can['size'] == [0.033, 0.135]
    assert paper['size'][:2] == [0.297, 0.210]
    assert entry['success']['object'] == 'can'
    (target,) = entry['regions']
    assert target['pose']['xyz'][:2] == paper['pose']['xyz'][:2]


def test_table_only_matches_can_on_table():
    scene = {obj['name']: obj for obj in _can_on_table()['objects']}
    del scene['can']
    table_only = catalog.load('scenarios', 'table_only')
    assert {obj['name']: obj for obj in table_only['objects']} == scene
    assert 'success' not in table_only


@pytest.mark.parametrize('change, match', [
    (lambda s: s['objects'][2].update(size=[0.033]), 'is too short'),
    (lambda s: s['objects'][0].update(spawn={'x': [0.0, 0.1]}), 'should not be valid'),
    (lambda s: s['objects'][2].pop('mass'), "'mass' is a required property"),
    (lambda s: s['objects'][2].update(shape='mesh'), "'mesh' is not one of"),
    (lambda s: s['objects'][2].update(name='../can'), 'does not match'),
    (lambda s: s['objects'][2]['spawn'].update(roll=[0.0, 0.1]), 'Additional properties'),
    (lambda s: s['success'].update(type='upright'), "'upright' is not one of"),
    (lambda s: s['success'].update(region='shelf'), "success region 'shelf'"),
])
def test_malformed_scenario_is_rejected(extra_prefix, change, match):
    entry = _can_on_table()
    del entry['package']
    entry['name'] = 'broken'
    change(entry)
    extra_prefix('broken', entry, kind='scenarios')
    with pytest.raises(catalog.CatalogError, match=match):
        catalog.load('scenarios', 'broken')


def _usd_object(**changes):
    return {
        'name': 'prop', 'usd': 'pkg/simready_usd/prop.usd', 'shape': 'box', 'size': [0.1, 0.1, 0.1],
        'color': [1.0, 0.0, 0.0, 1.0], 'pose': {'xyz': [0.0, 0.0, 0.0]}, **changes,
    }


def _object_problem(obj):
    schema = {**catalog.load_schema(), '$ref': '#/definitions/scenario_object'}
    try:
        jsonschema.validate(obj, schema)
    except jsonschema.ValidationError as error:
        return error.message
    return None


def test_a_usd_object_needs_no_mass_or_friction_of_its_own():
    assert _object_problem(_usd_object()) is None


@pytest.mark.parametrize('field,value', [('mass', 0.1), ('friction', 1.0), ('static', True)])
def test_a_usd_object_refuses_what_its_asset_decides(field, value):
    assert _object_problem(_usd_object(**{field: value})) is not None


@pytest.mark.parametrize('usd', [
    '../secret.usd', 'a/../../secret.usd', '/etc/secret.usd', 'a/.hidden.usd', 'prop.txt', 'a b.usd',
])
def test_a_usd_path_must_stay_inside_the_assets(usd):
    assert _object_problem(_usd_object(usd=usd)) is not None


@pytest.mark.parametrize('field,missing', [('mass', "'mass'"), ('friction', "'friction'")])
def test_a_shape_object_still_needs_its_physics(field, missing):
    obj = _usd_object(mass=0.1, friction=1.0)
    del obj['usd'], obj[field]
    assert missing in _object_problem(obj)


@pytest.mark.parametrize('profile,valid', [('fm_openarm_v2', True), (None, True), ('', False)])
def test_schema_robot_profile_is_a_name_or_null(profile, valid):
    robot = dict(catalog.load('robots', 'so101'), robot_profile=profile)
    schema = {**catalog.load_schema(), '$ref': '#/definitions/robot'}
    if valid:
        jsonschema.validate(robot, schema)
    else:
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(robot, schema)


CONTROL = {
    'name': 'wam',
    'blocks': [
        {'id': 'perception', 'kind': 'perception', 'variant': 'YOLO-seg',
         'live': '/detections', 'sim': {'variant': 'ground truth'}},
        {'id': 'task', 'kind': 'state_machine', 'variant': 'pick and place',
         'states': ['IDLE', 'PICK'],
         'source': {'topic': '/task/state', 'type': 'std_msgs/msg/String', 'field': 'data'}},
    ],
    'edges': [{'from': 'perception', 'to': 'task'}],
}


def _write_control(tmp_path, descriptor):
    path = tmp_path / 'share/fake_pkg/control/wam.yaml'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(descriptor))
    package_marker = tmp_path / 'share/ament_index/resource_index/packages/fake_pkg'
    package_marker.parent.mkdir(parents=True, exist_ok=True)
    package_marker.touch()
    return path


def test_controller_carries_family_and_its_control_descriptor(extra_prefix, tmp_path, capsys):
    _write_control(tmp_path, CONTROL)
    extra_prefix('wam', dict(PLACEHOLDER, family='wam_family',
                             control={'package': 'fake_pkg', 'path': 'control/wam.yaml'}))
    entry = catalog.load('controllers', 'wam')
    assert entry['family'] == 'wam_family'
    assert entry['control_descriptor'] == CONTROL
    assert catalog.main(['--json']) == 0
    output = json.loads(capsys.readouterr().out)
    jsonschema.validate(output, catalog.load_schema())
    (listed,) = [c for c in output['controllers'] if c['name'] == 'wam']
    assert listed['control'] == {'package': 'fake_pkg', 'path': 'control/wam.yaml'}
    assert listed['control_descriptor']['blocks'][0]['variant'] == 'YOLO-seg'


def test_control_file_must_name_its_entry(extra_prefix, tmp_path):
    _write_control(tmp_path, dict(CONTROL, name='other'))
    extra_prefix('wam', dict(PLACEHOLDER, control={'package': 'fake_pkg',
                                                   'path': 'control/wam.yaml'}))
    with pytest.raises(catalog.CatalogError, match="control file names itself 'other'"):
        catalog.load('controllers', 'wam')


def _with_block(**changes):
    blocks = [dict(CONTROL['blocks'][0], **changes), CONTROL['blocks'][1]]
    return dict(CONTROL, blocks=blocks)


@pytest.mark.parametrize('descriptor, match', [
    (_with_block(kind='planner'), 'is not one of'),
    (dict(CONTROL, edges=[{'from': 'perception', 'to': 'skill'}]), "missing block 'skill'"),
    (dict(CONTROL, blocks=[CONTROL['blocks'][0],
                           dict(CONTROL['blocks'][1], states=[])]), 'is too short'),
    (dict(CONTROL, blocks=[CONTROL['blocks'][0],
                           {k: v for k, v in CONTROL['blocks'][1].items() if k != 'states'}]),
     "'states' is a required property"),
    (_with_block(states=['IDLE']), 'should not be valid under'),
    (_with_block(source=CONTROL['blocks'][1]['source']), 'should not be valid under'),
    (_with_block(id='task'), 'duplicate block ids task'),
    (_with_block(live='detections'), 'does not match'),
], ids=['unknown_kind', 'edge_to_missing_block', 'empty_states', 'no_states',
        'states_outside_state_machine', 'source_outside_state_machine', 'duplicate_id', 'relative_topic'])
def test_bad_control_descriptor_is_rejected(tmp_path, descriptor, match):
    path = _write_control(tmp_path, descriptor)
    with pytest.raises(catalog.CatalogError, match=match):
        catalog.load_control(str(path))


def test_inline_control_descriptor_is_refused(extra_prefix):
    extra_prefix('wam', dict(PLACEHOLDER, control_descriptor=CONTROL))
    with pytest.raises(catalog.CatalogError, match='not a control_descriptor'):
        catalog.load('controllers', 'wam')


@pytest.mark.parametrize('content', [None, 'blocks: [unclosed'], ids=['missing', 'malformed'])
def test_unreadable_control_file_is_a_catalog_error(extra_prefix, tmp_path, content):
    path = _write_control(tmp_path, CONTROL)
    if content is None:
        path.unlink()
    else:
        path.write_text(content)
    extra_prefix('wam', dict(PLACEHOLDER, control={'package': 'fake_pkg',
                                                   'path': 'control/wam.yaml'}))
    with pytest.raises(catalog.CatalogError,
                       match='control fake_pkg/control/wam.yaml: unreadable') as raised:
        catalog.load('controllers', 'wam')
    assert str(tmp_path) not in str(raised.value)


def test_check_control_validates_files(tmp_path, capsys):
    good = _write_control(tmp_path, CONTROL)
    assert catalog.main(['--check-control', str(good)]) == 0
    bad = tmp_path / 'bad.yaml'
    bad.write_text(yaml.safe_dump(dict(CONTROL, edges=[{'from': 'x', 'to': 'task'}])))
    assert catalog.main(['--check-control', str(good), str(bad)]) == 1
    assert "missing block 'x'" in capsys.readouterr().err
