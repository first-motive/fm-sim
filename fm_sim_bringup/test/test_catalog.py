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
    assert {'empty', 'can_on_table'} <= set(catalog.names('scenarios'))
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


def test_malformed_entry_is_rejected(extra_prefix):
    broken = dict(PLACEHOLDER, name='broken')
    del broken['activate']
    extra_prefix('broken', broken)
    with pytest.raises(catalog.CatalogError, match="'activate' is a required property"):
        catalog.load('controllers', 'broken')


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


@pytest.mark.parametrize('profile,valid', [('fm_openarm_v2', True), (None, True), ('', False)])
def test_schema_robot_profile_is_a_name_or_null(profile, valid):
    robot = dict(catalog.load('robots', 'so101'), robot_profile=profile)
    schema = {**catalog.load_schema(), '$ref': '#/definitions/robot'}
    if valid:
        jsonschema.validate(robot, schema)
    else:
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate(robot, schema)
