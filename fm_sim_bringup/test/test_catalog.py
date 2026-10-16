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
    """Return a function that registers a controller entry in a throwaway prefix."""
    monkeypatch.setenv(
        'AMENT_PREFIX_PATH', f'{tmp_path}{os.pathsep}{os.environ["AMENT_PREFIX_PATH"]}'
    )

    def register(name, entry, marker_content=None):
        marker = tmp_path / 'share/ament_index/resource_index/fm_sim.controllers' / name
        marker.parent.mkdir(parents=True, exist_ok=True)
        relative = f'fake_pkg/catalog/controllers/{name}.yaml'
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
    assert 'openarm' in catalog.names('robots')
    assert 'empty' in catalog.names('scenarios')
    assert 'idle' in catalog.names('controllers')


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
    (openarm,) = [entry for entry in output['robots'] if entry['name'] == 'openarm']
    assert openarm['supported_backends'] == ['mujoco', 'gazebo', 'isaac']


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


def _openarm_with_servo(servo):
    robot = catalog.load('robots', 'openarm')
    robot['launch_roles'] = {'servo': servo}
    return robot


def _validate_robot(robot):
    schema = catalog.load_schema()
    jsonschema.validate(robot, {**schema, '$ref': '#/definitions/robot'})


def test_schema_accepts_the_openarm_servo_role():
    _validate_robot(catalog.load('robots', 'openarm'))


@pytest.mark.parametrize('servo', [
    {'requires': ['arm_trajectory']},
    {'requires': [], 'launch': {'package': 'p', 'path': 'x.launch.py'}},
    {'requires': ['arm_trajectory'], 'launch': {'package': 'p', 'path': 'x.launch.py'},
     'command_type': 'twist'},
], ids=['no_launch', 'empty_requires', 'unknown_key'])
def test_schema_rejects_a_malformed_launch_role(servo):
    with pytest.raises(jsonschema.ValidationError):
        _validate_robot(_openarm_with_servo(servo))
