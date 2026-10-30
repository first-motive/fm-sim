"""The sim catalog: backends, robots, scenarios and controllers from the ament index.

Any package registers an entry by installing two files:

- ``share/ament_index/resource_index/fm_sim.<kind>/<name>``, whose content is the
  entry yaml's path relative to ``share/`` (``<package>/catalog/<kind>/<name>.yaml``);
- that entry yaml, shaped by ``schema/catalog.schema.json``.

An entry that names ``requires_packages`` is listed as unavailable, and refused by
``require_available``, while any of those packages is not installed: a robot whose
description ships only in a private image, say.

Any entry may declare ``params``: tunables a run can override, checked on load
by ``fm_sim_bringup.params``.

A controller entry may name a ``control`` file: the descriptor that draws its
pipeline. The loader validates it and adds it as ``control_descriptor``.

``<kind>`` is one of ``KINDS``. The ``fm_sim.`` prefix keeps the resource types
apart from any other package's. ``python -m fm_sim_bringup.catalog --json`` (or
``ros2 run fm_sim_bringup catalog --json``) prints every entry;
``--check-control FILE...`` validates descriptor files on their own.
"""

import argparse
from collections import Counter
import json
import os
import re
import sys

from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import get_resource, get_resources, has_resource
from fm_sim_bringup import params
from fm_sim_scenarios import scenario
import jsonschema
import yaml

KINDS = ('backends', 'robots', 'scenarios', 'controllers')
RESOURCE_PREFIX = 'fm_sim.'
SCHEMA_VERSION = 1
# Same rule as the schema's relative_path: no leading slash, no '..' segment.
_RELATIVE_PATH = re.compile(r'^(?!/)(?!(.*/)?\.\.(/|$)).+$')
# Entry kind -> its definition in the schema.
_DEFINITIONS = {
    'backends': 'backend',
    'robots': 'robot',
    'scenarios': 'scenario',
    'controllers': 'controller',
}
# Every name a run sets: run.launch.py's arguments and what it passes the backend
# and controller launches. Launch configurations are one shared namespace, so a
# controller's launch reads these even where it is not handed them, and a
# launch_args entry of the same name would have it disagree with the run.
CONTROLLER_LAUNCH_OWNED = frozenset((
    'backend', 'robot', 'scenario', 'controller', 'use_sim', 'seed', 'timeout_s',
    'params_file', 'overrides_file', 'robot_profile', 'robot_description',
    'controllers_file', 'world', 'cameras_file', 'mesh_roots',
))


class CatalogError(RuntimeError):
    """An unknown, malformed or unavailable catalog entry."""


def schema_path():
    return os.path.join(
        get_package_share_directory('fm_sim_bringup'), 'schema', 'catalog.schema.json'
    )


def load_schema():
    with open(schema_path()) as schema_file:
        return json.load(schema_file)


def names(kind):
    """Return the sorted entry names registered for ``kind``."""
    return sorted(get_resources(RESOURCE_PREFIX + kind))


def _confined(base, relative_path, what):
    """Return ``base/relative_path``, refusing a path that leaves ``base``.

    The check is on the path as written, not on symlinks resolved: a
    ``--symlink-install`` share links every file back to its source tree.
    """
    if not _RELATIVE_PATH.match(relative_path):
        raise CatalogError(f'{what}: {relative_path!r} must be relative with no ".." segment')
    base = os.path.normpath(base)
    path = os.path.normpath(os.path.join(base, relative_path))
    if os.path.commonpath([path, base]) != base:
        raise CatalogError(f'{what}: {relative_path!r} resolves outside {base}')
    return path


def load(kind, name, schema=None, registered=None):
    """Return the entry ``name`` of ``kind``, validated, plus its ``package``.

    Raises CatalogError naming the valid entries when ``name`` is not registered.
    ``registered`` and ``schema`` let a caller loading many entries scan once.
    """
    registered = registered if registered is not None else names(kind)
    if name not in registered:
        raise CatalogError(
            f'unknown {kind[:-1]} {name!r}; registered: {", ".join(registered) or "none"}'
        )
    relative_path, prefix = get_resource(RESOURCE_PREFIX + kind, name)
    relative_path = relative_path.strip()
    what = f'{kind[:-1]} {name!r} marker'
    if not _RELATIVE_PATH.match(relative_path) or '/' not in relative_path:
        raise CatalogError(f'{what}: {relative_path!r} is not <package>/<path>')
    package, path_in_package = relative_path.split('/', 1)
    entry_path = _confined(os.path.join(prefix, 'share', package), path_in_package, what)
    with open(entry_path) as entry_file:
        entry = yaml.safe_load(entry_file)
    entry['package'] = package
    schema = schema or load_schema()
    validate(kind, entry, schema)
    if entry['name'] != name:
        raise CatalogError(
            f'{kind[:-1]} {name!r}: its yaml names itself {entry["name"]!r}'
        )
    if 'control_descriptor' in entry:
        raise CatalogError(f'controller {name!r}: name a control file, not a control_descriptor')
    if 'control' in entry:
        reference = entry['control']
        descriptor = load_control(
            share_file(reference), schema, f'{reference["package"]}/{reference["path"]}'
        )
        if descriptor['name'] != name:
            raise CatalogError(
                f'controller {name!r}: its control file names itself {descriptor["name"]!r}'
            )
        entry['control_descriptor'] = descriptor
    missing = [
        package for package in entry.get('requires_packages', [])
        if not has_resource('packages', package)
    ]
    if missing:
        entry['available'] = False
        entry['missing_packages'] = missing
    return entry


def validate(kind, entry, schema=None):
    """Check one entry of ``kind`` against the schema and what the schema cannot say.

    For a scenario: unique names, ranges and success references. For a
    controller: registered ``scenarios`` and no ``launch_args`` name in
    ``CONTROLLER_LAUNCH_OWNED``. For every entry: its ``params``
    (``fm_sim_bringup.params.check_params``). Run again on an entry a run's
    overrides changed.
    """
    name = entry.get('name')
    try:
        jsonschema.validate(
            entry, {**(schema or load_schema()), '$ref': f'#/definitions/{_DEFINITIONS[kind]}'}
        )
    except jsonschema.ValidationError as error:
        where = '/'.join(map(str, error.absolute_path))
        at = f' (at {where})' if where else ''
        raise CatalogError(f'{kind[:-1]} {name!r}: {error.message}{at}') from error
    if kind == 'controllers':
        # A misspelt scenario would leave the controller refused on every scenario.
        unknown = [s for s in entry.get('scenarios', []) if s not in names('scenarios')]
        if unknown:
            raise CatalogError(f'controller {name!r}: unknown scenarios {", ".join(unknown)}')
        owned = sorted(CONTROLLER_LAUNCH_OWNED & set(entry.get('launch_args', {})))
        if owned:
            raise CatalogError(
                f'controller {name!r}: launch_args may not set {", ".join(owned)}; '
                'run.launch.py owns them'
            )
    try:
        if kind == 'scenarios':
            scenario.check(entry)
        params.check_params(entry)
    except (scenario.ScenarioError, params.ParamError) as error:
        raise CatalogError(f'{kind[:-1]} {name!r}: {error}') from error


def require_available(kind, name):
    """Return ``load(kind, name)``, refusing an entry that is not available.

    That is one marked ``available: false``, or one whose ``requires_packages`` are
    not all installed; the error names the missing packages.
    """
    entry = load(kind, name)
    if not entry['available']:
        raise CatalogError(f'{kind[:-1]} {name!r} is listed but {_unavailable(entry)}')
    return entry


def _unavailable(entry):
    """Say why an unavailable entry cannot run."""
    if entry.get('missing_packages'):
        return f'needs {", ".join(entry["missing_packages"])}, which is not installed'
    return 'not available yet'


def load_control(path, schema=None, label=None):
    """Return the control descriptor at ``path``, validated.

    What the schema cannot say: block ids are unique and every edge names one.
    Errors name ``label`` (default ``path``), so a catalog error shows the
    share-relative reference rather than an install path.
    """
    what = f'control {label or path}'
    try:
        with open(path) as control_file:
            descriptor = yaml.safe_load(control_file)
    except (OSError, yaml.YAMLError) as error:
        raise CatalogError(f'{what}: unreadable ({type(error).__name__})') from error
    schema = schema or load_schema()
    try:
        jsonschema.validate(descriptor, {**schema, '$ref': '#/definitions/control_descriptor'})
    except jsonschema.ValidationError as error:
        raise CatalogError(f'{what}: {error.message}') from error
    ids = Counter(block['id'] for block in descriptor['blocks'])
    duplicates = sorted(block_id for block_id, count in ids.items() if count > 1)
    if duplicates:
        raise CatalogError(f'{what}: duplicate block ids {", ".join(duplicates)}')
    for edge in descriptor['edges']:
        for end in (edge['from'], edge['to']):
            if end not in ids:
                raise CatalogError(f'{what}: edge names missing block {end!r}')
    return descriptor


def share_file(reference):
    """Absolute path of a ``{package, path}`` reference, confined to that share."""
    return _confined(
        get_package_share_directory(reference['package']),
        reference['path'],
        f'{reference["package"]} share file',
    )


def catalog():
    """Return every registered entry, shaped by the schema's top level."""
    schema = load_schema()
    result = {'schema_version': SCHEMA_VERSION}
    for kind in KINDS:
        registered = names(kind)
        result[kind] = [load(kind, name, schema, registered) for name in registered]
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description='List the fm-sim catalog.')
    parser.add_argument('--json', action='store_true', help='print the catalog as JSON')
    parser.add_argument('--check-control', nargs='+', metavar='FILE',
                        help='validate control descriptor files instead')
    args = parser.parse_args(argv)
    if args.check_control:
        schema = load_schema()
        try:
            for path in args.check_control:
                load_control(path, schema)
        except CatalogError as error:
            print(f'catalog: {error}', file=sys.stderr)
            return 1
        return 0
    try:
        entries = catalog()
    except CatalogError as error:
        print(f'catalog: {error}', file=sys.stderr)
        return 1
    if args.json:
        json.dump(entries, sys.stdout, indent=2, sort_keys=True)
        print()
        return 0
    for kind in KINDS:
        print(f'{kind}:')
        for entry in entries[kind]:
            flag = '' if entry['available'] else f'  ({_unavailable(entry)})'
            print(f'  {entry["name"]:<12} {entry["description"]}{flag}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
