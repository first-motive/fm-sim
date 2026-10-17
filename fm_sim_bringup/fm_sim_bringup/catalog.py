"""The sim catalog: backends, robots, scenarios and controllers from the ament index.

Any package registers an entry by installing two files:

- ``share/ament_index/resource_index/fm_sim.<kind>/<name>``, whose content is the
  entry yaml's path relative to ``share/`` (``<package>/catalog/<kind>/<name>.yaml``);
- that entry yaml, shaped by ``schema/catalog.schema.json``.

``<kind>`` is one of ``KINDS``. The ``fm_sim.`` prefix keeps the resource types
apart from any other package's. ``python -m fm_sim_bringup.catalog --json`` (or
``ros2 run fm_sim_bringup catalog --json``) prints every entry.
"""

import argparse
import json
import os
import re
import sys

from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import get_resource, get_resources
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
    try:
        jsonschema.validate(
            entry, {**schema, '$ref': f'#/definitions/{_DEFINITIONS[kind]}'}
        )
    except jsonschema.ValidationError as error:
        raise CatalogError(f'{kind[:-1]} {name!r}: {error.message}') from error
    if kind == 'scenarios':
        # What the schema cannot say: unique names, ranges, success references.
        try:
            scenario.check(entry)
        except scenario.ScenarioError as error:
            raise CatalogError(f'scenario {name!r}: {error}') from error
    if entry['name'] != name:
        raise CatalogError(
            f'{kind[:-1]} {name!r}: its yaml names itself {entry["name"]!r}'
        )
    return entry


def require_available(kind, name):
    """Return ``load(kind, name)``, refusing an entry marked ``available: false``."""
    entry = load(kind, name)
    if not entry['available']:
        raise CatalogError(f'{kind[:-1]} {name!r} is listed but not available yet')
    return entry


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
    args = parser.parse_args(argv)
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
            flag = '' if entry['available'] else '  (not available)'
            print(f'  {entry["name"]:<12} {entry["description"]}{flag}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
