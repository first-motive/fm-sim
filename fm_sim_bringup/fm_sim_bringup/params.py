"""Run params: the tunables a catalog entry declares, and the overrides a run takes.

An entry's ``params`` maps a tunable name to its declaration (the schema's
``param``): a type, a default, bounds, and a ``target`` saying where an override
goes. A caller names one as ``<dimension>.<name>``, such as
``controller.max_joint_speed``.

What the schema cannot say is checked here, on every load: ``min <= max``
(integers for an int tunable), the default satisfies the tunable's own type and
bounds, and an ``entry`` target's path holds the default, so the app shows the
value the run would use, and ends on a value, not an identity field such as
``name``. An ``mjcf`` target must say it applies on mujoco only, the one
backend that loads the MJCF. No ``ros_param`` target may name a launch-owned
parameter (``guarded``), nor one another target of the entry also sets.

The sim agent keeps a mirror of ``check_value`` (it runs on the host, outside
the image); both suites check theirs against ``test/param_cases.json``.
"""

import math

#: The backends that load the robot's MJCF, so the only ones an mjcf target reaches.
MJCF_BACKENDS = ('mujoco',)
#: ROS param names no tunable may target on any node: safety gates, topics,
#: identity or path params, and the robot's own description. A param is
#: guarded when any segment of its dotted name is one of these or ends in
#: ``_topic``. MoveIt's ``robot_description_planning`` is not here: its joint
#: limits are what a run tunes. Launches re-assert theirs after the params
#: file too (a guard file), so this is the first wall, not the only one.
GUARDED_PARAMS = frozenset((
    'use_sim_time', 'sim_only', 'engage_on_start',
    'command', 'observation', 'enable', 'status',
    'checkpoint', 'policy_type', 'robot_profile',
    'robot_description', 'robot_description_semantic', 'robot_description_kinematics',
))
#: The scenario node run.launch.py starts, and the params it owns there.
SCENARIO_NODE = 'sim_scenario'
SCENARIO_OWNED = frozenset(('scenario', 'seed', 'backend', 'timeout_s', 'use_sim_time'))
#: Every sim_camera node is ``<prefix><role>``; the params run.launch.py owns there.
CAMERA_NODE_PREFIX = 'sim_camera_'
CAMERA_OWNED = frozenset(('role', 'cameras_file', 'render_topic', 'use_sim_time'))
#: Entry fields that say what an object is or which one is meant, never how it behaves.
IDENTITY_FIELDS = frozenset(('name', 'shape', 'static', 'type', 'object', 'region'))


class ParamError(ValueError):
    """A tunable declaration or an override value that is refused."""


def guarded(node, param):
    """Whether ``param`` on ``node`` (fully qualified) is one no tunable may set."""
    segments = param.split('.')
    if any(
        segment in GUARDED_PARAMS or segment.endswith('_topic')
        for segment in segments
    ):
        return True
    name = node.rsplit('/', 1)[-1]
    if name == SCENARIO_NODE:
        return segments[0] in SCENARIO_OWNED
    return name.startswith(CAMERA_NODE_PREFIX) and segments[0] in CAMERA_OWNED


def _number(value, kind):
    """Whether ``value`` is a JSON number of ``kind``: ``True`` is neither; an int is a float.

    An int of any size is finite; only a float can be NaN or infinite, and
    ``math.isfinite`` would overflow on an int past the float range.
    """
    if isinstance(value, bool):
        return False
    if isinstance(value, int):
        return True
    return kind == 'float' and isinstance(value, float) and math.isfinite(value)


def check_value(param, value):
    """Return ``value`` normalised for ``param``, or raise ParamError.

    A float tunable's value comes back a float even when given as an int: ROS
    refuses an integer for a double parameter. The message names what the
    value must be, never the value.
    """
    kind = param['type']
    if kind == 'bool':
        if not isinstance(value, bool):
            raise ParamError('must be true or false')
        return value
    if kind == 'enum':
        if not isinstance(value, str) or value not in param['enum']:
            raise ParamError(f'must be one of {", ".join(param["enum"])}')
        return value
    low, high = param['min'], param['max']
    if kind == 'array':
        items, length = param['items'], param['length']
        if (
            not isinstance(value, list) or len(value) != length
            or not all(_number(item, items) for item in value)
        ):
            raise ParamError(f'must be a list of {length} {items} values')
        if not all(low <= item <= high for item in value):
            raise ParamError(f'each element must be between {low} and {high}')
        if param.get('ordered') and any(a > b for a, b in zip(value, value[1:])):
            raise ParamError('must be in ascending order')
        return [float(item) if items == 'float' else item for item in value]
    if not _number(value, kind):
        raise ParamError('must be an integer' if kind == 'int' else 'must be a number')
    if not low <= value <= high:
        raise ParamError(f'must be between {low} and {high}')
    return float(value) if kind == 'float' else value


def _segment(node, segment):
    """``node[segment]``; in a list, the item whose ``name`` is ``segment``."""
    if isinstance(node, dict) and segment in node:
        return node[segment]
    if isinstance(node, list):
        named = [item for item in node if isinstance(item, dict) and item.get('name') == segment]
        if len(named) == 1:
            return named[0]
    raise ParamError(f'path segment {segment!r} names nothing')


def entry_value(entry, path):
    """The value at the dotted ``path`` in ``entry``, or raise ParamError."""
    node = entry
    for segment in path.split('.'):
        node = _segment(node, segment)
    return node


def set_entry_value(entry, path, value):
    """Set the value at the dotted ``path`` in ``entry``, which must already hold one."""
    *parents, leaf = path.split('.')
    node = entry
    for segment in parents:
        node = _segment(node, segment)
    if not isinstance(node, dict) or leaf not in node:
        raise ParamError(f'path segment {leaf!r} names nothing')
    node[leaf] = value


def overlapping(first, second):
    """Whether two ``(node, dotted param)`` targets set one ROS param, or one inside the other."""
    (node_a, param_a), (node_b, param_b) = first, second
    if node_a != node_b:
        return False
    a, b = param_a.split('.'), param_b.split('.')
    return a[:len(b)] == b or b[:len(a)] == a


def _check_bounds(param):
    if param['type'] not in ('float', 'int', 'array'):
        return
    integral = param['type'] == 'int' or param.get('items') == 'int'
    if integral and not all(
        isinstance(param[key], int) and not isinstance(param[key], bool) for key in ('min', 'max')
    ):
        raise ParamError('min and max must be integers')
    if param['min'] > param['max']:
        raise ParamError('min is above max')


def check_params(entry):
    """Check every tunable ``entry`` declares, beyond what the schema says."""
    seen = {}
    for name, param in entry.get('params', {}).items():
        try:
            _check_bounds(param)
            try:
                default = check_value(param, param['default'])
            except ParamError as error:
                raise ParamError(f'default {error}') from None
            target = param['target']
            if target['kind'] == 'entry':
                if target['path'].split('.')[-1] in IDENTITY_FIELDS:
                    raise ParamError(f'{target["path"]} names an identity field')
                if entry_value(entry, target['path']) != default:
                    raise ParamError(f'default is not the value at {target["path"]}')
            if target['kind'] == 'ros_param':
                key = (target['node'], target['param'])
                if guarded(*key):
                    raise ParamError(f'{target["param"]} is launch-owned and cannot be tuned')
                clash = next((other for other, at in seen.items() if overlapping(at, key)), None)
                if clash:
                    raise ParamError(f'sets the same ROS param as {clash!r}')
                seen[name] = key
            backends = param.get('backends')
            if 'mjcf' in target and not (backends and set(backends) <= set(MJCF_BACKENDS)):
                raise ParamError(
                    f'an mjcf target needs backends limited to {", ".join(MJCF_BACKENDS)}'
                )
        except ParamError as error:
            raise ParamError(f'param {name!r}: {error}') from None
