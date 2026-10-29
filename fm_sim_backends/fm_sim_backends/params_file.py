"""The ``params_file`` launch contract: a run's overrides, appended last, under a guard.

A launch that owns tunables declares ``params_file`` (``declare_params_file``).
When it is set, each node the launch starts gets it after its own parameters,
then a guard file that re-asserts what the launch owns (``with_params_file``):

    parameters = [node yaml, launch dict, params_file, guard file]

Both files are keyed by the node's fully qualified name. Order alone does not
decide precedence: launch_ros writes a dict parameter under ``/**``, and rcl
applies ``/**`` entries before node-keyed ones whatever the file order, so a
node-keyed params file beats every launch dict. Only a node-keyed file placed
after it can take a key back, hence a guard file and not a guard dict. A key
written relative (``servo_node``) is not the same key as ``/servo_node`` either.

``fm_sim_backends.params_check`` resolves a node's parameters through rcl, to
test both rules.
"""

import os
import tempfile

from launch.actions import DeclareLaunchArgument
import yaml


def declare_params_file():
    """Return the ``params_file`` launch argument every launch that owns tunables declares."""
    return DeclareLaunchArgument(
        'params_file', default_value='',
        description='Optional ROS params file appended last to every node, so its values win.',
    )


def with_params_file(parameters, params_file, guard_file=None):
    """Return ``parameters`` with ``params_file`` then ``guard_file`` appended, when set."""
    if not params_file:
        return list(parameters)
    return [*parameters, params_file, *([guard_file] if guard_file else [])]


def write_guard(directory, guards):
    """Write ``guards`` (``{fully qualified node: {param: value}}``) as a params file; return it.

    The file goes in ``directory``, which the caller owns and removes.
    """
    for node in guards:
        if not node.startswith('/'):
            raise ValueError(f'guard node {node!r} must be fully qualified')
    handle, path = tempfile.mkstemp(dir=directory, prefix='guard_', suffix='.yaml')
    with os.fdopen(handle, 'w') as guard_file:
        yaml.safe_dump(
            {node: {'ros__parameters': values} for node, values in guards.items()}, guard_file
        )
    return path
