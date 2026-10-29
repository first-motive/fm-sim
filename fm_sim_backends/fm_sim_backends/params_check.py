"""Test helpers for the ``params_file`` contract: what a launched node's parameters resolve to.

``assert_params_file_wins`` and ``assert_guard_holds`` take a launch_ros
``Node`` action, as a launch builds it, and resolve its parameters the way the
started process would: launch_ros expands them into ``--params-file``
arguments (a dict into a ``/**`` file), and a real rclpy node under the
action's name loads them through rcl. So a test sees rcl's precedence, not the
list's order.

Another package's launch test imports these where fm_sim_backends is
installed, or copies this file: it needs only rclpy and launch_ros. Usage::

    node = next(a for a in actions if isinstance(a, Node) and a.node_executable == 'x')
    assert_params_file_wins(node, context, {'max_speed': 1.5})
    assert_guard_holds(node, context, {'use_sim_time': True})
"""

from launch_ros.actions import Node
import rclpy


def resolved_parameters(node, context, names, node_name=None):
    """Return ``{name: value}`` for ``names`` as the node ``node`` starts would hold them.

    A name no file sets is absent. ``node`` is a launch_ros ``Node`` action;
    ``context`` is the launch context it was built in. ``node_name`` is the
    fully qualified name the process gives its node when the action names none
    (naming one remaps every node in the process, so an engine is left unnamed).
    """
    # The expanded arguments are private to launch_ros (Jazzy); reading them is
    # what keeps this the same expansion the launched process gets.
    node._perform_substitutions(context)
    files = [arg for arg, is_file in node._Node__expanded_parameter_arguments if is_file]
    namespace, _, name = (node_name or node.node_name).rpartition('/')
    # A node started with no namespace runs in the root one.
    namespace = namespace.replace(Node.UNSPECIFIED_NODE_NAMESPACE, '')
    rcl = rclpy.Context()
    rclpy.init(context=rcl)
    try:
        probe = rclpy.create_node(
            name, namespace=namespace or '/', context=rcl, use_global_arguments=False,
            cli_args=['--ros-args', *[arg for path in files for arg in ('--params-file', path)]],
            automatically_declare_parameters_from_overrides=True,
        )
        try:
            return {
                key: probe.get_parameter(key).value for key in names if probe.has_parameter(key)
            }
        finally:
            probe.destroy_node()
    finally:
        rclpy.shutdown(context=rcl)


def assert_params_file_wins(node, context, expected, node_name=None):
    """Assert each param in ``expected`` resolves to the override the params file gives it."""
    resolved = resolved_parameters(node, context, expected, node_name)
    assert resolved == expected, f'params_file lost; resolved {resolved}'


def assert_guard_holds(node, context, guarded, node_name=None):
    """Assert each launch-owned param in ``guarded`` keeps the launch's value."""
    resolved = resolved_parameters(node, context, guarded, node_name)
    assert resolved == guarded, f'a guarded param changed; resolved {resolved}'
