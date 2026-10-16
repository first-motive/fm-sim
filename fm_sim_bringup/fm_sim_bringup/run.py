"""Turn four catalog choices into what run.launch.py starts.

Kept apart from the launch file so the rules (description args, which controllers
start active) unit-test without a launch context, an engine or fm-robot.
"""

from fm_sim_bringup import catalog


def description_args(backend, robot, mjcf_path, controllers_file):
    """Return the xacro args for ``robot`` on ``backend``.

    The robot's own args come first; the backend adds the ones that select its
    hardware plugin, with ``{mjcf_path}`` and ``{controllers_file}`` filled in.
    """
    args = dict(robot['description_args'])
    for key, value in backend['description_args'].items():
        args[key] = value.format(mjcf_path=mjcf_path, controllers_file=controllers_file)
    return args


def controller_sets(robot, controller):
    """Return ``(active, inactive)`` controller names for ``controller`` on ``robot``.

    Broadcasters always start active, plus every role the controller activates;
    the robot's other controllers load inactive so a controller can switch to them.
    """
    roles = robot['controller_roles']
    unknown = [role for role in controller['activate'] if role not in roles]
    if unknown:
        raise catalog.CatalogError(
            f'controller {controller["name"]!r} activates {", ".join(unknown)}, which '
            f'robot {robot["name"]!r} does not have; its roles: {", ".join(sorted(roles))}'
        )
    active_roles = ['broadcasters', *controller['activate']]
    active = [name for role in active_roles for name in roles[role]]
    inactive = [
        name for role, names in roles.items() if role not in active_roles for name in names
    ]
    return active, inactive


def resolve(backend, robot, scenario, controller):
    """Load and check the four entries.

    Refuses unknown or unavailable entries, and a robot on a backend it does not
    list in ``supported_backends``.
    """
    entries = {
        'backend': catalog.require_available('backends', backend),
        'robot': catalog.require_available('robots', robot),
        'scenario': catalog.require_available('scenarios', scenario),
        'controller': catalog.require_available('controllers', controller),
    }
    supported = entries['robot']['supported_backends']
    if backend not in supported:
        raise catalog.CatalogError(
            f'robot {robot!r} is not supported on backend {backend!r} yet; '
            f'supported: {", ".join(supported)}'
        )
    return entries
