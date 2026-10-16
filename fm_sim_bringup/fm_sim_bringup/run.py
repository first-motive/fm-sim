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


def _activated_roles(robot, controller):
    """Return ``(controller_roles, launch_roles)`` names ``controller`` brings up.

    A role is either a controller role (names in ``controller_roles``) or a launch
    role (a node in ``launch_roles``, such as MoveIt Servo). A launch role's
    ``requires`` roles are activated with it, so the controller entry need not
    repeat them. A role listed twice starts once. Fails on a role the robot does
    not have.
    """
    roles = robot['controller_roles']
    launches = robot.get('launch_roles', {})
    clash = sorted(set(roles) & set(launches))
    if clash:
        raise catalog.CatalogError(
            f'robot {robot["name"]!r} names {", ".join(clash)} as both a controller '
            'role and a launch role'
        )
    unknown = [
        role for role in controller['activate'] if role not in roles and role not in launches
    ]
    if unknown:
        raise catalog.CatalogError(
            f'controller {controller["name"]!r} activates {", ".join(unknown)}, which '
            f'robot {robot["name"]!r} does not have; its roles: '
            f'{", ".join(sorted([*roles, *launches]))}'
        )
    active_roles, launch_roles = ['broadcasters'], []
    for role in controller['activate']:
        if role in launches:
            if role in launch_roles:
                continue
            launch_roles.append(role)
            needed = launches[role]['requires']
        else:
            needed = [role]
        missing = [name for name in needed if name not in roles]
        if missing:
            raise catalog.CatalogError(
                f'robot {robot["name"]!r} launch role {role!r} requires '
                f'{", ".join(missing)}, which is not a controller role'
            )
        active_roles.extend(name for name in needed if name not in active_roles)
    return active_roles, launch_roles


def controller_sets(robot, controller):
    """Return ``(active, inactive)`` controller names for ``controller`` on ``robot``.

    Broadcasters always start active, plus every role the controller activates
    (and every role its launch roles require); the robot's other controllers load
    inactive so a controller can switch to them.
    """
    roles = robot['controller_roles']
    active_roles, _ = _activated_roles(robot, controller)
    active = [name for role in active_roles for name in roles[role]]
    inactive = [
        name for role, names in roles.items() if role not in active_roles for name in names
    ]
    return active, inactive


def role_launches(robot, controller):
    """Return the robot's ``launch_roles`` entries ``controller`` activates, in order."""
    _, launch_roles = _activated_roles(robot, controller)
    return [robot['launch_roles'][role] for role in launch_roles]


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
