"""Turn four catalog choices into what run.launch.py starts.

Kept apart from the launch file so the rules (description args, which controllers
start active, the scenario world and its cameras) unit-test without a launch
context, an engine or fm-robot.
"""

import copy
from dataclasses import dataclass
import json
import math
import os

from fm_sim_bringup import catalog, params
from fm_sim_cameras import cameras_file, fisheye, mount
from fm_sim_scenarios import gazebo_world, isaac_stage, mujoco_world, scenario as sc

# Backend name -> the generator that builds its world from robot model + scenario +
# seed. A backend absent here runs the robot's own model and takes only scenarios
# without objects.
WORLD_GENERATORS = {
    'mujoco': mujoco_world.write,
    'gazebo': gazebo_world.write,
    'isaac': isaac_stage.write,
}
# Backends whose own launch finishes the generated world from the description
# and the robot's cameras: they also get ``world``, ``cameras_file``,
# ``mesh_roots`` and ``robot``.
STAGE_BACKENDS = ('gazebo', 'isaac')
# Backends whose generated world carries the robot's cameras: the generator module
# (``world_path``, ``body_names``, and ``write`` taking ``cameras``) and the topic
# its renderer publishes a camera's pinhole image on. The mujoco topic is the
# CameraPlugin's default for a camera named cam_<role> under the plugin key
# mujoco_cameras (fm_sim_backends config/mujoco_plugins.yaml).
RENDERED_CAMERAS = {'mujoco': (mujoco_world, '/mujoco_cameras/cam_{role}/color')}


@dataclass(frozen=True)
class CameraPlacement:
    """One robot camera on a rendering backend: what the world carries, what the node reads."""

    role: str
    cameras_file: str
    render_topic: str
    model: mujoco_world.Camera


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

    Refuses unknown or unavailable entries, a robot on a backend it does not list
    in ``supported_backends``, a controller with a launch on a robot with no
    ``robot_profile`` (that launch reads its names from the profile), and a
    controller on a scenario its ``scenarios`` list leaves out.
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
    if entries['controller']['launch'] and entries['robot']['robot_profile'] is None:
        raise catalog.CatalogError(
            f'controller {controller!r} needs a robot profile; robot {robot!r} has none yet'
        )
    scenarios = entries['controller'].get('scenarios')
    if scenarios is not None and scenario not in scenarios:
        raise catalog.CatalogError(
            f'controller {controller!r} runs only on scenarios {", ".join(scenarios)}; '
            f'not {scenario!r}'
        )
    return entries


# Run dimension -> its catalog kind.
_KINDS = {'backend': 'backends', 'robot': 'robots', 'scenario': 'scenarios',
          'controller': 'controllers'}


def load_overrides(path):
    """Return the overrides file at ``path`` (the agent's params.json); empty path is none."""
    if not path:
        return {}
    try:
        with open(path) as overrides_file:
            overrides = json.load(overrides_file)
    except (OSError, ValueError, RecursionError) as error:
        raise catalog.CatalogError(
            f'overrides_file is unreadable ({type(error).__name__})'
        ) from error
    if not isinstance(overrides, dict):
        raise catalog.CatalogError('overrides_file must hold a JSON object')
    return overrides


def apply_overrides(entries, overrides):
    """Return ``entries`` with ``overrides`` merged in, each checked against its tunable.

    ``overrides`` is keyed ``<dimension>.<name>``. An ``entry`` target is set in
    a copy of its entry, which is then checked again as the catalog checks it.
    A ``ros_param`` target is checked and left to the params file, a
    ``description`` one to ``description_overrides``. Refuses a
    key no entry declares, a value the tunable does not take, and a tunable
    whose ``backends`` leave out the run's backend.
    """
    patched = copy.deepcopy(entries)
    changed = set()
    for key, value in sorted(overrides.items()):
        dimension, _, name = str(key).partition('.')
        param = entries.get(dimension, {}).get('params', {}).get(name)
        if param is None:
            raise catalog.CatalogError(f'override {key!r} is not a declared tunable')
        backends = param.get('backends')
        if backends and entries['backend']['name'] not in backends:
            raise catalog.CatalogError(
                f'override {key!r} applies only on {", ".join(backends)}'
            )
        try:
            value = params.check_value(param, value)
            target = param['target']
            if target['kind'] == 'entry':
                params.set_entry_value(patched[dimension], target['path'], value)
                changed.add(dimension)
        except params.ParamError as error:
            raise catalog.CatalogError(f'override {key!r} {error}') from error
    for dimension in changed:
        # Without its params: their defaults are the entry as declared, not as overridden.
        entry = {k: v for k, v in patched[dimension].items() if k != 'params'}
        catalog.validate(_KINDS[dimension], entry)
    return patched


@dataclass(frozen=True)
class DescriptionOverrides:
    """What a run's overrides change in the robot description and its MJCF."""

    xacro_args: dict
    mjcf: tuple  # (element, name, attribute, value text), for mujoco_world.robot_with_patches


def _text(value):
    """A checked value as xacro or MJCF attribute text: lowercase bools, space-separated arrays."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, list):
        return ' '.join(_text(item) for item in value)
    return str(value)


def description_overrides(entries, overrides):
    """Return the ``DescriptionOverrides`` among ``overrides``.

    ``overrides`` were checked by ``apply_overrides`` against the same entries.
    """
    xacro_args, mjcf = {}, []
    for key, value in sorted(overrides.items()):
        dimension, _, name = key.partition('.')
        param = entries[dimension]['params'][name]
        target = param['target']
        if target['kind'] != 'description':
            continue
        text = _text(params.check_value(param, value))
        if 'xacro_arg' in target:
            xacro_args[target['xacro_arg']] = text
        else:
            patch = target['mjcf']
            mjcf.append((patch['element'], patch['name'], patch['attribute'], text))
    return DescriptionOverrides(xacro_args, tuple(mjcf))


def patched_mjcf(robot_mjcf, patches, out_dir):
    """Return the robot's MJCF with ``patches`` applied, written to ``out_dir``.

    With no patches it is ``robot_mjcf`` itself.

    Applied after fm_sim_models has compiled or found the model, so a patch
    sees the attributes the sim would otherwise run with.
    """
    if not patches:
        return robot_mjcf
    try:
        text = mujoco_world.robot_with_patches(robot_mjcf, patches)
    except sc.ScenarioError as error:
        raise catalog.CatalogError(f'description override: {error}') from error
    path = os.path.join(out_dir, 'robot_patched.xml')
    with open(path, 'w') as model_file:
        model_file.write(text)
    return path


def parse_seed(text):
    """Return the ``seed`` launch argument as an int, refusing anything not uint32."""
    try:
        return sc.check_seed(int(text))
    except (ValueError, sc.ScenarioError) as error:
        raise catalog.CatalogError(
            f'seed must be an integer in [0, {sc.SEED_MAX}], got {text!r}'
        ) from error


def parse_timeout(text):
    """Return the ``timeout_s`` launch argument: 0.0 when empty (the scenario's own).

    Anything else must be a finite positive number.
    """
    if text == '':
        return 0.0
    try:
        value = float(text)
    except ValueError:
        value = -1.0
    if not (value > 0.0 and math.isfinite(value)):
        raise catalog.CatalogError(f'timeout_s must be empty or a positive number, got {text!r}')
    return value


def _refuse_objects_without_generator(backend, scenario):
    if backend['name'] not in WORLD_GENERATORS and scenario['objects']:
        raise catalog.CatalogError(
            f'scenario {scenario["name"]!r} has objects, and backend {backend["name"]!r} '
            f'has no scenario generator yet; generators: {", ".join(sorted(WORLD_GENERATORS))}'
        )


def world_file(backend, scenario, seed, out_dir):
    """Return the world path the description must name before the world exists.

    On a backend whose world carries the robot's cameras (``RENDERED_CAMERAS``)
    the description names the world, and the cameras are placed from the
    description, so the path comes first. None elsewhere: the description then
    names the robot's own model. Refuses a scenario with objects on a backend
    that has no generator yet.
    """
    _refuse_objects_without_generator(backend, scenario)
    rendered = RENDERED_CAMERAS.get(backend['name'])
    return rendered[0].world_path(scenario, seed, out_dir) if rendered else None


def world_model(backend, robot, scenario, seed, robot_mjcf, out_dir, cameras=()):
    """Write the generated world for ``backend`` and return it, or None to run the robot's model.

    The world is written to ``out_dir``, which the caller owns and removes; on a
    ``RENDERED_CAMERAS`` backend at ``world_file(...)``, carrying ``cameras``
    (``sim_cameras(...)``). Refuses a scenario with objects on a backend that has
    no generator yet.
    """
    _refuse_objects_without_generator(backend, scenario)
    generator = WORLD_GENERATORS.get(backend['name'])
    if generator is None:
        return None
    mount_xyz = robot.get('mjcf_mount_xyz', [0.0, 0.0, 0.0])
    if backend['name'] in RENDERED_CAMERAS:
        return generator(
            robot_mjcf, mount_xyz, scenario, seed, out_dir, [camera.model for camera in cameras]
        )
    return generator(robot_mjcf, mount_xyz, scenario, seed, out_dir)


def sim_cameras(backend, robot, robot_description, robot_mjcf):
    """Return the robot's ``CameraPlacement``s on ``backend``; none where its world has none.

    The cameras are the roles of the robot's ``cameras_file``
    (``cameras_file.load``). Each renders a pinhole view wide enough for its real
    fisheye (``fisheye.render_camera``; the warp renders equidistant cameras
    only), mounted where the description puts its optical frame, on the nearest
    body the backend's model shares with the description. Refuses a cameras file
    the loader refuses, a camera the warp cannot render, and a frame that does
    not reach a body through fixed joints.
    """
    rendered = RENDERED_CAMERAS.get(backend['name'])
    if rendered is None or 'cameras_file' not in robot:
        return []
    generator, render_topic = rendered
    bodies = generator.body_names(robot_mjcf)
    path = catalog.share_file(robot['cameras_file'])
    try:
        placed = {
            role: (mount.fixed_mount(robot_description, camera['frame'], bodies),
                   fisheye.render_camera(fisheye.from_intrinsics(camera['intrinsics'])))
            for role, camera in cameras_file.load(path).items()
        }
    except (cameras_file.CameraError, OSError) as error:
        raise catalog.CatalogError(f'robot {robot["name"]!r} cameras: {error}') from error
    return [
        CameraPlacement(
            role, path, render_topic.format(role=role),
            mujoco_world.Camera(
                f'cam_{role}', where.parent, where.xyz, where.quat_wxyz,
                render.fovy_deg, (render.width, render.height),
            ),
        )
        for role, (where, render) in sorted(placed.items())
    ]


def camera_node_name(camera):
    """Return the ``sim_camera`` node's name for ``camera``."""
    return f'{params.CAMERA_NODE_PREFIX}{camera.role}'


def forward_params_file(arguments, params_file):
    """Return an included launch's ``arguments`` with ``params_file`` passed on, when set."""
    return {**arguments, 'params_file': params_file} if params_file else dict(arguments)


def own_node_guards(cameras, scenario_parameters):
    """Return the guard for the nodes run.launch.py starts: ``{fully qualified node: params}``.

    Each node's own parameters are what the launch owns there; ``params``
    lists the same keys (``CAMERA_OWNED``, ``SCENARIO_OWNED``) so no tunable
    targets one.
    """
    guards = {f'/{camera_node_name(camera)}': camera_node_parameters(camera) for camera in cameras}
    if scenario_parameters:
        guards[f'/{params.SCENARIO_NODE}'] = scenario_parameters
    return guards


def camera_node_parameters(camera):
    """Return the ``sim_camera`` node's parameters for ``camera``."""
    return {
        'role': camera.role,
        'cameras_file': camera.cameras_file,
        'render_topic': camera.render_topic,
        'use_sim_time': True,
    }


def mesh_roots(robot):
    """Return the robot's ``mesh_roots`` as ``[[prefix, share directory]]``."""
    return [[root['prefix'], catalog.share_file(root)] for root in robot.get('mesh_roots', [])]


def backend_launch_arguments(backend, robot, robot_description, controllers_file, world):
    """Return the backend launch's arguments.

    Every backend gets its ``launch_args``, the description and the controllers
    file; a ``STAGE_BACKENDS`` one also gets ``world`` (empty for none), the
    robot's ``cameras_file`` (empty for none), its ``mesh_roots`` as JSON and its
    name as ``robot`` (gazebo names the spawned model with it).
    """
    arguments = {
        **backend.get('launch_args', {}),
        'robot_description': robot_description,
        'controllers_file': controllers_file,
    }
    if backend['name'] in STAGE_BACKENDS:
        arguments['world'] = world or ''
        arguments['cameras_file'] = (
            catalog.share_file(robot['cameras_file']) if 'cameras_file' in robot else ''
        )
        arguments['mesh_roots'] = json.dumps(mesh_roots(robot))
        arguments['robot'] = robot['name']
    return arguments


def scenario_node_parameters(backend, scenario, seed, timeout_s):
    """Return the scenario node's parameters, or None when the scenario needs none.

    A scenario without objects has nothing to publish, reset or judge.
    """
    if not scenario['objects']:
        return None
    return {
        'scenario': json.dumps(scenario, sort_keys=True),
        'seed': seed,
        'backend': backend['name'],
        'timeout_s': timeout_s,
        'use_sim_time': True,
    }
