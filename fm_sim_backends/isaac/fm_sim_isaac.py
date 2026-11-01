"""Build and run an fm-sim stage in Isaac Sim 4.5 (ISAAC SIM ONLY).

Run with Isaac Sim's own Python, never the ROS one:

    $ISAAC_SIM/python.sh fm_sim_isaac.py --spec <run dir>/stage.json [--gui]

The spec is written by ``fm_sim_scenarios.isaac_stage.write_stage`` (from the
isaac backend launch) and decides everything: this script only replays it with
the omni APIs. It must see the same filesystem as the launch (the URDF's mesh
paths are absolute), and talks ROS 2 Jazzy through Isaac Sim's bundled bridge,
so the environment needs ``ROS_DISTRO=jazzy`` and the bridge's Jazzy libraries
(Isaac Sim's ROS 2 setup); ``ROS_DOMAIN_ID`` is inherited, and the isaac
launch sets ``ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`` as the agent does for a
run's container.

Stage: physics scene, ground plane, the URDF imported at the origin with
position drives on its joints, the scenario objects with mass and friction at
their seeded spawn, a camera on each camera frame, and the ROS 2 bridge graph
(``/clock``, joint states and commands, camera images, object poses). The loop
then steps the world and applies object teleports from
``object_commands_topic`` (tf2_msgs/TFMessage, through Isaac's rclpy): a free
object, a finite pose inside ``object_workspace``, a non-zero quaternion.

An object naming a ``usd`` asset is read from the directory ``FM_ISAAC_ASSETS_DIR``
names (the agent's ``--assets-dir``), which ``tools/fetch_simready.py`` fills.

Every omni import is inside a function: the module imports without Isaac Sim,
so its pure helpers unit-test in fm-sim's CI. The Isaac half has not run in CI.
"""

import argparse
import json
import math
import os
import sys
import traceback

STAGE_SCHEMA_VERSION = 1
# Set by the sim agent's Isaac container (agent/src/fm_sim_agent/isaac.py).
READY_FILE_ENV = 'FM_ISAAC_READY_FILE'
ASSETS_DIR_ENV = 'FM_ISAAC_ASSETS_DIR'
# Placeholder the spec uses for the articulation root (isaac_stage.ARTICULATION_ROOT).
ARTICULATION_ROOT = '$ARTICULATION_ROOT'
ROS2_BRIDGE_EXTENSION = 'isaacsim.ros2.bridge'
# A quaternion this close to zero norm has no rotation to normalise.
MIN_QUATERNION_NORM = 1e-6


class SpecError(ValueError):
    """A stage spec this runner cannot replay."""


# --- Pure helpers (tested without Isaac Sim) ---------------------------------


def load_spec(path):
    """Read a stage spec, refusing another schema version."""
    with open(path) as spec_file:
        spec = json.load(spec_file)
    if spec.get('schema_version') != STAGE_SCHEMA_VERSION:
        raise SpecError(
            f'{path}: schema_version {spec.get("schema_version")!r}, '
            f'this runner reads {STAGE_SCHEMA_VERSION}'
        )
    return spec


def usd_path(usd, assets_dir):
    """Return the file a spec's ``usd`` names inside ``assets_dir``, which must hold it.

    Checked before Isaac Sim starts: a cold start takes minutes to reach the same error.
    """
    if not assets_dir:
        raise SpecError(
            f'{usd}: {ASSETS_DIR_ENV} is not set; start the agent with --assets-dir, '
            'filled by tools/fetch_simready.py'
        )
    root = os.path.realpath(assets_dir)
    path = os.path.realpath(os.path.join(root, usd))
    if not path.startswith(root + os.sep):
        raise SpecError(f'{usd} is outside the assets directory {root}')
    if not os.path.isfile(path):
        raise SpecError(f'{usd} is not in {root}; tools/fetch_simready.py downloads it')
    return path


def graph_edit(graph, articulation_root, keys, to_prim_path):
    """Return the ``og.Controller.edit`` arguments for the spec's bridge graph.

    ``keys`` is ``og.Controller.Keys``; ``to_prim_path`` turns a path string into
    the value a prim-typed input takes (``usdrt.Sdf.Path``). The articulation
    placeholder becomes ``articulation_root``.
    """
    def _path(path):
        return to_prim_path(articulation_root if path == ARTICULATION_ROOT else path)

    edits = {
        keys.CREATE_NODES: [tuple(node) for node in graph['nodes']],
        keys.SET_VALUES: [tuple(value) for value in graph['values']] + [
            (attribute, [_path(p) for p in paths]) for attribute, paths in graph['prim_values']
        ],
        keys.CONNECT: [tuple(connection) for connection in graph['connections']],
    }
    return {'graph_path': graph['path'], 'evaluator_name': 'execution'}, edits


def _finite(values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def drive_target(kind, position):
    """Return a drive's target in USD units: degrees for angular, metres for linear."""
    return math.degrees(position) if kind == 'angular' else position


def object_commands(msg, free_objects, workspace, world_frames=('World', 'world')):
    """Return ``({name: (xyz, quat_wxyz)}, [problems])`` from a TFMessage command.

    Only a free object of the stage moves, to a finite position inside
    ``workspace`` (``{'min': xyz, 'max': xyz}``, the spec's
    ``object_workspace``) with a finite, non-zero quaternion, which is
    normalised. Anything else is reported in ``problems`` and not applied.
    """
    targets, problems = {}, []
    for transform in msg.transforms:
        name = transform.child_frame_id
        t, r = transform.transform.translation, transform.transform.rotation
        xyz, quat = (t.x, t.y, t.z), (r.w, r.x, r.y, r.z)
        if transform.header.frame_id not in world_frames:
            problems.append(f'{name}: frame {transform.header.frame_id!r} is not the world')
        elif name not in free_objects:
            problems.append(f'{name}: not a free object of this stage')
        elif not _finite(xyz + quat):
            problems.append(f'{name}: pose is not finite: {xyz} {quat}')
        elif not all(lo <= v <= hi for v, lo, hi in zip(xyz, workspace['min'], workspace['max'])):
            problems.append(f'{name}: {xyz} is outside the workspace {workspace}')
        else:
            # Scale by the largest component before the norm: squaring (or hypot of)
            # components near the float limit overflows to inf, even though the
            # quaternion is finite. Scaled, the norm lies in [1, 2].
            largest = max(abs(q) for q in quat)
            if largest < MIN_QUATERNION_NORM:
                problems.append(f'{name}: quaternion {quat} has no rotation (zero norm)')
            else:
                scaled = tuple(q / largest for q in quat)
                norm = math.hypot(*scaled)
                targets[name] = (xyz, tuple(q / norm for q in scaled))
    return targets, problems


# --- Isaac Sim (needs the omni runtime) --------------------------------------


def _set_xform(prim, translate, orient_wxyz, scale=None):
    from pxr import Gf, UsdGeom

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*orient_wxyz))
    if scale is not None:
        xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))


def _import_robot(robot):
    """Import the URDF; return the articulation root prim path."""
    import omni.kit.commands

    _, config = omni.kit.commands.execute('URDFCreateImportConfig')
    for key, value in robot['import_config'].items():
        getattr(config, f'set_{key}')(value)
    ok, root = omni.kit.commands.execute(
        'URDFParseAndImportFile', urdf_path=robot['urdf_path'], import_config=config,
        get_articulation_root=True,
    )
    if not ok or not root:
        raise SpecError(f'URDF import of {robot["urdf_path"]} failed')
    return str(root)


def _apply_drives(stage, robot):
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(robot['prim_path'])
    if not root.IsValid():
        raise SpecError(f'robot prim {robot["prim_path"]} missing after the URDF import')
    count = 0
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdPhysics.RevoluteJoint):
            kind = 'angular'
        elif prim.IsA(UsdPhysics.PrismaticJoint):
            kind = 'linear'
        else:
            continue
        gains = robot['drives'][kind]
        drive = UsdPhysics.DriveAPI.Apply(prim, kind)
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(gains['stiffness'])
        drive.CreateDampingAttr(gains['damping'])
        drive.CreateMaxForceAttr(gains['max_force'])
        # The bridge streams no command until a controller is active, so the
        # drive holds this target until then; zero would pull the arm into the table.
        position = robot['initial_positions'].get(prim.GetName(), 0.0)
        drive.CreateTargetPositionAttr(drive_target(kind, position))
        count += 1
    if count == 0:
        raise SpecError(f'no revolute or prismatic joint under {robot["prim_path"]}')


def _add_object(stage, obj):
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

    geom_class = getattr(UsdGeom, obj['prim_type'])
    geom = geom_class.Define(stage, obj['prim_path'])
    for attribute, value in obj['attributes'].items():
        geom.GetPrim().GetAttribute(attribute).Set(value)
    _set_xform(geom.GetPrim(), obj['translate'], obj['orient_wxyz'], obj['scale'])
    geom.CreateDisplayColorAttr([Gf.Vec3f(*obj['color'])])
    geom.CreateDisplayOpacityAttr([obj['opacity']])
    prim = geom.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    if obj['rigid_body']:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(obj['mass'])
    material_path = Sdf.Path(f'{obj["prim_path"]}_material')
    material = UsdShade.Material.Define(stage, material_path)
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(obj['material']['static_friction'])
    physics.CreateDynamicFrictionAttr(obj['material']['dynamic_friction'])
    physics.CreateRestitutionAttr(obj['material']['restitution'])
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateFrictionCombineModeAttr(obj['material']['friction_combine_mode'])
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, 'physics'
    )


def _add_usd_object(stage, obj, path):
    from pxr import Usd, UsdGeom, UsdPhysics

    prim = UsdGeom.Xform.Define(stage, obj['prim_path']).GetPrim()
    prim.GetReferences().AddReference(path)
    variant_set, selection = obj['variant']
    prim.GetVariantSets().GetVariantSet(variant_set).SetVariantSelection(selection)
    _set_xform(prim, obj['translate'], obj['orient_wxyz'])
    # A SimReady asset's body is a prim below its root, and the bridge names a pose after the
    # prim it tracks. The body moves up to the object's own prim, so poses and teleports use
    # its name; the colliders below keep their authored mass and materials.
    for descendant in Usd.PrimRange(prim):
        if descendant != prim and descendant.HasAPI(UsdPhysics.RigidBodyAPI):
            descendant.RemoveAPI(UsdPhysics.RigidBodyAPI)
    UsdPhysics.RigidBodyAPI.Apply(prim)


def _add_camera(stage, camera):
    from pxr import Gf, UsdGeom

    if not stage.GetPrimAtPath(camera['parent_prim']).IsValid():
        if camera['parent_prim'].startswith('/World/'):
            UsdGeom.Xform.Define(stage, camera['parent_prim'])
        else:
            raise SpecError(
                f'camera {camera["role"]}: link prim {camera["parent_prim"]} is missing '
                'after the URDF import'
            )
    usd_camera = UsdGeom.Camera.Define(stage, camera['prim_path'])
    _set_xform(usd_camera.GetPrim(), camera['translate'], camera['orient_wxyz'])
    lens = camera['lens']
    usd_camera.CreateFocalLengthAttr(lens['focalLength'])
    usd_camera.CreateHorizontalApertureAttr(lens['horizontalAperture'])
    usd_camera.CreateVerticalApertureAttr(lens['verticalAperture'])
    usd_camera.CreateHorizontalApertureOffsetAttr(lens['horizontalApertureOffset'])
    usd_camera.CreateVerticalApertureOffsetAttr(lens['verticalApertureOffset'])
    usd_camera.CreateClippingRangeAttr(Gf.Vec2f(*lens['clippingRange']))
    intrinsics = camera['intrinsics']
    if intrinsics['distortion_model'] == 'equidistant':
        # The real chest camera is a fisheye: render with its OpenCV fisheye model.
        from isaacsim.sensors.camera import Camera

        sensor = Camera(
            prim_path=camera['prim_path'],
            resolution=(intrinsics['width'], intrinsics['height']),
        )
        if hasattr(sensor, 'set_opencv_fisheye_properties'):
            sensor.set_opencv_fisheye_properties(
                cx=intrinsics['cx'], cy=intrinsics['cy'], fx=intrinsics['fx'], fy=intrinsics['fy'],
                fisheye=list(intrinsics['distortion']),
            )
        else:
            # Isaac Sim 4.5 has no OpenCV setter. OpenCV's fisheye model is Kannala-Brandt
            # (k1..k4), fitted from the lens attributes above once the sensor has its
            # render product.
            sensor.initialize()
            sensor.set_projection_type('fisheyePolynomial')
            sensor.set_kannala_brandt_properties(
                nominal_width=intrinsics['width'], nominal_height=intrinsics['height'],
                optical_centre_x=intrinsics['cx'], optical_centre_y=intrinsics['cy'],
                max_fov=None, distortion_model=list(intrinsics['distortion']),
            )


def _build_graph(graph, articulation_root):
    import omni.graph.core as og
    import usdrt.Sdf

    where, edits = graph_edit(graph, articulation_root, og.Controller.Keys, usdrt.Sdf.Path)
    og.Controller.edit(where, edits)


class _CommandListener:
    """Object teleports from ROS 2, through Isaac Sim's bundled rclpy."""

    def __init__(self, topic, free_objects, workspace):
        import rclpy
        from tf2_msgs.msg import TFMessage

        rclpy.init()
        self._rclpy = rclpy
        self._node = rclpy.create_node('fm_sim_isaac_stage')
        self._free = free_objects
        self._workspace = workspace
        self._pending = {}
        self._node.create_subscription(TFMessage, topic, self._on_command, 10)

    def _on_command(self, msg):
        targets, problems = object_commands(msg, self._free, self._workspace)
        for problem in problems:
            self._node.get_logger().error(f'object command skipped: {problem}')
        self._pending.update(targets)

    def error(self, message):
        self._node.get_logger().error(message)

    def take(self):
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        pending, self._pending = self._pending, {}
        return pending

    def close(self):
        self._node.destroy_node()
        self._rclpy.try_shutdown()


def _teleport(bodies, targets, log_error):
    """Move each target body at rest; a body that refuses is logged and skipped."""
    import numpy as np

    for name, (xyz, quat_wxyz) in targets.items():
        try:
            body = bodies[name]
            body.set_world_pose(position=np.array(xyz), orientation=np.array(quat_wxyz))
            body.set_linear_velocity(np.zeros(3))
            body.set_angular_velocity(np.zeros(3))
        except Exception as error:  # noqa: B902 - one bad teleport must not end the run
            log_error(f'teleport of {name} failed: {type(error).__name__}: {error}')


def _mark_ready():
    """Write the agent's ready file, if it gave one: the stage is built and playing."""
    path = os.environ.get(READY_FILE_ENV)
    if path:
        with open(path, 'w') as ready:
            ready.write('playing\n')


def _set_initial_positions(articulation_root, positions):
    """Start each joint at its ready pose, as MuJoCo and Gazebo do."""
    import numpy as np
    from isaacsim.core.prims import SingleArticulation

    robot = SingleArticulation(articulation_root)
    robot.initialize()
    # Called after world.reset(), which returns every joint to zero. The importer
    # names each dof after its URDF joint, the key the spec uses.
    robot.set_joint_positions(np.array([positions.get(n, 0.0) for n in robot.dof_names]))


def run(spec, headless):
    """Start Isaac Sim, build the stage from ``spec`` and step it until closed."""
    usd_files = {
        obj['name']: usd_path(obj['usd'], os.environ.get(ASSETS_DIR_ENV))
        for obj in spec['objects'] if 'usd' in obj
    }

    from isaacsim import SimulationApp

    app = SimulationApp({'headless': headless})
    try:
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleRigidPrim
        from isaacsim.core.utils.extensions import enable_extension
        import omni.usd

        enable_extension(ROS2_BRIDGE_EXTENSION)
        app.update()
        physics = spec['physics']
        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / physics['physics_hz'],
            rendering_dt=1.0 / physics['render_hz'],
        )
        world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()
        articulation_root = _import_robot(spec['robot'])
        _apply_drives(stage, spec['robot'])
        for obj in spec['objects']:
            if obj['name'] in usd_files:
                _add_usd_object(stage, obj, usd_files[obj['name']])
            else:
                _add_object(stage, obj)
        for camera in spec['cameras']:
            _add_camera(stage, camera)
        _build_graph(spec['graph'], articulation_root)
        free = {o['name']: o['prim_path'] for o in spec['objects'] if o['rigid_body']}
        world.reset()
        _set_initial_positions(articulation_root, spec['robot']['initial_positions'])
        bodies = {name: SingleRigidPrim(path) for name, path in free.items()}
        listener = _CommandListener(
            spec['object_commands_topic'], set(free), spec['object_workspace']
        )
        world.play()
        _mark_ready()
        try:
            while app.is_running():
                targets = listener.take()
                if targets:
                    _teleport(bodies, targets, listener.error)
                world.step(render=True)
        finally:
            listener.close()
    except BaseException:
        # app.close() ends the process with status 0 before an exception would print.
        traceback.print_exc()
        sys.stderr.flush()
        raise
    finally:
        app.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--spec', required=True, help='stage.json from write_stage')
    parser.add_argument('--gui', action='store_true', help='open the Isaac Sim window')
    args = parser.parse_args(argv)
    run(load_spec(args.spec), headless=not args.gui)
    return 0


if __name__ == '__main__':
    sys.exit(main())
