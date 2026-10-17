"""The isaac backend's stage wiring, camera relays and stage runner, without Isaac Sim.

The stage runner (``isaac/fm_sim_isaac.py``) is loaded from its file as Isaac
Sim's Python would; only its pure helpers run here, so its omni half is still
unverified until it runs on the GPU host.
"""

import importlib.util
import json
import math
import os
import tempfile
import threading
import time
import types

from fm_sim_backends import camera_info, isaac
from fm_sim_scenarios import isaac_stage
from launch import LaunchContext
from launch.actions import ExecuteProcess, LogInfo
from launch.utilities import perform_substitutions
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.join(HERE, '..')
RUNNER = os.path.join(PACKAGE, 'isaac', 'fm_sim_isaac.py')
INTRINSICS = {
    'width': 4, 'height': 2, 'fx': 500.0, 'fy': 510.0, 'cx': 2.0, 'cy': 1.0,
    'distortion_model': 'equidistant', 'distortion': [0.1, -0.02, 0.0, 0.0],
}
CAMERAS = {'chest': {'frame': 'cam', 'rate_hz': 30, 'intrinsics': INTRINSICS}}
SCENARIO = {
    'name': 'one_box', 'objects': [{
        'name': 'box', 'shape': 'box', 'size': [0.1, 0.1, 0.1], 'mass': 0.2,
        'friction': 0.5, 'color': [1.0, 0.0, 0.0, 1.0], 'pose': {'xyz': [0.4, 0.0, 0.05]},
    }],
}
URDF = """<robot name="probe">
  <link name="world"/><link name="cam"/>
  <joint name="cam_joint" type="fixed"><parent link="world"/><child link="cam"/></joint>
  <ros2_control name="probe_system" type="system">
    <hardware>
      <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
      <param name="joint_commands_topic">/isaac_joint_commands</param>
      <param name="joint_states_topic">/isaac_joint_states</param>
    </hardware>
  </ros2_control>
</robot>"""


def _load_runner():
    spec = importlib.util.spec_from_file_location('fm_sim_isaac', RUNNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_launch():
    path = os.path.join(PACKAGE, 'launch', 'isaac.launch.py')
    spec = importlib.util.spec_from_file_location('isaac_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# --- Stage runner, pure half --------------------------------------------------


def test_runner_imports_without_isaac_sim():
    runner = _load_runner()
    assert runner.STAGE_SCHEMA_VERSION == isaac_stage.STAGE_SCHEMA_VERSION
    assert runner.ARTICULATION_ROOT == isaac_stage.ARTICULATION_ROOT


def test_runner_refuses_another_schema_version(tmp_path):
    runner = _load_runner()
    path = tmp_path / 'stage.json'
    path.write_text(json.dumps({'schema_version': 99}))
    with pytest.raises(runner.SpecError, match='schema_version 99'):
        runner.load_spec(str(path))


def test_graph_edit_replays_the_spec_graph():
    runner = _load_runner()
    graph = isaac_stage.bridge_graph(
        '/isaac_joint_states', '/isaac_joint_commands', [],
        isaac_stage.object_specs(SCENARIO, 0),
    )
    keys = types.SimpleNamespace(CREATE_NODES='create', SET_VALUES='set', CONNECT='connect')
    where, edits = runner.graph_edit(graph, '/probe/root_joint', keys, lambda p: ('path', p))
    assert where == {'graph_path': isaac_stage.GRAPH_PRIM, 'evaluator_name': 'execution'}
    assert edits['create'] == [tuple(node) for node in graph['nodes']]
    assert edits['connect'] == [tuple(c) for c in graph['connections']]
    values = dict(edits['set'])
    assert values['joint_state.inputs:targetPrim'] == [('path', '/probe/root_joint')]
    assert values['transforms.inputs:targetPrims'] == [('path', '/World/scenario/box')]
    assert values['clock.inputs:topicName'] == '/clock'


def _transform(name, frame, xyz, quat_wxyz=(1.0, 0.0, 0.0, 0.0)):
    from geometry_msgs.msg import TransformStamped

    t = TransformStamped()
    t.header.frame_id = frame
    t.child_frame_id = name
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    rotation = t.transform.rotation
    rotation.w, rotation.x, rotation.y, rotation.z = quat_wxyz
    return t


WORKSPACE = isaac_stage.object_workspace(SCENARIO)
NAN = float('nan')


def test_object_commands_apply_a_valid_teleport():
    runner = _load_runner()
    msg = TFMessage(transforms=[_transform('box', 'World', (0.4, 0.1, 0.05))])
    targets, problems = runner.object_commands(msg, {'box'}, WORKSPACE)
    assert targets == {'box': ((0.4, 0.1, 0.05), (1.0, 0.0, 0.0, 0.0))}
    assert problems == []


def test_object_commands_normalise_the_quaternion():
    runner = _load_runner()
    transform = _transform('box', 'World', (0.4, 0.0, 0.05), quat_wxyz=(2.0, 0.0, 0.0, 0.0))
    targets, _ = runner.object_commands(TFMessage(transforms=[transform]), {'box'}, WORKSPACE)
    assert targets['box'][1] == (1.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize('quat, expected', [
    ((1e200, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
    # Finite components whose norm is beyond the float range (hypot gives inf).
    ((1.5e308, 1.5e308, 0.0, 0.0), (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)),
    ((1e308, 1e308, 1e308, 1e308), (0.5, 0.5, 0.5, 0.5)),
    # Tiny but not degenerate.
    ((0.0, 0.0, 0.0, 1e-3), (0.0, 0.0, 0.0, 1.0)),
])
def test_object_commands_normalise_a_huge_quaternion_without_overflow(quat, expected):
    runner = _load_runner()
    transform = _transform('box', 'World', (0.4, 0.0, 0.05), quat_wxyz=quat)
    targets, problems = runner.object_commands(
        TFMessage(transforms=[transform]), {'box'}, WORKSPACE
    )
    assert problems == []
    assert targets['box'][1] == pytest.approx(expected)


@pytest.mark.parametrize('name, frame, xyz, quat, problem', [
    ('table', 'World', (0.4, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0), 'not a free object'),
    ('box', 'openarm_body_link0', (0.4, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0), 'not the world'),
    ('box', 'World', (NAN, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0), 'not finite'),
    ('box', 'World', (0.4, float('inf'), 0.05), (1.0, 0.0, 0.0, 0.0), 'not finite'),
    ('box', 'World', (0.4, 0.0, 0.05), (NAN, 0.0, 0.0, 0.0), 'not finite'),
    # A default TransformStamped: identity position, all-zero quaternion.
    ('box', 'World', (0.4, 0.0, 0.05), (0.0, 0.0, 0.0, 0.0), 'zero norm'),
    ('box', 'World', (5.0, 0.0, 0.05), (1.0, 0.0, 0.0, 0.0), 'outside the workspace'),
    ('box', 'World', (0.4, 0.0, -3.0), (1.0, 0.0, 0.0, 0.0), 'outside the workspace'),
])
def test_object_commands_skip_a_bad_teleport(name, frame, xyz, quat, problem):
    runner = _load_runner()
    msg = TFMessage(transforms=[_transform(name, frame, xyz, quat_wxyz=quat)])
    targets, problems = runner.object_commands(msg, {'box'}, WORKSPACE)
    assert targets == {}
    (reported,) = problems
    assert problem in reported


def test_a_teleport_that_raises_is_logged_and_the_rest_still_move():
    numpy = pytest.importorskip('numpy')
    del numpy
    runner = _load_runner()

    class Body:
        def __init__(self, fails):
            self.fails, self.moved = fails, False

        def set_world_pose(self, position, orientation):
            if self.fails:
                raise RuntimeError('physx said no')
            self.moved = True

        def set_linear_velocity(self, velocity):
            pass

        def set_angular_velocity(self, velocity):
            pass

    bodies = {'bad': Body(fails=True), 'good': Body(fails=False)}
    errors = []
    target = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    runner._teleport(bodies, {'bad': target, 'good': target}, errors.append)
    assert bodies['good'].moved
    (error,) = errors
    assert 'bad' in error and 'physx said no' in error


# --- camera_info node ---------------------------------------------------------


def test_camera_info_carries_the_intrinsics():
    info = camera_info.camera_info(INTRINSICS)
    assert (info.width, info.height, info.distortion_model) == (4, 2, 'equidistant')
    assert list(info.k) == [500.0, 0.0, 2.0, 0.0, 510.0, 1.0, 0.0, 0.0, 1.0]
    assert list(info.p)[:3] == [500.0, 0.0, 2.0]
    assert list(info.d) == INTRINSICS['distortion']


def test_camera_info_node_answers_each_image_of_the_calibrated_size():
    rclpy.init()
    try:
        node = camera_info.CameraInfoNode(
            parameter_overrides=[Parameter('cameras', value=json.dumps(CAMERAS))]
        )
        probe = rclpy.create_node('camera_info_probe')
        received = []
        probe.create_subscription(CameraInfo, '/cam_chest/camera_info', received.append, 10)
        images = probe.create_publisher(Image, '/cam_chest/image_raw', 10)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(probe)
        threading.Thread(target=executor.spin, daemon=True).start()

        def publish(width):
            image = Image(width=width, height=2)
            image.header.frame_id = 'cam'
            image.header.stamp.sec = 42
            images.publish(image)

        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            publish(4)
            time.sleep(0.1)
        assert received, 'no camera_info for a calibrated image'
        assert received[0].header.frame_id == 'cam'
        assert received[0].header.stamp.sec == 42
        assert received[0].k[0] == 500.0
        count = len(received)
        for _ in range(5):
            publish(8)
            time.sleep(0.05)
        time.sleep(0.2)
        assert len(received) == count, 'camera_info for an image of another size'
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
    finally:
        rclpy.try_shutdown()


# --- Stage actions and the launch ---------------------------------------------


class _NodeRecorder:

    def __init__(self):
        self.calls = []

    def __call__(self, **kwargs):
        self.calls.append(kwargs)
        return kwargs


def _cameras_file(tmp_path):
    path = tmp_path / 'cameras.yaml'
    path.write_text(yaml.safe_dump({'cameras': CAMERAS}))
    return str(path)


def _world(tmp_path):
    run_dir = tempfile.mkdtemp(prefix=isaac_stage.WORLD_DIR_PREFIX, dir=str(tmp_path))
    return isaac_stage.write('/unused.xml', [0, 0, 0], SCENARIO, 4, run_dir)


def test_no_world_builds_no_stage():
    (action,) = isaac.stage_actions('', URDF, '', '', False)
    assert isinstance(action, LogInfo)


def test_stage_with_cameras_starts_the_relays_and_logs_the_runner(tmp_path, monkeypatch):
    recorder = _NodeRecorder()
    monkeypatch.setattr(isaac, 'Node', recorder)
    world = _world(tmp_path)
    actions = isaac.stage_actions(world, URDF, _cameras_file(tmp_path), '', False)
    run_dir = os.path.dirname(world)
    assert os.path.isfile(os.path.join(run_dir, 'stage.json'))
    assert os.path.isfile(os.path.join(run_dir, 'robot.urdf'))
    republish, info = recorder.calls
    assert (republish['package'], republish['executable']) == ('image_transport', 'republish')
    assert republish['parameters'] == [{'in_transport': 'raw', 'out_transport': 'compressed'}]
    # Pinned: a remap of the base name 'out' publishes /out/compressed on Jazzy.
    assert republish['remappings'] == [
        ('in', '/cam_chest/image_raw'),
        ('out/compressed', '/cam_chest/image_raw/compressed'),
    ]
    assert info['executable'] == 'camera_info'
    # Host-local discovery, like the runner.
    assert republish['additional_env'] == info['additional_env'] == isaac.RUNNER_ENV
    assert json.loads(info['parameters'][0]['cameras']) == CAMERAS
    (notice,) = actions[2:]
    assert isinstance(notice, LogInfo)
    text = ''.join(part.text for part in notice.msg)
    assert 'isaac_python is empty' in text
    assert isaac.ISAAC_PYTHON_PLACEHOLDER in text
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' in text


def test_relays_are_launch_ros_nodes_with_the_host_local_env():
    # Unpatched: launch_ros's Node on Jazzy must accept additional_env.
    from launch_ros.actions import Node

    relays = isaac.camera_relays(CAMERAS)
    assert len(relays) == 2 and all(isinstance(relay, Node) for relay in relays)
    context = LaunchContext()
    for relay in relays:
        env = {
            perform_substitutions(context, key): perform_substitutions(context, value)
            for key, value in relay.additional_env
        }
        assert env == {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}


def test_isaac_python_starts_the_runner_on_the_stage(tmp_path, monkeypatch):
    monkeypatch.setattr(isaac, 'Node', _NodeRecorder())
    world = _world(tmp_path)
    stage_path = os.path.join(os.path.dirname(world), 'stage.json')
    notice, process = isaac.stage_actions(world, URDF, '', '/isaac/python.sh', True)
    assert isinstance(notice, LogInfo) and isinstance(process, ExecuteProcess)
    command = isaac.runner_command(stage_path, '/isaac/python.sh', True)
    # The literal isaac_python, not a placeholder, in what is logged and run.
    assert ' '.join(command) in ''.join(part.text for part in notice.msg)
    assert command[0] == '/isaac/python.sh'
    assert command[1].endswith('/fm_sim_backends/isaac/fm_sim_isaac.py')
    assert os.path.isfile(command[1])
    assert command[2:] == ['--spec', stage_path, '--gui']
    context = LaunchContext()
    assert [perform_substitutions(context, part) for part in process.cmd] == command
    env = {
        perform_substitutions(context, key): perform_substitutions(context, value)
        for key, value in process.additional_env
    }
    assert env == {'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}


def test_bad_cameras_file_fails_the_launch(tmp_path):
    path = tmp_path / 'cameras.yaml'
    path.write_text(yaml.safe_dump({'cameras': {'chest': {'frame': 'cam'}}}))
    with pytest.raises(isaac_stage.StageError, match='rate_hz'):
        isaac.stage_actions(_world(tmp_path), URDF, str(path), '', False)


def test_launch_hands_its_arguments_to_the_stage(tmp_path, monkeypatch):
    module = _load_launch()
    calls = []
    monkeypatch.setattr(module.isaac, 'stage_actions', lambda *a: calls.append(a) or [])
    context = LaunchContext()
    context.launch_configurations.update({
        'world': '/w.json', 'robot_description': URDF, 'cameras_file': '/c.yaml',
        'isaac_python': '/p.sh', 'isaac_gui': 'false',
    })
    assert module._stage_setup(context) == []
    assert calls == [('/w.json', URDF, '/c.yaml', '/p.sh', False)]
    context.launch_configurations['isaac_gui'] = 'yes'
    with pytest.raises(ValueError, match='isaac_gui'):
        module._stage_setup(context)
