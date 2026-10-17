"""Live: OpenArm on Gazebo with can_on_table and the idle controller.

run.launch.py must build the scenario world, spawn the robot as ``openarm`` and
start the scenario node: the can shows on /sim/objects, /sim/reset with a seed
moves it to that seed's spawn inside the scenario's ranges, and with nothing
moving the can, /sim/outcome latches a timeout failure for the reset's episode.
Both cameras publish JPEG images and a camera_info with the cameras file's
intrinsics. ``timeout_s:=10`` keeps the episode short. After shutdown, no gz-sim
server of this run survives and the generated world's directory is gone.
Linux only (it reads /proc): Gazebo runs server-only with headless (EGL)
rendering, on software GL in the image.
"""

import glob
import os
import platform
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from fm_sim_bringup import catalog
from fm_sim_msgs.msg import ObjectStates, Outcome
from fm_sim_msgs.srv import Reset
from fm_sim_scenarios import scenario as sc
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage
import yaml

if platform.system() != 'Linux':
    pytest.skip('Gazebo runs headless in the Linux image only', allow_module_level=True)

STARTUP_TIMEOUT_S = 240.0
EPISODE_TIMEOUT_S = 10.0
SEED = 5
# The can settles from its spawn onto the table or paper: a millimetre down, a
# little sideways at most.
SETTLE_TOLERANCE_M = 0.005
# /sim/objects must run at 10 Hz or more. Judged on the header stamps (sim time),
# so a host rendering on software GL below real time does not fail it: the
# PosePublisher publishes at 30 Hz of sim time, and the widest gap may be 0.1 s.
RATE_SAMPLES = 20
MAX_STAMP_GAP_S = 0.1
# Where run.launch.py writes the generated world; it must be gone after shutdown.
WORLD_GLOB = os.path.join(tempfile.gettempdir(), 'fm_sim_world_*')
WORLDS_BEFORE = set(glob.glob(WORLD_GLOB))
# This run's worlds, seen while it runs; the post-shutdown check looks for a gz-sim
# server still running on one of them.
RUN_WORLDS = set()
CAMERA_ROLES = ('chest', 'wrist_right')


@pytest.mark.launch_test
def generate_test_description():
    run_launch = os.path.join(
        get_package_share_directory('fm_sim_bringup'), 'launch', 'run.launch.py'
    )
    # A partition of its own, so no other Gazebo on this host answers this run.
    os.environ['GZ_PARTITION'] = f'fm_sim_gazebo_scenario_{os.getpid()}'
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(run_launch),
            launch_arguments={
                'backend': 'gazebo',
                'robot': 'openarm',
                'scenario': 'can_on_table',
                'controller': 'idle',
                'timeout_s': str(EPISODE_TIMEOUT_S),
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


def _spin_until(node, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def _can(states):
    for state in states.objects:
        if state.name == 'can':
            position = state.pose.position
            return (position.x, position.y, position.z)
    return None


def _gz_servers(worlds):
    """Pids of gz-sim processes running on any of ``worlds``."""
    pids = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/cmdline', 'rb') as cmdline:
                argv = cmdline.read().decode(errors='replace').split('\0')
        except OSError:
            continue
        if 'sim' in argv and any(a.startswith(w) for a in argv for w in worlds):
            pids.append(int(entry))
    return pids


class TestCanOnTable(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('scenario_launch_probe')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_objects_reset_and_outcome(self):
        node = self.node
        scenario = catalog.load('scenarios', 'can_on_table')
        objects = []
        outcomes = []
        node.create_subscription(ObjectStates, '/sim/objects', objects.append, 10)
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        node.create_subscription(Outcome, '/sim/outcome', outcomes.append, latched)

        self.assertTrue(
            _spin_until(node, lambda: objects and _can(objects[-1]), STARTUP_TIMEOUT_S),
            'no can on /sim/objects',
        )
        self.assertEqual([s.name for s in objects[-1].objects], ['table', 'paper', 'can'])
        self.assertEqual(objects[-1].header.frame_id, 'world')
        RUN_WORLDS.update(set(glob.glob(WORLD_GLOB)) - WORLDS_BEFORE)
        self.assertTrue(RUN_WORLDS, 'no generated world')
        self.assertTrue(_gz_servers(RUN_WORLDS), 'no gz-sim server on the generated world')
        (world_dir,) = RUN_WORLDS
        self.assertTrue(os.path.isfile(os.path.join(world_dir, 'openarm.urdf')))

        objects.clear()
        self.assertTrue(_spin_until(node, lambda: len(objects) >= RATE_SAMPLES, 30.0))
        stamps = [Time.from_msg(states.header.stamp).nanoseconds * 1e-9 for states in objects]
        gaps = [later - earlier for earlier, later in zip(stamps, stamps[1:])]
        self.assertLessEqual(max(gaps), MAX_STAMP_GAP_S, f'/sim/objects stamp gaps {gaps}')

        # Reset; the engine's respawn service may still be coming up.
        reset = node.create_client(Reset, '/sim/reset')
        self.assertTrue(reset.wait_for_service(timeout_sec=30.0))
        response = None
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            future = reset.call_async(Reset.Request(seed=SEED))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            response = future.result()
            if response is not None and response.ok:
                break
            time.sleep(1.0)
        self.assertTrue(response is not None and response.ok, response)

        (spawn_xyz, _) = sc.spawn_poses(scenario, SEED)['can']
        (can,) = [obj for obj in scenario['objects'] if obj['name'] == 'can']

        def moved():
            position = _can(objects[-1])
            return position is not None and all(
                abs(p - s) <= SETTLE_TOLERANCE_M for p, s in zip(position, spawn_xyz)
            )

        objects.clear()
        self.assertTrue(
            _spin_until(node, lambda: objects and moved(), 10.0),
            f'can at {_can(objects[-1]) if objects else None}, spawn {spawn_xyz}',
        )
        nominal = can['pose']['xyz']
        position = _can(objects[-1])
        for axis, index in (('x', 0), ('y', 1)):
            low, high = can['spawn'][axis]
            self.assertGreaterEqual(position[index], nominal[index] + low - SETTLE_TOLERANCE_M)
            self.assertLessEqual(position[index], nominal[index] + high + SETTLE_TOLERANCE_M)

        # Idle never moves the can: the reset's episode times out as a failure.
        self.assertTrue(
            _spin_until(
                node, lambda: any(o.seed == SEED for o in outcomes), EPISODE_TIMEOUT_S * 6
            ),
            f'no outcome for seed {SEED}: {outcomes}',
        )
        (outcome,) = [o for o in outcomes if o.seed == SEED]
        self.assertEqual(outcome.scenario, 'can_on_table')
        self.assertFalse(outcome.success)
        self.assertIn('timeout', outcome.reason)
        self.assertAlmostEqual(outcome.elapsed_s, EPISODE_TIMEOUT_S, delta=0.5)

        # Latched: a subscriber that joins late still gets it.
        late = []
        node.create_subscription(Outcome, '/sim/outcome', late.append, latched)
        self.assertTrue(_spin_until(node, lambda: late, 5.0))
        self.assertEqual(late[-1].seed, SEED)

    def test_cameras_publish_jpeg_and_real_intrinsics(self):
        node = self.node
        cameras_file = catalog.share_file(catalog.load('robots', 'openarm')['cameras_file'])
        with open(cameras_file) as camera_yaml:
            cameras = yaml.safe_load(camera_yaml)['cameras']
        images = {role: [] for role in CAMERA_ROLES}
        infos = {role: [] for role in CAMERA_ROLES}
        for role in CAMERA_ROLES:
            node.create_subscription(
                CompressedImage, f'/cam_{role}/image_raw/compressed', images[role].append, 1
            )
            node.create_subscription(
                CameraInfo, f'/cam_{role}/camera_info', infos[role].append, 1
            )
        self.assertTrue(
            _spin_until(
                node, lambda: all(images[r] and infos[r] for r in CAMERA_ROLES),
                STARTUP_TIMEOUT_S,
            ),
            {r: (len(images[r]), len(infos[r])) for r in CAMERA_ROLES},
        )
        for role in CAMERA_ROLES:
            camera = cameras[role]
            intrinsics = camera['intrinsics']
            image, info = images[role][-1], infos[role][-1]
            self.assertIn('jpeg', image.format)
            self.assertEqual(bytes(image.data[:2]), b'\xff\xd8')
            self.assertEqual(image.header.frame_id, camera['frame'])
            self.assertEqual(info.header.frame_id, camera['frame'])
            self.assertEqual(
                (info.width, info.height), (intrinsics['width'], intrinsics['height'])
            )
            fx, fy, cx, cy = (intrinsics[k] for k in ('fx', 'fy', 'cx', 'cy'))
            self.assertEqual(list(info.k), [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0])
            # Gazebo renders a pinhole: no distortion.
            self.assertEqual(info.distortion_model, 'plumb_bob')
            self.assertEqual(list(info.d), [0.0] * 5)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_no_gz_server_survives(self):
        self.assertTrue(RUN_WORLDS, 'the live test saw no world')
        deadline = time.monotonic() + 10.0
        while _gz_servers(RUN_WORLDS) and time.monotonic() < deadline:
            time.sleep(0.5)
        self.assertEqual(_gz_servers(RUN_WORLDS), [])

    def test_generated_world_is_removed_on_shutdown(self):
        self.assertEqual(set(glob.glob(WORLD_GLOB)) - WORLDS_BEFORE, set())
