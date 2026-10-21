"""Live: OpenArm on MuJoCo with can_on_table renders both cameras as the real ones.

run.launch.py must put the cameras in the world, render them in the engine's
CameraPlugin (EGL with no display) and start one sim_camera node per role. Each
role publishes a JPEG fisheye image of the real size on
/cam_<role>/image_raw/compressed with the profile's optical frame and a sim-time
stamp, at no more than 10 Hz of sim time (and not stalled), plus /cam_<role>/camera_info with the
real intrinsics. The can on /sim/objects, projected through TF and the real chest
intrinsics with cv2.fisheye, lands on can-green pixels in the chest image: the
mount, the render and the warp all agree with the description.
Needs the private openarm_v2_anvil_description (the fm-sim-controllers image) and
Mesa's EGL; skips without the description.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import cv2
from description_model import needs_openarm
from fm_sim_msgs.msg import ObjectStates
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage
import tf2_ros

STARTUP_TIMEOUT_S = 180.0
# Contract §4: the loader_openarm_v2 profile's camera frames.
FRAMES = {
    'chest': 'cam_chest_optical',
    'wrist_right': 'cam_wrist_r_optical',
}
# fm-skills src/openarm_v2_skills_config/config/camera_chest.yaml at aa91a3f; the
# wrist camera stands in with the same numbers.
K = [740.244, 0.0, 958.504, 0.0, 750.103, 573.152, 0.0, 0.0, 1.0]
D = [0.111681, -0.028494, 0.0, 0.0]
# A 1920x1080 JPEG of a rendered scene: well above a blank frame, well below raw.
MIN_JPEG_BYTES = 20_000
MAX_JPEG_BYTES = 1_500_000
RATE_SAMPLES = 15
# 10 Hz of sim time, CameraPlugin's rate: a stamp gap may jitter by a sim step or
# so below 0.1 s, never to 15 Hz.
MIN_STAMP_GAP_S = 0.066
MAX_MEDIAN_GAP_S = 0.5
LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


# A skip mark here, not a module-level pytest.skip: launch_testing imports this file to
# collect it, and a skip raised then ends the whole session's collection.
@needs_openarm
@pytest.mark.launch_test
def generate_test_description():
    run_launch = os.path.join(
        get_package_share_directory('fm_sim_bringup'), 'launch', 'run.launch.py'
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(run_launch),
            launch_arguments={
                'backend': 'mujoco',
                'robot': 'openarm',
                'scenario': 'can_on_table',
                'controller': 'idle',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


def _spin_until(node, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def _seconds(stamp):
    return Time.from_msg(stamp).nanoseconds * 1e-9


class TestCameras(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('camera_launch_probe')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_cameras_publish_the_real_fisheyes(self):
        node = self.node
        images = {role: [] for role in FRAMES}
        infos = {role: [] for role in FRAMES}
        clocks, objects = [], []
        for role in FRAMES:
            node.create_subscription(
                CompressedImage, f'/cam_{role}/image_raw/compressed', images[role].append, LATEST
            )
            node.create_subscription(
                CameraInfo, f'/cam_{role}/camera_info', infos[role].append, 10
            )
        node.create_subscription(ObjectStates, '/sim/objects', objects.append, 10)
        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer, node)

        self.assertTrue(
            _spin_until(node, lambda: all(images.values()) and all(infos.values()),
                        STARTUP_TIMEOUT_S),
            f'frames per role: {[(r, len(v)) for r, v in images.items()]}',
        )
        for role, frame in FRAMES.items():
            image, info = images[role][-1], infos[role][-1]
            self.assertEqual(image.header.frame_id, frame)
            self.assertEqual(image.format, 'jpeg')
            self.assertTrue(MIN_JPEG_BYTES < len(image.data) < MAX_JPEG_BYTES, len(image.data))
            decoded = cv2.imdecode(np.frombuffer(bytes(image.data), np.uint8), cv2.IMREAD_COLOR)
            self.assertEqual(decoded.shape, (1080, 1920, 3))
            self.assertGreater(float(decoded.std()), 10.0, f'{role} frame is flat')
            self.assertEqual(info.header.frame_id, frame)
            self.assertEqual((info.width, info.height), (1920, 1080))
            self.assertEqual(info.distortion_model, 'equidistant')
            np.testing.assert_allclose(info.k, K)
            np.testing.assert_allclose(info.d, D)
            self.assertGreater(_seconds(image.header.stamp), 0.0)

        # Bounded rate, judged on sim-time stamps.
        for role in FRAMES:
            images[role].clear()
        self.assertTrue(_spin_until(
            node, lambda: all(len(v) >= RATE_SAMPLES for v in images.values()), 60.0
        ))
        for role, received in images.items():
            stamps = [_seconds(image.header.stamp) for image in received]
            gaps = np.diff(stamps)
            self.assertGreaterEqual(gaps.min(), MIN_STAMP_GAP_S, f'{role} gaps {gaps}')
            # Throughput is the host's: llvmpipe renders both cameras in turn, and a
            # busy host renders below 10 Hz. Only a stalled camera fails here.
            self.assertLess(float(np.median(gaps)), MAX_MEDIAN_GAP_S, f'{role} gaps {gaps}')

        # Sim time: the latest stamps trail the engine's clock by a frame or so.
        # /clock runs at the physics rate, so it is read only now: earlier it
        # would crowd this node's single thread and drop frames.
        clock_subscription = node.create_subscription(Clock, '/clock', clocks.append, 10)
        self.assertTrue(_spin_until(node, lambda: clocks, 10.0))
        node.destroy_subscription(clock_subscription)
        for role, received in images.items():
            lag = _seconds(clocks[-1].clock) - _seconds(received[-1].header.stamp)
            self.assertTrue(0.0 <= lag < 5.0, f'{role} stamp lags the sim clock by {lag} s')

        # The can projects onto can-green pixels in the chest image.
        self.assertTrue(_spin_until(node, lambda: objects, 30.0))
        (can,) = [o.pose.position for o in objects[-1].objects if o.name == 'can']
        transform = None

        def looked_up():
            nonlocal transform
            try:
                transform = tf_buffer.lookup_transform(
                    FRAMES['chest'], 'world', rclpy.time.Time()
                ).transform
            except tf2_ros.TransformException:
                return False
            return True

        self.assertTrue(_spin_until(node, looked_up, 30.0))
        q = transform.rotation
        rotation, _ = cv2.Rodrigues(_rotation_matrix(q.w, q.x, q.y, q.z))
        translation = np.array([transform.translation.x, transform.translation.y,
                                transform.translation.z])
        point = np.array([[[can.x, can.y, can.z]]])
        pixel, _ = cv2.fisheye.projectPoints(
            point, rotation, translation, np.array(K).reshape(3, 3), np.array(D)
        )
        u, v = (int(round(c)) for c in pixel.reshape(2))
        chest = cv2.imdecode(
            np.frombuffer(bytes(images['chest'][-1].data), np.uint8), cv2.IMREAD_COLOR
        )
        self.assertTrue(0 <= u < 1920 and 0 <= v < 1080, (u, v))
        blue, green, red = chest[v - 3:v + 4, u - 3:u + 4].reshape(-1, 3).mean(axis=0)
        self.assertGreater(green, red + 10, f'not the can at {(u, v)}: bgr {blue, green, red}')
        self.assertGreater(green, blue + 10, f'not the can at {(u, v)}: bgr {blue, green, red}')


def _rotation_matrix(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
