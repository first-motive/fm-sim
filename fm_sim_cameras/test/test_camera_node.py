"""The sim_camera node on a fake render: what it publishes, thins and refuses."""

import os
import subprocess
import time

from conftest import CHEST_D, CHEST_K, OPENARM_CAMERAS
import cv2
from fm_sim_cameras import camera_node, cameras_file
from fm_sim_cameras.camera_node import SimCamera
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
import yaml

RENDER_TOPIC = '/test_render/cam_chest/color'
LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def _camera(role='chest', cameras_file=OPENARM_CAMERAS):
    return SimCamera(parameter_overrides=[
        Parameter('role', value=role),
        Parameter('cameras_file', value=cameras_file),
        Parameter('render_topic', value=RENDER_TOPIC),
    ])


def _render(render, sec, nanosec=0, width=None, step_pad=0):
    width = width or render.width
    # Left half red, right half blue (rgb8), so the warp's orientation shows.
    pixels = np.zeros((render.height, width, 3), np.uint8)
    pixels[:, :width // 2] = (255, 0, 0)
    pixels[:, width // 2:] = (0, 0, 255)
    rows = np.zeros((render.height, width * 3 + step_pad), np.uint8)
    rows[:, :width * 3] = pixels.reshape(render.height, width * 3)
    msg = Image(height=render.height, width=width, encoding='rgb8', step=width * 3 + step_pad,
                data=rows.tobytes())
    msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
    msg.header.frame_id = 'cam_chest_frame'
    return msg


class _Rig:

    def __init__(self, camera):
        self.camera = camera
        self.probe = rclpy.create_node('sim_camera_probe')
        self.images, self.infos = [], []
        self.probe.create_subscription(
            CompressedImage, '/cam_chest/image_raw/compressed', self.images.append, LATEST)
        self.probe.create_subscription(CameraInfo, '/cam_chest/camera_info', self.infos.append, 10)
        self.render_pub = self.probe.create_publisher(Image, RENDER_TOPIC, LATEST)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(camera)
        self.executor.add_node(self.probe)
        self.spin_until(lambda: self.render_pub.get_subscription_count() > 0)

    def spin_until(self, done, timeout_s=10.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not done():
            self.executor.spin_once(timeout_sec=0.05)
        return done()

    def send(self, msg, expect):
        self.render_pub.publish(msg)
        return self.spin_until(lambda: len(self.images) >= expect, 5.0)

    def close(self):
        self.camera.destroy_node()
        self.probe.destroy_node()


def test_publishes_the_fisheye_jpeg_and_real_camera_info(ros):
    rig = _Rig(_camera())
    try:
        render = rig.camera.render
        assert rig.send(_render(render, 12, 500), 1)
        assert rig.spin_until(lambda: rig.infos)
        image, info = rig.images[0], rig.infos[0]
        assert image.format == 'jpeg'
        assert image.header.frame_id == 'cam_chest_optical'
        assert (image.header.stamp.sec, image.header.stamp.nanosec) == (12, 500)
        assert bytes(image.data[:2]) == b'\xff\xd8'
        decoded = cv2.imdecode(np.frombuffer(bytes(image.data), np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape == (1080, 1920, 3)
        # Red stays left, blue right (BGR after decoding); the corners are black.
        left, right = decoded[540, 400], decoded[540, 1500]
        assert left[2] > 200 and left[0] < 50
        assert right[0] > 200 and right[2] < 50
        assert decoded[2, 2].max() < 20
        assert info.header == image.header
        assert list(info.k) == pytest.approx(CHEST_K)
        assert list(info.d) == pytest.approx(CHEST_D)
        assert info.distortion_model == 'equidistant'
        assert (info.width, info.height) == (1920, 1080)
    finally:
        rig.close()


def test_skips_a_render_of_the_wrong_size(ros):
    rig = _Rig(_camera())
    try:
        assert not rig.send(_render(rig.camera.render, 1, width=640), 1)
        assert rig.send(_render(rig.camera.render, 2), 1)
    finally:
        rig.close()


def test_skips_a_render_whose_buffer_does_not_fit(ros):
    rig = _Rig(_camera())
    try:
        render = rig.camera.render
        assert not rig.send(_render(render, 1, step_pad=4), 1)
        short = _render(render, 2)
        short.data = bytes(short.data)[:-3]
        assert not rig.send(short, 1)
        assert rig.send(_render(render, 3), 1)
    finally:
        rig.close()


def test_skips_a_render_stamped_ahead_of_the_clock(ros):
    rig = _Rig(_camera())
    try:
        render = rig.camera.render
        now = rig.camera.get_clock().now().seconds_nanoseconds()[0]
        assert not rig.send(_render(render, now + 10), 1)
        assert rig.send(_render(render, now - 1), 1)
        # A later render from the past still publishes: the future one never counted.
        assert rig.send(_render(render, now), 2)
    finally:
        rig.close()


class _Logger:

    def __init__(self):
        self.warnings = []

    def warn(self, message, **kwargs):
        self.warnings.append(message)


def test_warns_when_renders_stop(ros, monkeypatch):
    camera = _camera()
    try:
        logger = _Logger()
        monkeypatch.setattr(camera, 'get_logger', lambda: logger)
        camera.check_stalled()  # the first tick only starts the count
        assert logger.warnings == []
        camera.last_render_ns -= (camera_node.STALL_PERIODS + 1) * camera.period_ns
        camera.check_stalled()
        (warning,) = logger.warnings
        assert 'no render for' in warning
    finally:
        camera.destroy_node()


def test_thins_renders_above_the_rate(ros, tmp_path):
    with open(OPENARM_CAMERAS) as openarm_file:
        data = yaml.safe_load(openarm_file)
    data['cameras']['chest']['rate_hz'] = 5.0
    slow = tmp_path / 'cameras.yaml'
    slow.write_text(yaml.safe_dump(data))
    rig = _Rig(_camera(cameras_file=str(slow)))
    try:
        render = rig.camera.render
        # Renders every 0.1 s of sim time; 5 Hz keeps every other one.
        stamps = [(10, 0), (10, 100_000_000), (10, 200_000_000), (10, 300_000_000),
                  (10, 400_000_000)]
        for sec, nanosec in stamps:
            rig.render_pub.publish(_render(render, sec, nanosec))
            rig.spin_until(lambda: False, 0.4)
        assert [(i.header.stamp.sec, i.header.stamp.nanosec) for i in rig.images] == [
            (10, 0), (10, 200_000_000), (10, 400_000_000)]
    finally:
        rig.close()


@pytest.mark.parametrize('role', ['wrist_left', '../chest', ''])
def test_refuses_an_unknown_role(ros, role):
    with pytest.raises(cameras_file.CameraError, match='unknown camera role'):
        _camera(role=role)


def test_refuses_a_role_the_cameras_file_lacks(ros, tmp_path):
    with open(OPENARM_CAMERAS) as openarm_file:
        data = yaml.safe_load(openarm_file)
    del data['cameras']['wrist_right']
    path = tmp_path / 'cameras.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(cameras_file.CameraError, match="no 'wrist_right' camera"):
        _camera(role='wrist_right', cameras_file=str(path))


def test_executable_exits_nonzero_on_a_bad_role():
    result = subprocess.run(
        ['ros2', 'run', 'fm_sim_cameras', 'sim_camera', '--ros-args',
         '-p', 'role:=head', '-p', f'cameras_file:={OPENARM_CAMERAS}',
         '-p', f'render_topic:={RENDER_TOPIC}'],
        capture_output=True, text=True, timeout=60, env=os.environ,
    )
    assert result.returncode != 0
    assert "unknown camera role 'head'" in result.stderr
