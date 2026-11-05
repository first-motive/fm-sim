"""The sim_camera node on a fake render: what it publishes, thins and refuses."""

import os
import subprocess
import time

from conftest import CHEST_D, CHEST_K, OPENARM_CAMERAS
import cv2
from fm_sim_cameras import camera_node, cameras_file, depth
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
DEPTH_RENDER_TOPIC = '/test_render/cam_chest/depth'
# A rate slow enough that the stall warning, 5 periods without a colour render,
# cannot release a pending depth render while a test waits on the wall clock.
HOLDING_RATE_HZ = 0.5
LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def _camera(role='chest', cameras_file=OPENARM_CAMERAS, depth_render_topic=DEPTH_RENDER_TOPIC,
            **extra):
    return SimCamera(parameter_overrides=[
        Parameter('role', value=role),
        Parameter('cameras_file', value=cameras_file),
        Parameter('render_topic', value=RENDER_TOPIC),
        Parameter('depth_render_topic', value=depth_render_topic),
        *(Parameter(name, value=value) for name, value in extra.items()),
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


def _depth_render(render, sec, nanosec=0, metres=1.25, width=None):
    width = width or render.width
    pixels = np.full((render.height, width), metres, '<f4')
    msg = Image(height=render.height, width=width, encoding='32FC1', step=width * 4,
                data=pixels.tobytes())
    msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nanosec
    msg.header.frame_id = 'cam_chest_frame'
    return msg


class _Rig:

    def __init__(self, camera):
        self.camera = camera
        self.probe = rclpy.create_node('sim_camera_probe')
        self.images, self.infos, self.depths = [], [], []
        self.probe.create_subscription(
            CompressedImage, '/cam_chest/depth/compressed', self.depths.append, LATEST)
        self.depth_pub = self.probe.create_publisher(Image, DEPTH_RENDER_TOPIC, LATEST)
        self.probe.create_subscription(
            CompressedImage, '/cam_chest/image_raw/compressed', self.images.append, LATEST)
        self.probe.create_subscription(CameraInfo, '/cam_chest/camera_info', self.infos.append, 10)
        self.render_pub = self.probe.create_publisher(Image, RENDER_TOPIC, LATEST)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(camera)
        self.executor.add_node(self.probe)
        self.spin_until(lambda: self.render_pub.get_subscription_count() > 0
                        and self.depth_pub.get_subscription_count() > 0)

    def spin_until(self, done, timeout_s=10.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not done():
            self.executor.spin_once(timeout_sec=0.05)
        return done()

    def send(self, msg, expect):
        self.render_pub.publish(msg)
        return self.spin_until(lambda: len(self.images) >= expect, 5.0)

    def send_depth(self, msg, expect, timeout_s=5.0):
        self.depth_pub.publish(msg)
        return self.spin_until(lambda: len(self.depths) >= expect, timeout_s)

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
         '-p', f'render_topic:={RENDER_TOPIC}',
         '-p', f'depth_render_topic:={DEPTH_RENDER_TOPIC}'],
        capture_output=True, text=True, timeout=60, env=os.environ,
    )
    assert result.returncode != 0
    assert "unknown camera role 'head'" in result.stderr


def test_rate_hz_overrides_the_cameras_file(ros):
    camera = _camera(rate_hz=5.0)
    try:
        assert camera.period_ns == 200_000_000
    finally:
        camera.destroy_node()
    camera = _camera()
    try:
        assert camera.period_ns == int(1e9 / 15)  # the openarm file's chest rate
    finally:
        camera.destroy_node()


@pytest.mark.parametrize('rate_hz', [-1.0, 500.0])
def test_refuses_a_rate_out_of_range(ros, rate_hz):
    with pytest.raises(cameras_file.CameraError, match='rate_hz must be in'):
        _camera(rate_hz=rate_hz)


def _stamp(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)


def test_publishes_fisheye_depth_with_the_colour_header(ros):
    rig = _Rig(_camera())
    try:
        render = rig.camera.render
        assert rig.send(_render(render, 12, 500), 1)
        assert rig.send_depth(_depth_render(render, 12, 500), 1)
        (image,), (depth_image,) = rig.images, rig.depths
        assert depth_image.format == '16UC1; png'
        assert depth_image.header == image.header
        decoded = depth.decode(bytes(depth_image.data))
        assert decoded.dtype == np.uint16
        assert decoded.shape == (1080, 1920)
        # A flat wall of z-depth stays the same z-depth; the corners see nothing.
        assert decoded[540, 960] == 1250 and decoded[540, 400] == 1250
        assert decoded[2, 2] == 0
    finally:
        rig.close()


def test_depth_arriving_first_waits_for_its_colour(ros):
    rig = _Rig(_camera(rate_hz=HOLDING_RATE_HZ))
    try:
        render = rig.camera.render
        assert not rig.send_depth(_depth_render(render, 3), 1, timeout_s=1.0)
        assert rig.send(_render(render, 3), 1)
        assert rig.spin_until(lambda: rig.depths)
        assert _stamp(rig.depths[0]) == _stamp(rig.images[0]) == (3, 0)
    finally:
        rig.close()


def test_depth_of_a_thinned_colour_render_is_dropped(ros):
    rig = _Rig(_camera(rate_hz=5.0))
    try:
        render = rig.camera.render
        assert rig.send(_render(render, 10, 0), 1)
        assert rig.send_depth(_depth_render(render, 10, 0), 1)
        # 0.1 s later: thinned at 5 Hz, so its depth publishes nothing.
        assert not rig.send(_render(render, 10, 100_000_000), 2)
        assert not rig.send_depth(_depth_render(render, 10, 100_000_000), 2)
        assert rig.send(_render(render, 10, 200_000_000), 2)
        assert rig.send_depth(_depth_render(render, 10, 200_000_000), 2)
        assert [_stamp(d) for d in rig.depths] == [(10, 0), (10, 200_000_000)]
    finally:
        rig.close()


def test_skips_a_depth_render_of_the_wrong_size_or_encoding(ros):
    rig = _Rig(_camera())
    try:
        render = rig.camera.render
        assert rig.send(_render(render, 4), 1)
        assert not rig.send_depth(_depth_render(render, 4, width=640), 1)
        wrong_encoding = _depth_render(render, 4)
        wrong_encoding.encoding = '16UC1'
        assert not rig.send_depth(wrong_encoding, 1)
        assert rig.send_depth(_depth_render(render, 4), 1)
    finally:
        rig.close()


def test_refuses_an_empty_depth_render_topic(ros):
    with pytest.raises(cameras_file.CameraError, match='depth_render_topic is empty'):
        _camera(depth_render_topic='')


def test_a_newer_depth_render_replaces_a_pending_one(ros):
    rig = _Rig(_camera(rate_hz=HOLDING_RATE_HZ))
    try:
        render = rig.camera.render
        assert not rig.send_depth(_depth_render(render, 5, 0, metres=1.0), 1, timeout_s=1.0)
        assert not rig.send_depth(
            _depth_render(render, 5, 100_000_000, metres=2.0), 1, timeout_s=1.0)
        assert rig.send(_render(render, 5, 100_000_000), 1)
        assert rig.spin_until(lambda: rig.depths)
        (depth_image,) = rig.depths
        assert _stamp(depth_image) == (5, 100_000_000)
        assert depth.decode(bytes(depth_image.data))[540, 960] == 2000
    finally:
        rig.close()


def test_depth_whose_colour_never_arrives_stays_unpublished(ros):
    rig = _Rig(_camera(rate_hz=HOLDING_RATE_HZ))
    try:
        render = rig.camera.render
        assert rig.send(_render(render, 6), 1)
        assert not rig.send_depth(_depth_render(render, 7), 1, timeout_s=1.0)
        assert rig.camera.pending_depth is not None
        # The stall warning releases it.
        rig.camera.last_render_ns -= (camera_node.STALL_PERIODS + 1) * rig.camera.period_ns
        rig.camera.check_stalled()
        assert rig.camera.pending_depth is None
    finally:
        rig.close()
