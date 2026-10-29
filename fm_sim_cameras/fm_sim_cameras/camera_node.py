"""The ``sim_camera`` node: one simulated camera as the real one publishes it.

It reads the simulator's pinhole render of one camera role, resamples it into the
real fisheye image (``fisheye.warp_maps``), and publishes it JPEG-compressed on
``/cam_<role>/image_raw/compressed`` with the real intrinsics on
``/cam_<role>/camera_info``. Both carry the render's stamp (sim time) and the
camera's optical frame. The role's frame, rate and intrinsics come from the
robot's cameras file (``cameras_file.load``).

It publishes at most once per render and at most at the role's ``rate_hz`` (or
the node's ``rate_hz`` parameter, when set),
judged on the render stamps: a render less than three quarters of a period after
the last one published is skipped, so a render rate below ``rate_hz`` passes
through unthinned. A render it cannot use is skipped and logged: the wrong size
or encoding, a buffer that does not match its own size, a stamp more than one
period ahead of the sim clock, or any render before the sim clock runs. When no
render arrives for ``STALL_PERIODS`` periods of sim time it warns.
"""

import sys

import cv2
from fm_sim_cameras import cameras_file, fisheye
from fm_sim_cameras.camera_info import camera_info
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

RENDER_ENCODING = 'rgb8'
JPEG_FORMAT = 'jpeg'
# Past what any renderer here produces; a rate above it would only thin nothing.
MAX_RATE_HZ = 120.0
# Periods of sim time without a render before the node warns that renders stopped.
STALL_PERIODS = 5
LOG_THROTTLE_S = 5.0


class SimCamera(Node):

    def __init__(self, **kwargs):
        super().__init__('sim_camera', **kwargs)
        self.role = cameras_file.check_role(self.declare_parameter('role', '').value)
        cameras = cameras_file.load(self.declare_parameter('cameras_file', '').value)
        if self.role not in cameras:
            raise cameras_file.CameraError(f'the cameras file has no {self.role!r} camera')
        camera = cameras[self.role]
        self.frame_id = camera['frame']
        self.camera = fisheye.from_intrinsics(camera['intrinsics'])
        # A run may override the cameras file's rate (a run param); 0 keeps it.
        rate_hz = float(self.declare_parameter('rate_hz', 0.0).value) or camera['rate_hz']
        if not 0.0 < rate_hz <= MAX_RATE_HZ:
            raise cameras_file.CameraError(f'rate_hz must be in (0, {MAX_RATE_HZ}]')
        self.period_ns = int(1e9 / rate_hz)
        self.min_gap_ns = int(0.75 * self.period_ns)
        self.last_published_ns = None
        self.last_render_ns = None
        render_topic = self.declare_parameter('render_topic', '').value
        if not render_topic:
            raise cameras_file.CameraError('render_topic is empty')
        self.quality = int(self.declare_parameter('jpeg_quality', 90).value)
        if not 1 <= self.quality <= 100:
            raise cameras_file.CameraError(
                f'jpeg_quality must be in [1, 100], got {self.quality}'
            )

        self.render = fisheye.render_camera(self.camera)
        map_x, map_y = fisheye.warp_maps(self.camera, self.render)
        # Fixed-point maps remap several times faster than float ones.
        self.maps = cv2.convertMaps(map_x, map_y, cv2.CV_16SC2)
        self.info = camera_info(camera['intrinsics'])
        self.info.header.frame_id = self.frame_id

        # Reliable, depth 1, both ways: never a stale frame queued, and a render is
        # several megabytes, whose fragments best effort loses often enough to
        # drop most frames. The publishers match reliable and best-effort
        # subscribers alike.
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.image_pub = self.create_publisher(
            CompressedImage, cameras_file.image_topic(self.role), latest
        )
        self.info_pub = self.create_publisher(
            CameraInfo, cameras_file.info_topic(self.role), latest
        )
        self.create_subscription(Image, render_topic, self.on_render, latest)
        self.create_timer(self.period_ns * 1e-9, self.check_stalled)
        self.get_logger().info(
            f'{self.role}: {render_topic} ({self.render.width}x{self.render.height}, '
            f'fovy {self.render.fovy_deg:.3f} deg) -> {cameras_file.image_topic(self.role)} '
            f'({self.camera.width}x{self.camera.height} fisheye, frame {self.frame_id})'
        )

    def _skip(self, reason):
        self.get_logger().warn(
            f'{self.role}: skipping a render: {reason}', throttle_duration_sec=LOG_THROTTLE_S
        )

    def check_stalled(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.last_render_ns is None:
            self.last_render_ns = now_ns
            return
        if now_ns - self.last_render_ns > STALL_PERIODS * self.period_ns:
            self.get_logger().warn(
                f'{self.role}: no render for {(now_ns - self.last_render_ns) * 1e-9:.1f} s '
                'of sim time', throttle_duration_sec=LOG_THROTTLE_S,
            )

    def on_render(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        self.last_render_ns = now_ns
        expected = (self.render.width, self.render.height, RENDER_ENCODING)
        if (msg.width, msg.height, msg.encoding) != expected:
            self._skip(
                f'{msg.width}x{msg.height} {msg.encoding}, expected '
                f'{expected[0]}x{expected[1]} {expected[2]}'
            )
            return
        if msg.step != msg.width * 3 or len(msg.data) != msg.step * msg.height:
            self._skip(f'step {msg.step} and {len(msg.data)} bytes do not fit the size')
            return
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if now_ns == 0:
            self._skip('the sim clock is not running yet')
            return
        if stamp_ns > now_ns + self.period_ns:
            self._skip(f'stamped {(stamp_ns - now_ns) * 1e-9:.3f} s ahead of the sim clock')
            return
        if self.last_published_ns is not None and (
            0 <= stamp_ns - self.last_published_ns < self.min_gap_ns
        ):
            return  # thinning to rate_hz, the normal case: not logged
        try:
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        except ValueError as error:
            self._skip(f'cannot read the buffer: {error}')
            return
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        image = cv2.remap(
            bgr, *self.maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0
        )
        ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            self.get_logger().error('JPEG encoding failed', throttle_duration_sec=LOG_THROTTLE_S)
            return
        compressed = CompressedImage(format=JPEG_FORMAT, data=jpeg.tobytes())
        compressed.header.stamp = msg.header.stamp
        compressed.header.frame_id = self.frame_id
        self.info.header.stamp = msg.header.stamp
        self.last_published_ns = stamp_ns
        self.image_pub.publish(compressed)
        self.info_pub.publish(self.info)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SimCamera()
    except (cameras_file.CameraError, OSError) as error:
        print(f'sim_camera: {error}', file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
