"""The ``sim_camera`` node: one simulated camera as the real one publishes it.

It reads the simulator's pinhole render of one camera role, resamples it into the
real fisheye image (``fisheye.warp_maps``), and publishes it JPEG-compressed on
``/cam_<role>/image_raw/compressed`` with the real intrinsics on
``/cam_<role>/camera_info``. Both carry the render's stamp (sim time) and the
camera's optical frame. The role's frame, rate and intrinsics come from the
robot's cameras file (``cameras_file.load``).

The depth render of the same camera goes through the same maps, nearest
neighbour (``depth``), onto ``/cam_<role>/depth/compressed``: a 16-bit PNG of
millimetres of z-depth, 0 where there is none. Depth follows colour: a depth
render is published only once the colour render with the same stamp has been,
with that header, so every depth frame has its colour frame. One whose colour
was thinned or skipped is dropped.

With ``depth_sensor`` true it also publishes ``/cam_<role>/depth_sensor/compressed``:
the same depth as a RealSense-like sensor would report it (``depth_sensor.measure``),
in the same format and header, its noise drawn from ``seed``, the role and the stamp.

It publishes at most once per render and at most at the role's ``rate_hz`` (or
the node's ``rate_hz`` parameter, when set),
judged on the render stamps: a render less than three quarters of a period after
the last one published is skipped, so a render rate below ``rate_hz`` passes
through unthinned. A render it cannot use is skipped and logged: the wrong size
or encoding, a buffer that does not match its own size, a stamp before the epoch or more than one
period ahead of the sim clock, or any render before the sim clock runs. When no
render arrives for ``STALL_PERIODS`` periods of sim time it warns.
"""

import sys

import cv2
from fm_sim_cameras import cameras_file, depth, depth_sensor, fisheye
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
        depth_render_topic = self.declare_parameter('depth_render_topic', '').value
        if not depth_render_topic:
            raise cameras_file.CameraError('depth_render_topic is empty')
        # The latest depth render whose colour render has not been published yet.
        self.pending_depth = None
        # A simulated depth sensor on this camera (depth_sensor): off unless a run opts in.
        sensor = self.declare_parameter('depth_sensor', False).value
        # The run's seed (launch-owned): the sensor's noise, frame by frame.
        self.seed = self.declare_parameter('seed', 0).value
        try:
            depth_sensor.frame_rng(self.seed, self.role, 0)
        except ValueError as error:
            raise cameras_file.CameraError(str(error)) from error
        self.quality = int(self.declare_parameter('jpeg_quality', 90).value)
        if not 1 <= self.quality <= 100:
            raise cameras_file.CameraError(
                f'jpeg_quality must be in [1, 100], got {self.quality}'
            )

        self.render = fisheye.render_camera(self.camera)
        map_x, map_y = fisheye.warp_maps(self.camera, self.render)
        # Fixed-point maps remap several times faster than float ones.
        self.colour_maps = cv2.convertMaps(map_x, map_y, cv2.CV_16SC2)
        self.depth_maps = depth.nearest_maps(map_x, map_y)
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
        self.depth_pub = self.create_publisher(
            CompressedImage, cameras_file.depth_topic(self.role), latest
        )
        self.depth_sensor_pub = self.create_publisher(
            CompressedImage, cameras_file.depth_sensor_topic(self.role), latest
        ) if sensor else None
        self.create_subscription(Image, render_topic, self.on_render, latest)
        self.create_subscription(Image, depth_render_topic, self.on_depth_render, latest)
        self.create_timer(self.period_ns * 1e-9, self.check_stalled)
        self.get_logger().info(
            f'{self.role}: {render_topic} ({self.render.width}x{self.render.height}, '
            f'fovy {self.render.fovy_deg:.3f} deg) -> {cameras_file.image_topic(self.role)} '
            f'({self.camera.width}x{self.camera.height} fisheye, frame {self.frame_id}); '
            f'{depth_render_topic} -> {cameras_file.depth_topic(self.role)}'
            + (f', {cameras_file.depth_sensor_topic(self.role)} (seed {self.seed})'
               if sensor else '')
        )

    def _skip(self, reason, kind='render'):
        self.get_logger().warn(
            f'{self.role}: skipping a {kind}: {reason}', throttle_duration_sec=LOG_THROTTLE_S
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
            # A stalled colour render would otherwise hold a whole depth render.
            self.pending_depth = None

    def on_render(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        self.last_render_ns = now_ns
        expected = (self.render.width, self.render.height, RENDER_ENCODING)
        if (msg.width, msg.height, msg.encoding) != expected:
            self._skip(
                f'{msg.width}x{msg.height} {msg.encoding[:32]!r}, expected '
                f'{expected[0]}x{expected[1]} {expected[2]}'
            )
            return
        if msg.step != msg.width * 3 or len(msg.data) != msg.step * msg.height:
            self._skip(f'step {msg.step} and {len(msg.data)} bytes do not fit the size')
            return
        if msg.header.stamp.sec < 0:
            self._skip('stamped before the epoch')
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
            bgr, *self.colour_maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0
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
        self.publish_paired_depth()

    def on_depth_render(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        expected = (self.render.width, self.render.height, depth.RENDER_ENCODING)
        if (msg.width, msg.height, msg.encoding) != expected:
            self._skip(
                f'{msg.width}x{msg.height} {msg.encoding[:32]!r}, expected '
                f'{expected[0]}x{expected[1]} {expected[2]}', 'depth render',
            )
            return
        if msg.step != msg.width * 4 or len(msg.data) != msg.step * msg.height:
            self._skip(
                f'step {msg.step} and {len(msg.data)} bytes do not fit the size', 'depth render'
            )
            return
        if msg.header.stamp.sec < 0:
            self._skip('stamped before the epoch', 'depth render')
            return
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if now_ns == 0:
            self._skip('the sim clock is not running yet', 'depth render')
            return
        if stamp_ns > now_ns + self.period_ns:
            self._skip(
                f'stamped {(stamp_ns - now_ns) * 1e-9:.3f} s ahead of the sim clock',
                'depth render',
            )
            return
        if self.last_published_ns is not None and stamp_ns < self.last_published_ns:
            self.get_logger().debug(
                f'{self.role}: dropping a depth render older than the last colour published',
                throttle_duration_sec=LOG_THROTTLE_S,
            )
            return
        self.pending_depth = msg
        self.publish_paired_depth()

    def publish_paired_depth(self):
        """Publish the pending depth render once its colour render has been published."""
        if self.pending_depth is None or self.last_published_ns is None:
            return
        msg = self.pending_depth
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp_ns > self.last_published_ns:
            return  # its colour render has not arrived yet
        self.pending_depth = None
        if stamp_ns < self.last_published_ns:
            return  # its colour render was thinned or skipped
        dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
        try:
            render_m = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        except ValueError as error:
            self._skip(f'cannot read the buffer: {error}', 'depth render')
            return
        depth_m = depth.warp(render_m, self.depth_maps)
        self._publish_depth(self.depth_pub, depth_m, msg.header.stamp)
        if self.depth_sensor_pub is not None:
            self._publish_sensor_depth(depth_m, stamp_ns, msg.header.stamp)

    def _publish_sensor_depth(self, depth_m, stamp_ns, stamp):
        """Publish what the simulated depth sensor reports of true depth ``depth_m``."""
        rng = depth_sensor.frame_rng(self.seed, self.role, stamp_ns)
        self._publish_depth(self.depth_sensor_pub, depth_sensor.measure(depth_m, rng), stamp)

    def _publish_depth(self, publisher, depth_m, stamp):
        try:
            png = depth.encode(depth.to_millimetres(depth_m))
        except cameras_file.CameraError as error:
            self.get_logger().error(str(error), throttle_duration_sec=LOG_THROTTLE_S)
            return
        compressed = CompressedImage(format=depth.FORMAT, data=png)
        compressed.header.stamp = stamp
        compressed.header.frame_id = self.frame_id
        publisher.publish(compressed)


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
