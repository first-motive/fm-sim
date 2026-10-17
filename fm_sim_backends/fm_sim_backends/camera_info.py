"""camera_info node: the real intrinsics for every rendered camera image.

A sim renders each camera on ``/cam_<role>/image_raw``; this node answers every
image with a ``/cam_<role>/camera_info`` carrying the same header and the
camera's intrinsics from the robot's cameras file, so consumers see exactly the
calibration the image was rendered with. An image whose size is not the
calibrated one gets no camera_info (logged): intrinsics for another size would
be wrong.

Parameter ``cameras``: JSON ``{role: camera}`` as
``fm_sim_scenarios.isaac_stage.check_cameras`` returns it. The CameraInfo comes
from ``fm_sim_cameras.camera_info``, the builder every backend shares.
"""

import json

from fm_sim_cameras.camera_info import camera_info
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoNode(Node):

    def __init__(self, **kwargs):
        super().__init__('sim_camera_info', **kwargs)
        cameras = json.loads(self.declare_parameter('cameras', '').value or 'null')
        if not isinstance(cameras, dict) or not cameras:
            raise ValueError("parameter 'cameras' must be a JSON object of cameras by role")
        for role, camera in sorted(cameras.items()):
            template = camera_info(camera['intrinsics'])
            publisher = self.create_publisher(CameraInfo, f'/cam_{role}/camera_info', 10)
            self.create_subscription(
                Image, f'/cam_{role}/image_raw',
                lambda image, r=role, t=template, p=publisher: self._on_image(r, t, p, image),
                10,
            )
        self.get_logger().info(f'camera_info for {", ".join(sorted(cameras))}')

    def _on_image(self, role, template, publisher, image):
        if (image.width, image.height) != (template.width, template.height):
            self.get_logger().error(
                f'cam_{role}: image is {image.width}x{image.height}, calibrated for '
                f'{template.width}x{template.height}; no camera_info',
                throttle_duration_sec=5.0,
            )
            return
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = template.width, template.height
        info.distortion_model = template.distortion_model
        info.d, info.k, info.r, info.p = template.d, template.k, template.r, template.p
        publisher.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
