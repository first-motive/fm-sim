"""The CameraInfo of a checked camera, shared by every backend's camera nodes."""

from sensor_msgs.msg import CameraInfo


def camera_info(intrinsics):
    """Return the CameraInfo (header unset) of a camera's ``intrinsics``.

    ``intrinsics`` are as ``cameras_file.check`` returns them.
    """
    fx, fy, cx, cy = (intrinsics[key] for key in ('fx', 'fy', 'cx', 'cy'))
    info = CameraInfo()
    info.width = intrinsics['width']
    info.height = intrinsics['height']
    info.distortion_model = intrinsics['distortion_model']
    info.d = list(intrinsics['distortion'])
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info
