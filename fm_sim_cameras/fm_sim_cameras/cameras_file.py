"""A robot's cameras file: the one loader every backend uses. ROS-free.

The file (a robot entry's ``cameras_file``) maps ``cameras`` to roles::

    cameras:
      chest:
        frame: cam_chest_optical   # optical frame in the description
        rate_hz: 15
        intrinsics: {width, height, fx, fy, cx, cy, distortion_model, distortion}
        allow_large_image: true            # optional; needed above 1920x1080

``check`` is the shared verdict on it, as plain dicts; ``fm_sim_cameras.camera_info``
builds the CameraInfo from a checked camera. Each backend then refuses what it cannot
render on top: MuJoCo's fisheye warp only ``equidistant``
(``fisheye.from_intrinsics``), Isaac a ``rate_hz`` that does not divide its
render rate (``isaac_stage.check_cameras``).
"""

import math
import re

import yaml

# The camera roles of the simulation contracts (§2).
ROLES = ('chest', 'wrist_right')
FRAME_NAME = re.compile(r'[A-Za-z][A-Za-z0-9_]*')
# camera_info distortion models and their coefficient counts.
DISTORTION_MODELS = {'plumb_bob': 5, 'equidistant': 4}
# Rendered size above which a camera must opt in (allow_large_image: true).
MAX_IMAGE_PIXELS = 1920 * 1080


class CameraError(ValueError):
    """A cameras file, role or frame that cannot be used."""


def check_role(role):
    """Return ``role``, refusing a role the contract does not define."""
    if role not in ROLES:
        raise CameraError(f'unknown camera role {role!r}; roles: {", ".join(ROLES)}')
    return role


def check_frame(frame):
    """Return ``frame``, refusing a name that is not a plain frame name."""
    if not isinstance(frame, str) or not FRAME_NAME.fullmatch(frame):
        raise CameraError(f'camera frame {frame!r} is not a valid frame name')
    return frame


def image_topic(role):
    """Return the compressed image topic of ``role``."""
    return f'/cam_{check_role(role)}/image_raw/compressed'


def info_topic(role):
    """Return the camera_info topic of ``role``."""
    return f'/cam_{check_role(role)}/camera_info'


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _positive(value):
    return _finite(value) and value > 0


def _check_intrinsics(intrinsics, where, allow_large_image):
    if not isinstance(intrinsics, dict):
        raise CameraError(f'{where}: intrinsics must be a mapping')
    for key in ('width', 'height'):
        value = intrinsics.get(key)
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            raise CameraError(f'{where}: {key} must be a positive integer, got {value!r}')
    for key in ('fx', 'fy', 'cx', 'cy'):
        if not _positive(intrinsics.get(key)):
            raise CameraError(
                f'{where}: {key} must be a positive number, got {intrinsics.get(key)!r}'
            )
    pixels = intrinsics['width'] * intrinsics['height']
    if pixels > MAX_IMAGE_PIXELS and not allow_large_image:
        raise CameraError(
            f'{where}: {intrinsics["width"]}x{intrinsics["height"]} is above '
            f'{MAX_IMAGE_PIXELS} pixels; set allow_large_image: true to render it'
        )
    model = intrinsics.get('distortion_model')
    if model not in DISTORTION_MODELS:
        raise CameraError(
            f'{where}: unknown distortion_model {model!r}; models: '
            f'{", ".join(sorted(DISTORTION_MODELS))}'
        )
    distortion = intrinsics.get('distortion')
    if (
        not isinstance(distortion, list) or len(distortion) != DISTORTION_MODELS[model]
        or not all(_finite(d) for d in distortion)
    ):
        raise CameraError(
            f'{where}: {model} takes {DISTORTION_MODELS[model]} finite distortion '
            f'coefficients, got {distortion!r}'
        )
    return {
        'width': intrinsics['width'],
        'height': intrinsics['height'],
        **{key: float(intrinsics[key]) for key in ('fx', 'fy', 'cx', 'cy')},
        'distortion_model': model,
        'distortion': [float(d) for d in distortion],
    }


def check(data):
    """Return the cameras of a cameras file's contents as ``{role: camera}``.

    A camera is ``{frame, rate_hz, intrinsics, allow_large_image}``, plain data
    (JSON-serialisable). Refuses a role outside ``ROLES``, a bad frame name, a
    ``rate_hz`` that is not a positive number, an image above
    ``MAX_IMAGE_PIXELS`` without ``allow_large_image: true``, and distortion
    outside ``DISTORTION_MODELS`` or with the wrong coefficient count.
    """
    if not isinstance(data, dict) or not isinstance(data.get('cameras'), dict):
        raise CameraError("a cameras file is a mapping with a 'cameras' mapping")
    checked = {}
    for role, camera in data['cameras'].items():
        check_role(role)
        where = f'camera {role}'
        if not isinstance(camera, dict):
            raise CameraError(f'{where}: must be a mapping')
        frame = camera.get('frame')
        if not isinstance(frame, str) or not FRAME_NAME.fullmatch(frame):
            raise CameraError(f'{where}: frame {frame!r} is not a frame name')
        rate = camera.get('rate_hz')
        if not _positive(rate):
            raise CameraError(f'{where}: rate_hz must be a positive number, got {rate!r}')
        allow_large_image = camera.get('allow_large_image', False)
        if not isinstance(allow_large_image, bool):
            raise CameraError(f'{where}: allow_large_image must be true or false')
        checked[role] = {
            'frame': frame,
            'rate_hz': float(rate),
            'intrinsics': _check_intrinsics(camera.get('intrinsics'), where, allow_large_image),
            'allow_large_image': allow_large_image,
        }
    return checked


def load(path):
    """Return ``check`` of the cameras file at ``path``."""
    with open(path) as cameras_file:
        data = yaml.safe_load(cameras_file)
    try:
        return check(data)
    except CameraError as error:
        raise CameraError(f'{path}: {error}') from error
