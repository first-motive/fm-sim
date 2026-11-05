"""Fisheye depth from a simulator's pinhole depth render. ROS-free.

MuJoCo's CameraPlugin publishes each camera's depth as ``32FC1`` metres of
z-depth: the distance along the camera's optical axis, linearised from the GL
depth buffer (mujoco_ros2_control 0.1.2 ``camera_plugin.cpp``). The pinhole
render and the fisheye share one optical frame, so a point's z is the same in
both: ``warp`` moves each value to the fisheye pixel whose ray hits it and
leaves it unchanged, which makes the output fisheye z-depth with no conversion.

The warp is nearest neighbour, never a blend: interpolating across an object's
edge would invent a depth between foreground and background that no surface
has. ``encode`` writes a 16-bit PNG of millimetres, 0 where there is no depth.
"""

import cv2
from fm_sim_cameras.cameras_file import CameraError
import numpy as np

RENDER_ENCODING = '32FC1'
#: The CompressedImage ``format``: image_transport's ``compressed`` convention,
#: the decoded image's encoding then the codec. Plain PNG, not ``compressedDepth``
#: (which prefixes a 12-byte quantisation header).
FORMAT = '16UC1; png'
MM_PER_M = 1000.0
#: Depth past this is no data. Where a ray hits nothing, MuJoCo writes the far
#: clip plane (``zfar`` times the model's extent, tens of metres), which would
#: otherwise read as a wall; nothing a tabletop camera sees is this far.
MAX_RANGE_M = 10.0
# Fastest zlib level: a 1920x1080 frame at the camera rate, where size matters less.
_PNG_COMPRESSION = 1


def nearest_maps(map_x, map_y):
    """Return the ``cv2.remap`` map of ``fisheye.warp_maps``' maps for nearest-neighbour use.

    Rounded to the nearest render pixel; the fixed-point maps the colour warp
    uses truncate instead, half a pixel off under ``INTER_NEAREST``.
    """
    rounded, _ = cv2.convertMaps(map_x, map_y, cv2.CV_16SC2, nninterpolation=True)
    return rounded


def warp(render_m, maps):
    """Return the fisheye z-depth (float32 metres) of a pinhole depth render.

    ``maps`` is ``nearest_maps``' output. Pixels whose ray misses the render are 0.
    """
    return cv2.remap(
        np.ascontiguousarray(render_m, dtype=np.float32), maps, None, cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0,
    )


def to_millimetres(depth_m):
    """Return ``depth_m`` as uint16 millimetres, rounded; 0 where it is not a valid depth.

    Not valid: NaN or infinite, at or below 0, or past ``MAX_RANGE_M``.
    """
    depth_m = np.asarray(depth_m, dtype=np.float32)
    with np.errstate(invalid='ignore'):
        valid = np.isfinite(depth_m) & (depth_m > 0) & (depth_m <= MAX_RANGE_M)
    millimetres = np.rint(np.where(valid, depth_m, 0.0) * MM_PER_M)
    return millimetres.astype(np.uint16)


def encode(depth_mm):
    """Return the PNG bytes of a uint16 millimetre image."""
    ok, png = cv2.imencode('.png', depth_mm, [cv2.IMWRITE_PNG_COMPRESSION, _PNG_COMPRESSION])
    if not ok:
        raise CameraError('PNG encoding of the depth image failed')
    return png.tobytes()


def decode(png):
    """Return the uint16 millimetre image of ``encode``'s PNG bytes."""
    return cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
