"""A real fisheye camera from a simulator's pinhole render. ROS-free.

Simulators render pinhole images; the OpenArm cameras are fisheyes (cv2.fisheye's
equidistant model). A sim camera therefore renders a wider pinhole view, the
``render_camera``, and ``warp_maps`` resamples it into the real camera's image, so
every pixel looks along the ray the real camera's intrinsics give it.

The render follows the MuJoCo camera convention mujoco_ros2_control's CameraPlugin
uses: square pixels, the principal point at the image centre, and a focal length
of ``height / 2 / tan(fovy / 2)``. The render covers the rays through the
midpoints of the fisheye image's four edges; pixels beyond that (towards the
corners) look outside the render and stay black.
"""

from dataclasses import dataclass
import math

from fm_sim_cameras.cameras_file import CameraError
import numpy as np

DISTORTION_MODEL = 'equidistant'
# Newton steps that invert the fisheye model; it converges in a handful.
_NEWTON_STEPS = 30
_NEWTON_TOLERANCE = 1e-9


@dataclass(frozen=True)
class Fisheye:
    """Real intrinsics: image size (px), K (px) and cv2.fisheye's k1..k4."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple

    @property
    def k(self):
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])


@dataclass(frozen=True)
class Pinhole:
    """The pinhole render a simulator draws for one camera."""

    width: int
    height: int
    fovy_deg: float

    @property
    def focal(self):
        return self.height / 2.0 / math.tan(math.radians(self.fovy_deg) / 2.0)


def from_intrinsics(intrinsics):
    """Return the ``Fisheye`` of a camera's checked intrinsics (``cameras_file.check``).

    Refuses any distortion model but equidistant: it is the only one the warp
    inverts.
    """
    model = intrinsics['distortion_model']
    if model != DISTORTION_MODEL:
        raise CameraError(
            f'distortion_model {model!r} cannot be rendered by the fisheye warp; '
            f'only {DISTORTION_MODEL!r}'
        )
    return Fisheye(
        intrinsics['width'], intrinsics['height'], intrinsics['fx'], intrinsics['fy'],
        intrinsics['cx'], intrinsics['cy'], tuple(intrinsics['distortion']),
    )


def _theta(fisheye, theta_d):
    """Invert theta_d = theta (1 + k1 theta^2 + ... + k4 theta^8); NaN where it fails."""
    k1, k2, k3, k4 = fisheye.distortion
    theta = theta_d.copy()
    for _ in range(_NEWTON_STEPS):
        t2 = theta * theta
        value = theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4)))) - theta_d
        slope = 1 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * (7 * k3 + t2 * 9 * k4)))
        theta = theta - value / slope
    t2 = theta * theta
    residual = theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4)))) - theta_d
    slope = 1 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * (7 * k3 + t2 * 9 * k4)))
    # A ray at or past 90 degrees has no pinhole image; a negative slope is the
    # model folding back on itself, outside any calibrated field.
    good = (np.abs(residual) < _NEWTON_TOLERANCE) & (slope > 0) & (theta >= 0) & (
        theta < math.pi / 2
    )
    return np.where(good, theta, np.nan)


def rays(fisheye, u, v):
    """Return the pinhole-plane ray ``(x, y)`` of fisheye pixels ``(u, v)``; NaN if none."""
    xd = (np.asarray(u, dtype=float) - fisheye.cx) / fisheye.fx
    yd = (np.asarray(v, dtype=float) - fisheye.cy) / fisheye.fy
    theta_d = np.hypot(xd, yd)
    theta = _theta(fisheye, theta_d)
    with np.errstate(divide='ignore', invalid='ignore'):
        scale = np.where(theta_d > 1e-12, np.tan(theta) / theta_d, 1.0)
    return xd * scale, yd * scale


def render_camera(fisheye):
    """Return the ``Pinhole`` render, as wide as the fisheye image, that ``warp_maps`` reads.

    Its field reaches the rays through the pixels at the midpoints of the fisheye
    image's four edges; its height is rounded up to an even number of pixels.
    """
    last_u, last_v = fisheye.width - 1, fisheye.height - 1
    # The pixel rows and columns either side of the principal point, so the
    # midpoint pixels themselves are covered, not just the exact midpoints.
    rows = [math.floor(fisheye.cy), math.ceil(fisheye.cy)]
    columns = [math.floor(fisheye.cx), math.ceil(fisheye.cx)]
    x, _ = rays(fisheye, [0.0, 0.0, last_u, last_u], rows + rows)
    _, y = rays(fisheye, columns + columns, [0.0, 0.0, last_v, last_v])
    if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
        raise CameraError('the fisheye edges look at or past 90 degrees; no pinhole covers them')
    # Those rays land on the centres of the render's edge pixels, half a pixel in
    # from its borders (see warp_maps).
    focal = (fisheye.width / 2.0 - 0.5) / float(np.max(np.abs(x)))
    height = 2 * math.ceil(float(np.max(np.abs(y))) * focal + 0.5)
    fovy = math.degrees(2.0 * math.atan(height / 2.0 / focal))
    return Pinhole(fisheye.width, height, fovy)


def warp_maps(fisheye, pinhole):
    """Return ``(map_x, map_y)`` for ``cv2.remap(render, map_x, map_y, ...)``.

    Both are float32 of the fisheye image's shape: for each fisheye pixel, where
    in the pinhole render its ray lands, in OpenCV pixel-centre coordinates. A
    pixel whose ray falls outside the render maps to -1 (black under
    ``BORDER_CONSTANT``).
    """
    v, u = np.mgrid[0:fisheye.height, 0:fisheye.width].astype(float)
    x, y = rays(fisheye, u, v)
    focal = pinhole.focal
    # The render's principal point is its centre, width / 2 in GL's pixel-edge
    # coordinates: half a pixel less in OpenCV's pixel-centre ones.
    map_x = focal * x + pinhole.width / 2.0 - 0.5
    map_y = focal * y + pinhole.height / 2.0 - 0.5
    inside = (
        np.isfinite(map_x) & np.isfinite(map_y)
        & (map_x >= 0) & (map_x <= pinhole.width - 1)
        & (map_y >= 0) & (map_y <= pinhole.height - 1)
    )
    map_x = np.where(inside, map_x, -1.0).astype(np.float32)
    map_y = np.where(inside, map_y, -1.0).astype(np.float32)
    return map_x, map_y
