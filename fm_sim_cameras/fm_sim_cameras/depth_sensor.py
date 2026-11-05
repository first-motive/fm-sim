"""A RealSense-like depth sensor made from ground-truth depth. ROS-free.

``measure`` turns a camera's exact z-depth (``depth.warp``'s float32 metres, 0
where there is none) into what an active stereo camera of the Intel RealSense
D400 family would report, so a consumer of sensor depth can be tested against
the truth it came from. Four effects, each a module constant:

- **Axial noise** grows with the square of the depth: stereo depth is
  ``f * B / disparity``, so a fixed disparity error ``s`` gives
  ``sigma(z) = z**2 * s / (f * B)``. This is the depth RMS error formula of
  Intel's "Best-Known-Methods for Tuning Intel RealSense D400 Depth Cameras for
  Best Performance" (Grunnet-Jepsen et al., rev 1.9), taken for a D435 at
  848x480: ``s`` 0.1 px (the guide's bound for a well-calibrated camera),
  ``f`` 447 px (848 px over its 87 deg HFOV) and ``B`` 50 mm. That gives
  18 mm at 2 m (0.9 %), inside the D400 datasheet's "under 2 % at 2 m".
- **Edge dropout**: stereo matching fails where the depth jumps, as one
  camera sees background the other cannot. Pixels within ``EDGE_DILATE_PX`` of
  a neighbour jump above ``EDGE_JUMP_M`` are 0, as are those next to no data.
- **Random dropout**: ``INVALID_RATE`` of the remaining pixels, uniformly, read 0
  (unmatched texture). An order of magnitude, not a datasheet number.
- **Range**: true depth outside ``[MIN_RANGE_M, MAX_RANGE_M]`` reads 0 (the
  D435's minimum depth at 848x480 and the end of its stated ideal range).

Noise is drawn from ``frame_rng(seed, role, stamp_ns)``: one frame's noise
depends on the seed, the camera and the frame's stamp only, so a replay or a
test reproduces it whichever other frames were dropped.
"""

import cv2
from fm_sim_cameras.cameras_file import ROLES, check_role
import numpy as np

#: Disparity RMS error (px), focal length (px) and baseline (m): a D435 at 848x480.
SUBPIXEL_RMS_PX = 0.1
FOCAL_PX = 447.0
BASELINE_M = 0.050
#: sigma(z) = AXIAL_NOISE_PER_M * z**2, in metres: about 0.0045 per metre.
AXIAL_NOISE_PER_M = SUBPIXEL_RMS_PX / (FOCAL_PX * BASELINE_M)
#: A depth jump between neighbouring pixels above this is an edge; slanted
#: surfaces in view step well under it (a table at 80 deg, 1 m away: about 8 mm).
EDGE_JUMP_M = 0.05
#: How far from an edge, in pixels, the sensor reports no depth.
EDGE_DILATE_PX = 3
#: The share of the other pixels that read no depth at random.
INVALID_RATE = 0.005
MIN_RANGE_M = 0.2
MAX_RANGE_M = 3.0
#: ``frame_rng``'s seed bound: the run's uint32 seed.
SEED_MAX = 2**32 - 1


def frame_rng(seed, role, stamp_ns):
    """Return the random generator of one camera's frame.

    Refuses a ``seed`` outside ``[0, SEED_MAX]`` and an unknown ``role``.
    """
    if not isinstance(seed, int) or isinstance(seed, bool) or not 0 <= seed <= SEED_MAX:
        raise ValueError(f'seed must be an integer in [0, {SEED_MAX}], got {seed!r}')
    return np.random.default_rng([seed, ROLES.index(check_role(role)), int(stamp_ns)])


def edges(depth_m):
    """Return where the sensor loses depth at an edge: a bool mask of ``depth_m``'s shape.

    A pixel is an edge when a 4-neighbour differs from it by more than
    ``EDGE_JUMP_M`` (no data counts as 0, so the border of no data is one); the
    mask is then dilated by ``EDGE_DILATE_PX``.
    """
    depth_m = np.asarray(depth_m, dtype=np.float32)
    jump = np.zeros(depth_m.shape, np.uint8)
    across = np.abs(np.diff(depth_m, axis=1)) > EDGE_JUMP_M
    down = np.abs(np.diff(depth_m, axis=0)) > EDGE_JUMP_M
    # Both pixels of a jump, so the band is symmetric about the edge.
    jump[:, :-1] |= across
    jump[:, 1:] |= across
    jump[:-1, :] |= down
    jump[1:, :] |= down
    size = 2 * EDGE_DILATE_PX + 1
    return cv2.dilate(jump, np.ones((size, size), np.uint8)).astype(bool)


def measure(depth_m, rng):
    """Return the sensor's z-depth (float32 metres, 0 for none) of true z-depth ``depth_m``.

    ``depth_m`` is 0 where there is no depth, as ``depth.warp`` gives it; ``rng``
    is ``frame_rng``'s generator.
    """
    depth_m = np.asarray(depth_m, dtype=np.float32)
    sigma = AXIAL_NOISE_PER_M * depth_m * depth_m
    noisy = depth_m + sigma * rng.standard_normal(depth_m.shape, dtype=np.float32)
    lost = (
        (depth_m < MIN_RANGE_M) | (depth_m > MAX_RANGE_M) | edges(depth_m)
        | (rng.random(depth_m.shape, dtype=np.float32) < INVALID_RATE)
    )
    return np.where(lost, np.float32(0.0), noisy).astype(np.float32)
