"""The fisheye camera model: the render that covers it, and the warp."""

from conftest import OPENARM_CAMERAS
import cv2
from fm_sim_cameras import cameras_file, fisheye
import numpy as np
import pytest


@pytest.fixture
def chest():
    return fisheye.from_intrinsics(cameras_file.load(OPENARM_CAMERAS)['chest']['intrinsics'])


def test_warp_refuses_a_model_it_cannot_invert():
    intrinsics = dict(
        cameras_file.load(OPENARM_CAMERAS)['chest']['intrinsics'],
        distortion_model='plumb_bob', distortion=[0.0] * 5,
    )
    with pytest.raises(cameras_file.CameraError, match="'plumb_bob' cannot be rendered"):
        fisheye.from_intrinsics(intrinsics)


def test_render_reaches_the_edge_midpoints(chest):
    render = fisheye.render_camera(chest)
    assert render.width == chest.width
    assert render.height % 2 == 0
    # CameraPlugin's focal length for this fovy and height is the render's.
    focal = render.height / 2.0 / np.tan(np.radians(render.fovy_deg) / 2.0)
    assert focal == pytest.approx(render.focal)
    map_x, map_y = fisheye.warp_maps(chest, render)
    for row, column in ((573, 958), (574, 959)):
        for u, v in ((0, row), (chest.width - 1, row), (column, 0), (column, chest.height - 1)):
            assert map_x[v, u] >= 0 and map_y[v, u] >= 0
    # The extreme corners look past the render: about 12 % of the frame is black.
    assert map_x[0, 0] == -1.0
    assert 0.10 < float(np.mean(map_x < 0)) < 0.13


def test_warp_inverts_the_opencv_fisheye_model(chest):
    render = fisheye.render_camera(chest)
    map_x, map_y = fisheye.warp_maps(chest, render)
    v, u = np.nonzero(map_x >= 0)
    pick = np.linspace(0, len(u) - 1, 2000).astype(int)
    u, v = u[pick], v[pick]
    # Back from render pixel (OpenCV pixel centres) to the pinhole plane, then
    # through cv2.fisheye's model, as fm_skills_perception uses it.
    x = (map_x[v, u] + 0.5 - render.width / 2.0) / render.focal
    y = (map_y[v, u] + 0.5 - render.height / 2.0) / render.focal
    points = np.stack([x, y], axis=-1).reshape(-1, 1, 2).astype(np.float64)
    distorted = cv2.fisheye.distortPoints(points, chest.k, np.array(chest.distortion))
    assert np.max(np.abs(distorted.reshape(-1, 2) - np.stack([u, v], axis=-1))) < 0.01


def test_principal_point_maps_to_the_render_centre(chest):
    render = fisheye.render_camera(chest)
    x, y = fisheye.rays(chest, [chest.cx], [chest.cy])
    assert (x[0], y[0]) == pytest.approx((0.0, 0.0))
    map_x, map_y = fisheye.warp_maps(chest, render)
    v, u = round(chest.cy), round(chest.cx)
    expected_x = render.focal * (u - chest.cx) / chest.fx + render.width / 2.0 - 0.5
    assert map_x[v, u] == pytest.approx(expected_x, abs=0.01)
