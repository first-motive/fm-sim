"""Fisheye depth: the nearest-neighbour warp, the millimetre conversion and the PNG."""

from conftest import OPENARM_CAMERAS
from fm_sim_cameras import cameras_file, depth, fisheye
import numpy as np
import pytest


@pytest.fixture
def chest():
    return fisheye.from_intrinsics(cameras_file.load(OPENARM_CAMERAS)['chest']['intrinsics'])


def test_millimetres_round_trip_through_the_png():
    depth_m = np.array([[0.001, 0.4567, 1.0, depth.MAX_RANGE_M]], np.float32)
    png = depth.encode(depth.to_millimetres(depth_m))
    assert png[:8] == b'\x89PNG\r\n\x1a\n'
    decoded = depth.decode(png)
    assert decoded.dtype == np.uint16
    assert decoded.tolist() == [[1, 457, 1000, 10000]]


@pytest.mark.parametrize('invalid', [np.nan, np.inf, -np.inf, 0.0, -0.5, depth.MAX_RANGE_M + 0.01])
def test_invalid_depth_is_zero(invalid):
    millimetres = depth.to_millimetres(np.array([[invalid, 1.5]], np.float32))
    assert millimetres.tolist() == [[0, 1500]]


def test_the_far_plane_mujoco_writes_for_no_hit_is_zero():
    assert depth.to_millimetres(np.array([50.0], np.float32)).tolist() == [0]


def test_warp_takes_the_nearest_render_pixel_and_never_blends(chest):
    render = fisheye.render_camera(chest)
    maps_x, maps_y = fisheye.warp_maps(chest, render)
    # A step edge: near on the left half of the render, far on the right.
    render_m = np.full((render.height, render.width), 2.0, np.float32)
    render_m[:, :render.width // 2] = 0.5
    warped = depth.warp(render_m, depth.nearest_maps(maps_x, maps_y))
    assert warped.shape == (chest.height, chest.width)
    assert set(np.unique(warped).tolist()) == {0.0, 0.5, 2.0}
    # Pixels whose ray misses the render (the corners) carry no depth.
    assert np.array_equal(warped == 0.0, maps_x < 0)


def test_warp_rounds_to_the_nearest_render_pixel(chest):
    render = fisheye.render_camera(chest)
    map_x, map_y = fisheye.warp_maps(chest, render)
    # Every render pixel a distinct value: its index, so the source pixel is readable.
    render_m = np.arange(1, render.width * render.height + 1, dtype=np.float64).reshape(
        render.height, render.width
    ).astype(np.float32)
    warped = depth.warp(render_m, depth.nearest_maps(map_x, map_y))
    v, u = np.nonzero(map_x >= 0)
    pick = np.linspace(0, len(u) - 1, 2000).astype(int)
    u, v = u[pick], v[pick]
    rows = np.clip(np.rint(map_y[v, u]), 0, render.height - 1).astype(int)
    columns = np.clip(np.rint(map_x[v, u]), 0, render.width - 1).astype(int)
    # float32 holds the indices exactly up to 2**24; the render has fewer pixels.
    assert render.width * render.height < 2**24
    assert np.array_equal(warped[v, u], render_m[rows, columns])


def test_warp_keeps_z_depth_unchanged(chest):
    # The render and the fisheye share an optical frame: a flat wall at z = 1.25 m
    # is 1.25 m of z-depth at every fisheye pixel that sees it, not ray distance.
    render = fisheye.render_camera(chest)
    map_x, map_y = fisheye.warp_maps(chest, render)
    render_m = np.full((render.height, render.width), 1.25, np.float32)
    millimetres = depth.to_millimetres(depth.warp(render_m, depth.nearest_maps(map_x, map_y)))
    assert set(np.unique(millimetres[map_x >= 0]).tolist()) == {1250}
