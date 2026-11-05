"""ChArUco boards: the yaml checks, the board frame, the textures and the boards MuJoCo renders.

The board layout is checked against OpenCV's own ChArUco layout, written out here
rather than taken from OpenCV, so the tests hold whatever OpenCV the image has: a
square is black where its row plus column is even, every other square holds a
marker, and markers are numbered row by row from ``first_marker_id``.
"""

import math
import os

from conftest import ARM_MJCF, CATALOG_SCENARIOS
from fm_sim_scenarios import gazebo_world, isaac_stage, mujoco_world, scenario as sc
import numpy as np
import pytest
import yaml

# The binding's headless GL; a host that has none skips the render test.
os.environ.setdefault('MUJOCO_GL', 'egl')
cv2 = pytest.importorskip('cv2', reason='python3-opencv')

MOUNT = [0.0, 0.0, 0.698]
# How far a found marker corner may sit from where the declared pose puts it.
TEXTURE_TOLERANCE_PX = 1.5
RENDER_TOLERANCE_PX = 2.0


@pytest.fixture
def tags():
    with open(os.path.join(CATALOG_SCENARIOS, 'tabletop_tags.yaml')) as entry_file:
        return yaml.safe_load(entry_file)


def _boards(scenario):
    return [obj for obj in scenario['objects'] if 'charuco' in obj]


def _markers(board):
    """``{id: four corners}`` in the board frame (m, x along a row, y down the columns)."""
    columns, rows = board['squares']
    side = board['square_mm'] / 1000.0
    marker = side * board['marker_fraction']
    inset = (side - marker) / 2
    ids = iter(sc.charuco_marker_ids(board))
    corners = {}
    for row in range(rows):
        for column in range(columns):
            if (row + column) % 2 == 0:
                continue  # A black square.
            x, y = column * side + inset, row * side + inset
            # Top-left, then clockwise, as ArUco orders a marker's corners.
            corners[next(ids)] = np.array(
                [[x, y], [x + marker, y], [x + marker, y + marker], [x, y + marker]]
            )
    return corners


def _detect(grey, dictionary):
    """``{id: four pixel corners}`` of every marker in ``grey``, on OpenCV 4.6 or later."""
    vocabulary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(vocabulary, cv2.aruco.DetectorParameters())
        corners, ids, _rejected = detector.detectMarkers(grey)
    else:
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            grey, vocabulary, parameters=cv2.aruco.DetectorParameters_create()
        )
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}


def _matrix(xyz, quat_xyzw):
    x, y, z, w = quat_xyzw
    transform = np.eye(4)
    transform[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    transform[:3, 3] = xyz
    return transform


# The yaml checks.

def test_the_tags_scenario_passes_the_checks(tags):
    sc.check(tags)
    assert [obj['name'] for obj in _boards(tags)] == ['board_ahead', 'board_left', 'board_right']


def test_a_box_not_the_size_of_its_squares_is_refused(tags):
    _boards(tags)[0]['size'][0] = 0.39
    with pytest.raises(sc.ScenarioError, match='board_ahead: a 8 x 5 board of 50 mm squares'):
        sc.check(tags)


def test_two_boards_sharing_a_marker_id_are_refused(tags):
    _boards(tags)[2]['charuco']['first_marker_id'] = 0
    with pytest.raises(sc.ScenarioError, match='board_ahead and board_right both number marker 0'):
        sc.check(tags)


def test_a_board_numbers_one_marker_per_white_square():
    board = {'squares': [8, 5], 'first_marker_id': 40}
    assert list(sc.charuco_marker_ids(board)) == list(range(40, 60))


@pytest.mark.parametrize('generate', [
    lambda scenario: gazebo_world.generate_world(scenario, 0),
    lambda scenario: isaac_stage.generate(scenario, 0),
], ids=['gazebo', 'isaac'])
def test_only_mujoco_builds_boards(tags, generate):
    with pytest.raises(sc.ScenarioError, match='are ChArUco boards, which only the mujoco'):
        generate(tags)


# The board frame.

def test_the_board_frame_sits_at_the_top_faces_minus_x_plus_y_corner():
    obj = {'size': [0.4, 0.25, 0.002], 'pose': {'xyz': [1.0, 2.0, 3.0]}}
    xyz, quat = sc.charuco_board_frame(obj)
    frame = _matrix(xyz, quat)
    assert xyz == pytest.approx((0.8, 2.125, 3.001))
    # x along a row (+x), y down the columns (-y), z into the board (-z).
    assert frame[:3, :3] == pytest.approx(np.diag([1.0, -1.0, -1.0]))
    # The board's far corner is the top face's +x, -y corner.
    assert (frame @ [0.4, 0.25, 0.0, 1.0])[:3] == pytest.approx([1.2, 1.875, 3.001])


def test_the_board_frame_turns_with_the_box():
    obj = {'size': [0.4, 0.25, 0.002],
           'pose': {'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, math.pi / 2]}}
    xyz, quat = sc.charuco_board_frame(obj)
    frame = _matrix(xyz, quat)
    # Turned a quarter about z, the box's -x, +y corner is at world (-0.125, -0.2).
    assert xyz == pytest.approx((-0.125, -0.2, 0.001))
    assert frame[:3, 0] == pytest.approx([0.0, 1.0, 0.0])
    assert frame[:3, 2] == pytest.approx([0.0, 0.0, -1.0])


# The textures.

def test_every_board_has_its_texture(tags):
    for obj in _boards(tags):
        assert os.path.isfile(mujoco_world.charuco_texture(obj['charuco'])), obj['name']


def test_each_texture_is_its_declared_board(tags):
    for obj in _boards(tags):
        board = obj['charuco']
        grey = cv2.imread(mujoco_world.charuco_texture(board), cv2.IMREAD_GRAYSCALE)
        columns, rows = board['squares']
        square_px = grey.shape[1] / columns
        assert grey.shape[0] == pytest.approx(rows * square_px), obj['name']
        # A white surround, as the table gives the board in the world.
        border = int(square_px)
        found = _detect(cv2.copyMakeBorder(grey, *[border] * 4, cv2.BORDER_CONSTANT, value=255),
                        board['dictionary'])
        expected = _markers(board)
        assert sorted(found) == sorted(expected), obj['name']
        metres_to_px = square_px / (board['square_mm'] / 1000.0)
        for marker, corners in expected.items():
            # OpenCV pixel centres: the image's edge is at -0.5.
            pixels = corners * metres_to_px - 0.5 + border
            assert found[marker] == pytest.approx(pixels, abs=TEXTURE_TOLERANCE_PX), marker


# The world.

def test_each_board_is_a_world_geom_at_its_declared_pose_with_its_texture(tags):
    mujoco = pytest.importorskip('mujoco', reason='mujoco Python binding (requirements-sim.txt)')
    model = mujoco.MjModel.from_xml_string(mujoco_world.generate(ARM_MJCF, MOUNT, tags, 0))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for obj in _boards(tags):
        geom = model.geom(obj['name'])
        xyz, quat = sc.static_poses(tags)[obj['name']]
        assert geom.bodyid[0] == 0
        assert data.geom(obj['name']).xpos == pytest.approx(xyz)
        assert data.geom(obj['name']).xmat.reshape(3, 3) == pytest.approx(
            _matrix(xyz, quat)[:3, :3], abs=1e-9)
        assert geom.size == pytest.approx([s / 2 for s in obj['size']])
        material = model.material(geom.matid[0])
        assert material.name == f'{obj["name"]}_charuco'
        assert material.texid[mujoco.mjtTextureRole.mjTEXROLE_RGB] >= 0


def test_a_board_without_its_texture_is_refused(tags, monkeypatch, tmp_path):
    monkeypatch.setattr(mujoco_world, 'CHARUCO_TEXTURES', str(tmp_path))
    with pytest.raises(sc.ScenarioError, match='board_ahead: no texture'):
        mujoco_world.generate(ARM_MJCF, MOUNT, tags, 0)


def _camera_above(xyz, height, fovy_deg, resolution):
    """A world camera ``height`` above ``xyz`` looking straight down, image right along +x."""
    x, y, z = xyz
    return mujoco_world.Camera(
        'cam_above', None, (x, y, z + height), (0.0, 1.0, 0.0, 0.0), fovy_deg, resolution
    )


def test_a_camera_finds_every_marker_where_the_declared_pose_puts_it(tags, tmp_path):
    mujoco = pytest.importorskip('mujoco', reason='mujoco Python binding (requirements-sim.txt)')
    width, height, fovy = 1280, 960, 60.0
    focal = height / 2.0 / math.tan(math.radians(fovy) / 2.0)
    for obj in _boards(tags):
        xyz, _quat = sc.static_poses(tags)[obj['name']]
        camera = _camera_above(xyz, 0.45, fovy, (width, height))
        model = mujoco.MjModel.from_xml_string(
            mujoco_world.generate(ARM_MJCF, MOUNT, tags, 0, world_cameras=[camera]))
        model.vis.global_.offwidth, model.vis.global_.offheight = width, height
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        try:
            renderer = mujoco.Renderer(model, height, width)
        except Exception as error:  # noqa: B902 - no GL on this host
            pytest.skip(f'no MuJoCo rendering here: {error}')
        renderer.update_scene(data, camera='cam_above')
        grey = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2GRAY)
        renderer.close()

        board = obj['charuco']
        expected = _markers(board)
        # A neighbouring board's markers can show at the image's edge.
        found = {k: v for k, v in _detect(grey, board['dictionary']).items() if k in expected}
        # Each marker's corners, board frame -> world -> camera (optical: x right, y down).
        world_board = _matrix(*sc.charuco_board_frame(obj))
        w, x, y, z = camera.quat_wxyz
        world_camera = _matrix(camera.pos, (x, y, z, w))
        camera_board = np.linalg.inv(world_camera) @ world_board
        assert sorted(found) == sorted(expected), obj['name']
        for marker, corners in expected.items():
            points = np.c_[corners, np.zeros(4), np.ones(4)] @ camera_board.T
            pixels = focal * points[:, :2] / points[:, 2:3] + [width / 2 - 0.5, height / 2 - 0.5]
            assert found[marker] == pytest.approx(pixels, abs=RENDER_TOLERANCE_PX), marker
