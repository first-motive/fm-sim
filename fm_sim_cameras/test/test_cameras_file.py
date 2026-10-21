"""The shared cameras-file loader and CameraInfo builder."""

import copy

from conftest import CHEST_D, CHEST_K, OPENARM_CAMERAS
from fm_sim_cameras import cameras_file
from fm_sim_cameras.camera_info import camera_info
import pytest
import yaml


def _raw():
    with open(OPENARM_CAMERAS) as openarm_file:
        return yaml.safe_load(openarm_file)


def _write(tmp_path, data):
    path = tmp_path / 'cameras.yaml'
    path.write_text(yaml.safe_dump(data))
    return str(path)


def test_openarm_cameras_carry_the_real_chest_intrinsics():
    cameras = cameras_file.load(OPENARM_CAMERAS)
    assert sorted(cameras) == ['chest', 'wrist_right']
    assert cameras['chest']['frame'] == 'cam_chest_optical'
    assert cameras['wrist_right']['frame'] == 'cam_wrist_r_optical'
    for camera in cameras.values():
        info = camera_info(camera['intrinsics'])
        assert list(info.k) == CHEST_K
        assert list(info.d) == CHEST_D
        assert info.distortion_model == 'equidistant'
        assert (info.width, info.height) == (1920, 1080)
        assert list(info.r) == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        assert list(info.p) == [*CHEST_K[:3], 0.0, *CHEST_K[3:6], 0.0, *CHEST_K[6:], 0.0]


def test_check_returns_plain_normalised_data():
    data = _raw()
    data['cameras']['chest']['rate_hz'] = 10
    (chest, _) = cameras_file.check(data).values()
    assert chest['rate_hz'] == 10.0 and isinstance(chest['rate_hz'], float)
    assert chest['allow_large_image'] is False
    assert isinstance(chest['intrinsics']['fx'], float)


def test_plumb_bob_is_a_shared_model():
    data = _raw()
    data['cameras']['chest']['intrinsics'].update(
        distortion_model='plumb_bob', distortion=[0.1, 0.0, 0.0, 0.0, 0.0]
    )
    assert cameras_file.check(data)['chest']['intrinsics']['distortion_model'] == 'plumb_bob'


def test_a_large_image_needs_allow_large_image():
    data = _raw()
    data['cameras']['chest']['intrinsics'] = dict(
        data['cameras']['chest']['intrinsics'], width=3840, height=2160
    )
    with pytest.raises(cameras_file.CameraError, match='allow_large_image'):
        cameras_file.check(copy.deepcopy(data))
    data['cameras']['chest']['allow_large_image'] = True
    assert cameras_file.check(data)['chest']['intrinsics']['width'] == 3840


@pytest.mark.parametrize('change, message', [
    (lambda d: d['cameras'].update(head=d['cameras']['chest']), "unknown camera role 'head'"),
    (lambda d: d['cameras'].update({'../x': d['cameras']['chest']}), 'unknown camera role'),
    (lambda d: d['cameras']['chest'].update(frame='../x'), 'not a frame name'),
    (lambda d: d['cameras']['chest'].update(rate_hz=0), 'rate_hz must be a positive'),
    (lambda d: d['cameras']['chest'].update(rate_hz=True), 'rate_hz must be a positive'),
    (lambda d: d['cameras']['chest'].update(rate_hz=float('inf')), 'rate_hz must be a positive'),
    (lambda d: d['cameras']['chest'].update(allow_large_image='yes'), 'true or false'),
    (lambda d: d['cameras']['chest'].pop('intrinsics'), 'intrinsics must be a mapping'),
    (lambda d: d['cameras']['chest']['intrinsics'].update(distortion_model='rational'),
     "unknown distortion_model 'rational'"),
    (lambda d: d['cameras']['chest']['intrinsics'].update(distortion=[0.1, 0.0, 0.0]),
     'equidistant takes 4'),
    (lambda d: d['cameras']['chest']['intrinsics'].update(distortion=[0.1, 0.0, 0.0, None]),
     'equidistant takes 4'),
    (lambda d: d['cameras']['chest']['intrinsics'].update(fx=-1.0), 'fx must be a positive'),
    (lambda d: d['cameras']['chest']['intrinsics'].update(cx=float('nan')), 'cx must be'),
    (lambda d: d['cameras']['chest']['intrinsics'].update(width=1920.5), 'positive integer'),
    (lambda d: d['cameras']['chest']['intrinsics'].update(height=True), 'positive integer'),
    (lambda d: d.pop('cameras'), "a 'cameras' mapping"),
])
def test_load_refuses_a_bad_cameras_file(tmp_path, change, message):
    data = copy.deepcopy(_raw())
    change(data)
    with pytest.raises(cameras_file.CameraError, match=message):
        cameras_file.load(_write(tmp_path, data))


def test_topics_follow_the_role_and_refuse_unknown_ones():
    assert cameras_file.image_topic('wrist_right') == '/cam_wrist_right/image_raw/compressed'
    assert cameras_file.info_topic('chest') == '/cam_chest/camera_info'
    with pytest.raises(cameras_file.CameraError):
        cameras_file.image_topic('wrist_left')
