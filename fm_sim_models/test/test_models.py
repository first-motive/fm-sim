"""Unit-test the MJCF registry lookup — pure, no ROS."""

import os

from fm_sim_models.models import mjcf_path
import pytest


def test_known_robots_resolve():
    for key in ('openarm_pinch', 'so101', 'g1_d', 'axol'):
        path = mjcf_path(key)
        assert path.endswith('.xml')


def test_axol_mjcf_is_installed():
    """Axol's MJCF ships in this package's share, so it must exist on disk.

    The vendored models, by contrast, live under the gitignored external mount.
    """
    path = mjcf_path('axol')
    assert os.path.isfile(path), f'axol MJCF missing from share: {path}'


def test_vendored_path_follows_fm_ws(monkeypatch):
    """Vendored MJCF paths resolve against the workspace root, not a hardcoded mount.

    Same registry has to serve the container (/ws) and a native checkout, so the
    root must track FM_WS at call time.
    """
    monkeypatch.setenv('FM_WS', '/somewhere/ws')
    path = mjcf_path('openarm_pinch')
    assert path == '/somewhere/ws/external/openarm_mujoco/v2/openarm_bimanual.xml'


def test_unknown_robot_raises_with_registered_keys():
    with pytest.raises(RuntimeError) as excinfo:
        mjcf_path('nope')
    message = str(excinfo.value)
    assert 'nope' in message
    assert 'openarm' in message  # error lists the registered keys


def test_openarm_is_the_loader_hand(monkeypatch, tmp_path):
    # The registry hands the vendored openarm_mujoco to the loader-hand patch.
    monkeypatch.setenv('FM_WS', str(tmp_path))
    with pytest.raises(RuntimeError, match='openarm_mujoco is not vendored'):
        mjcf_path('openarm')
