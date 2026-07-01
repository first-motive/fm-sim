"""Unit-test the MJCF registry lookup — pure, no ROS."""

import os

import pytest

from fm_sim_models.models import mjcf_path


def test_known_robots_resolve():
    for key in ("openarm", "so101", "g1_d", "axol"):
        path = mjcf_path(key)
        assert path.endswith(".xml")


def test_axol_mjcf_is_installed():
    """Axol's MJCF is committed into this package's share, so it must exist on disk
    (unlike the vendored models, which live under the gitignored external mount)."""
    path = mjcf_path("axol")
    assert os.path.isfile(path), f"axol MJCF missing from share: {path}"


def test_unknown_robot_raises_with_registered_keys():
    with pytest.raises(RuntimeError) as excinfo:
        mjcf_path("nope")
    message = str(excinfo.value)
    assert "nope" in message
    assert "openarm" in message  # error lists the registered keys
