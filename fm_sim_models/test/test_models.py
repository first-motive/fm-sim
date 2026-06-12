"""Unit-test the MJCF registry lookup — pure, no ROS."""

import pytest

from fm_sim_models.models import mjcf_path


def test_known_robots_resolve():
    for key in ("openarm", "so101", "g1_d"):
        path = mjcf_path(key)
        assert path.endswith(".xml")


def test_unknown_robot_raises_with_registered_keys():
    with pytest.raises(RuntimeError) as excinfo:
        mjcf_path("nope")
    message = str(excinfo.value)
    assert "nope" in message
    assert "openarm" in message  # error lists the registered keys
