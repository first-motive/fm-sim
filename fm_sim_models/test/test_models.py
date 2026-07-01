"""Unit-test the MJCF registry lookup — pure, no ROS."""

import pytest

from fm_sim_models.models import materialize_task_env_model, mjcf_path


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


def test_task_env_materialization_copies_template_into_external_tree(tmp_path):
    template_dir = tmp_path / "assets" / "mujoco" / "so101"
    template_dir.mkdir(parents=True)
    (template_dir / "pick_place.xml").write_text(
        '<mujoco model="x"><include file="scene.xml" /></mujoco>',
        encoding="utf-8",
    )

    model_path = materialize_task_env_model("pick_place", workspace_root=str(tmp_path))

    assert model_path.endswith("/external/so_arm/Simulation/SO101/fm_task_env_pick_place.xml")
    assert "scene.xml" in (
        tmp_path / "external" / "so_arm" / "Simulation" / "SO101" / "fm_task_env_pick_place.xml"
    ).read_text(encoding="utf-8")
