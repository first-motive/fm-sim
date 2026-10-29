"""A RunSpec's params: shape, the check against the catalog, and the files a run gets."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest
import yaml

from fm_sim_agent import params
from fm_sim_agent.runs import PARAMS_JSON, PARAMS_YAML
from fm_sim_agent.spec import SpecError, parse_spec
from tests.fakes import CATALOG, SPEC
from tests.test_agent import ask, code, up_running
from tests.test_runner import RUN_ID, docker  # noqa: F401 - the fixture

CASES = json.loads((Path(__file__).parents[2] / "fm_sim_bringup" / "test" / "param_cases.json").read_text())[
    "cases"
]

SPEED = {
    "label": "Arm speed", "type": "float", "default": 0.5, "min": 0.1, "max": 2.0,
    "target": {"kind": "ros_param", "node": "/motion_server", "param": "max_joint_speed"},
}  # fmt: skip
RATE = {
    "label": "Object pose rate", "type": "float", "default": 30.0, "min": 1.0, "max": 100.0,
    "backends": ["mujoco"],
    "target": {
        "kind": "ros_param", "node": "/mujoco_ros2_control_node",
        "param": "mujoco_plugins.object_poses.publish_rate",
    },
}  # fmt: skip
MASS = {
    "label": "Can mass", "type": "float", "default": 0.35, "min": 0.05, "max": 2.0,
    "target": {"kind": "entry", "path": "objects.can.mass"},
}  # fmt: skip


def entries_with(**tunables) -> dict:
    """The four entries of SPEC, the controller declaring ``tunables``."""
    by_name = {
        d: next(e for e in CATALOG[f"{d}s"] if e["name"] == SPEC[d]) for d in ("backend", "robot", "scenario")
    }
    return {**copy.deepcopy(by_name), "controller": {"name": "idle", "params": tunables}}


@pytest.mark.parametrize("case", CASES, ids=[case["name"] for case in CASES])
def test_check_value_matches_the_shared_cases(case):
    if case["accepted"]:
        value = params.check_value(case["param"], case["value"])
        assert json.dumps(value) == json.dumps(case["normalised"])  # 1.0 stays 1.0
    else:
        with pytest.raises(SpecError, match=case["reason"]):
            params.check_value(case["param"], case["value"])


# -- shape -------------------------------------------------------------------


def test_a_spec_without_params_carries_none():
    assert "params" not in parse_spec(SPEC)
    assert "params" not in parse_spec({**SPEC, "params": {}})


def test_params_come_back_sorted():
    spec = parse_spec({**SPEC, "params": {"scenario.b": 1, "controller.a": [1, 2]}})
    assert list(spec["params"]) == ["controller.a", "scenario.b"]


@pytest.mark.parametrize(
    "value",
    [
        ["not", "an", "object"],
        {"max_joint_speed": 1.0},
        {"planner.max_joint_speed": 1.0},
        {"controller.Max": 1.0},
        {"controller.speed": None},
        {"controller.speed": {"nested": 1}},
        {"controller.speed": float("nan")},
        {"controller.speed": [1, True]},
        {"controller.speed": "x" * 65},
        {"controller.speed": list(range(65))},
        {f"controller.p{i}": 1 for i in range(257)},
    ],
)
def test_parse_spec_refuses_params_of_the_wrong_shape(value):
    with pytest.raises(SpecError):
        parse_spec({**SPEC, "params": value})


def test_a_params_refusal_never_echoes_the_value():
    with pytest.raises(SpecError) as refused:
        parse_spec({**SPEC, "params": {"controller.speed": {"\x1b[2Jevil": 1}}})
    assert "evil" not in str(refused.value)


# -- against the catalog -----------------------------------------------------


def test_check_normalises_against_the_declared_tunable():
    assert params.check({"controller.max_joint_speed": 1}, entries_with(max_joint_speed=SPEED)) == {
        "controller.max_joint_speed": 1.0
    }


@pytest.mark.parametrize(
    "overrides, match",
    [
        ({"controller.max_joint_speed": 1.5}, "not a tunable of controller idle"),
        ({"robot.speed": 1.0}, "not a tunable of robot openarm"),
        ({"controller.speed": True}, "params.controller.speed must be a number"),
        ({"controller.speed": 9.0}, "params.controller.speed must be between 0.1 and 2.0"),
    ],
    ids=["unknown", "on another dimension", "wrong type", "out of bounds"],
)
def test_check_refuses(overrides, match):
    with pytest.raises(SpecError, match=match):
        params.check(overrides, entries_with(speed=SPEED))


def test_check_refuses_a_tunable_off_its_backends():
    entries = entries_with(rate=RATE)
    assert params.check({"controller.rate": 10.0}, entries)
    entries["backend"] = {**entries["backend"], "name": "gazebo"}
    with pytest.raises(SpecError, match="applies only on mujoco"):
        params.check({"controller.rate": 10.0}, entries)


@pytest.mark.parametrize("node", ["/x\n/y", "/**", "/**/motion_server", "motion_server"])
def test_a_malformed_ros_param_declaration_is_not_a_tunable(node):
    # A wildcard key would sit in the tier of launch dicts, under every node-keyed guard.
    bad = {**SPEED, "target": {"kind": "ros_param", "node": node, "param": "p"}}
    with pytest.raises(SpecError, match="not a tunable"):
        params.check({"controller.speed": 1.0}, entries_with(speed=bad))


def test_ros_params_nests_by_node_then_dotted_param_and_renders_floats():
    entries = entries_with(speed=SPEED, rate=RATE, mass=MASS)
    checked = params.check({"controller.speed": 1, "controller.rate": 20, "controller.mass": 0.5}, entries)
    document = params.ros_params(checked, entries)
    assert document == {
        "/motion_server": {"ros__parameters": {"max_joint_speed": 1.0}},
        "/mujoco_ros2_control_node": {
            "ros__parameters": {"mujoco_plugins": {"object_poses": {"publish_rate": 20.0}}}
        },
    }
    assert "max_joint_speed: 1.0" in yaml.safe_dump(document)


def test_only_declared_targets_reach_the_params_file():
    entries = entries_with(speed=SPEED)
    with pytest.raises(SpecError):
        params.check({"controller.speed": 1.0, "controller.use_sim_time": False}, entries)
    document = params.ros_params(params.check({"controller.speed": 1.0}, entries), entries)
    assert document == {"/motion_server": {"ros__parameters": {"max_joint_speed": 1.0}}}


# -- through the agent and the runner ---------------------------------------


@pytest.fixture
def tunable_catalog(runner):
    doc = copy.deepcopy(CATALOG)
    doc["controllers"][0]["params"] = {"speed": SPEED, "mass": MASS}
    runner.catalog_doc = doc


def test_up_records_and_files_the_checked_params(agent, runner, store, tunable_catalog):
    run_id = up_running(agent, params={"controller.speed": 1, "controller.mass": 0.5})
    expected = {"controller.mass": 0.5, "controller.speed": 1.0}
    assert agent.status()["spec"]["params"] == expected
    assert store.read(run_id)["spec"]["params"] == expected
    directory = store.params_dir(run_id)
    assert json.loads((directory / PARAMS_JSON).read_text()) == expected
    assert yaml.safe_load((directory / PARAMS_YAML).read_text()) == {
        "/motion_server": {"ros__parameters": {"max_joint_speed": 1.0}}
    }


def test_up_without_params_files_none(agent, store, tunable_catalog):
    run_id = up_running(agent)
    assert "params" not in agent.status()["spec"]
    assert not store.params_dir(run_id).exists()


def test_up_with_only_entry_overrides_writes_no_ros_params_file(agent, store, tunable_catalog):
    run_id = up_running(agent, params={"controller.mass": 0.5})
    assert (store.params_dir(run_id) / PARAMS_JSON).is_file()
    assert not (store.params_dir(run_id) / PARAMS_YAML).exists()


@pytest.mark.parametrize("overrides", [{"controller.nope": 1.0}, {"controller.speed": 5.0}])
def test_up_refuses_bad_params_before_any_container(agent, runner, tunable_catalog, overrides):
    assert code(ask(agent, "up", {**SPEC, "params": overrides})) == "invalid_spec"
    assert "start" not in runner.names()
    assert agent.status()["state"] == "idle"


def test_run_argv_mounts_the_params_read_only_and_names_them(docker):  # noqa: F811
    directory = docker.runs_dir / RUN_ID / "params"
    assert "overrides_file:=/fm-run/params/params.json" not in docker.run_argv(RUN_ID, SPEC)
    directory.mkdir(parents=True)
    (directory / PARAMS_JSON).write_text("{}")
    argv = docker.run_argv(RUN_ID, SPEC)
    assert f"{directory}:/fm-run/params:ro" in argv
    assert argv[-1] == "overrides_file:=/fm-run/params/params.json"
    (directory / PARAMS_YAML).write_text("{}")
    assert docker.run_argv(RUN_ID, SPEC)[-2:] == [
        "overrides_file:=/fm-run/params/params.json",
        "params_file:=/fm-run/params/params.yaml",
    ]


@pytest.mark.parametrize("literal", ["NaN", "Infinity", "-Infinity", "[0, NaN]"])
def test_up_refuses_a_non_finite_value_json_lets_through(agent, runner, tunable_catalog, literal):
    # json.loads accepts these literals; a NaN bound check passes in some forms.
    body = json.dumps(SPEC)[:-1] + f', "params": {{"controller.speed": {literal}}}}}'
    assert code(agent.answer("up", body.encode())) == "invalid_spec"
    assert "start" not in runner.names()


@pytest.mark.parametrize("value", [float("nan"), float("inf"), [0.5, float("nan"), 0.5]])
def test_check_value_refuses_non_finite_numbers(value):
    array = {"type": "array", "items": "float", "length": 3, "min": -1.0, "max": 1.0}
    with pytest.raises(SpecError):
        params.check_value(array if isinstance(value, list) else SPEED, value)


def test_up_refuses_an_int_past_the_float_range_as_a_spec(agent, runner, tunable_catalog):
    body = json.dumps(SPEC)[:-1] + ', "params": {"controller.speed": 1' + "0" * 400 + "}}"
    assert code(agent.answer("up", body.encode())) == "invalid_spec"
    assert "start" not in runner.names()


@pytest.mark.parametrize("second", ["max_joint_speed", "max_joint_speed.arm"])
def test_two_entries_setting_one_ros_param_are_refused(second):
    entries = entries_with(speed=SPEED)
    entries["robot"] = {
        **entries["robot"],
        "params": {"speed": {**SPEED, "target": {**SPEED["target"], "param": second}}},
    }
    checked = params.check({"controller.speed": 1.0, "robot.speed": 1.0}, entries)
    with pytest.raises(SpecError, match="set the same ROS param"):
        params.ros_params(checked, entries)
