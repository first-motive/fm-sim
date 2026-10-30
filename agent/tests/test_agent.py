"""Every verb's reply shape and error codes, driven through ``SimAgent.answer``."""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time

import pytest

from fm_sim_agent import agent as agent_module
from fm_sim_agent.agent import RESET_TIMEOUT_S, combination_refusal
from fm_sim_agent.runs import SUMMARY_KEYS
from fm_sim_agent.spec import valid_run_id
from tests.fakes import SPEC, URDF, entry, wait_for

STATUS_KEYS = {"host", "state", "run_id", "spec", "since", "version"}


def ask(agent, verb, body=None):
    return agent.answer(verb, None if body is None else json.dumps(body).encode())


def code(reply) -> str:
    assert reply["ok"] is False
    assert set(reply["error"]) == {"code", "message"}
    return reply["error"]["code"]


def up_running(agent, **overrides):
    reply = ask(agent, "up", {**SPEC, **overrides})
    assert reply["ok"] is True, reply
    wait_for(lambda: agent.status()["state"] == "running")
    return reply["run_id"]


def summary_of(run_id: str, stamp: str) -> dict:
    return {
        "run_id": run_id, "host": "fm_ws_01", "spec": SPEC, "started_at": stamp, "ended_at": stamp,
        "outcome": None, "mcap": None, "metrics": {},
    }  # fmt: skip


# -- status and catalog -------------------------------------------------------


def test_idle_status_has_the_contract_shape(agent):
    reply = ask(agent, "status")
    assert reply["ok"] is True
    assert set(reply) - {"ok"} == STATUS_KEYS
    assert reply["host"] == "fm_ws_01"
    assert reply["state"] == "idle"
    assert reply["run_id"] is None and reply["spec"] is None
    assert reply["since"].endswith("Z")


def test_catalog_is_returned_under_catalog(agent, runner):
    assert ask(agent, "catalog") == {"ok": True, "catalog": runner.catalog_doc}


def test_catalog_carries_controller_family_and_control_descriptor(agent):
    controllers = {c["name"]: c for c in ask(agent, "catalog")["catalog"]["controllers"]}
    picker = controllers["picker_classical"]
    assert picker["family"] == "picker"
    assert picker["control_descriptor"]["blocks"][0]["variant"] == "bg-diff"


def test_catalog_is_cached_per_image_id(agent, runner):
    ask(agent, "catalog")
    ask(agent, "catalog")
    assert runner.catalog_fetches == 1
    runner.image = "sha256:two"
    ask(agent, "catalog")
    assert runner.catalog_fetches == 2


def test_concurrent_catalog_queries_share_one_fetch(agent, runner):
    gate = threading.Event()
    real = runner.catalog

    def slow():
        gate.wait(5)
        return real()

    runner.catalog = slow
    replies = []
    threads = [threading.Thread(target=lambda: replies.append(ask(agent, "catalog"))) for _ in range(4)]
    for thread in threads:
        thread.start()
    gate.set()
    for thread in threads:
        thread.join(5)
    assert [r["ok"] for r in replies] == [True] * 4
    assert runner.catalog_fetches == 1


@pytest.mark.parametrize(
    "doc",
    [
        {"schema_version": 2, "backends": [], "robots": [], "scenarios": [], "controllers": []},
        {"schema_version": 1, "backends": [], "robots": [], "scenarios": []},
        {"schema_version": 1, "backends": [{"name": "x"}], "robots": [], "scenarios": [], "controllers": []},
        ["not", "a", "catalog"],
    ],
)
def test_a_malformed_catalog_is_refused(agent, runner, doc):
    runner.catalog_doc = doc
    assert code(ask(agent, "catalog")) == "catalog_unavailable"


def test_a_catalog_failure_is_generic_and_cached_briefly(agent, runner, capsys):
    runner.fail["catalog"] = "no fm_sim_bringup in /opt/secret/path"
    reply = ask(agent, "catalog")
    assert code(reply) == "catalog_unavailable"
    assert "secret" not in reply["error"]["message"]
    assert "secret" in capsys.readouterr().err
    del runner.fail["catalog"]
    assert code(ask(agent, "catalog")) == "catalog_unavailable"
    assert runner.catalog_fetches == 1


def test_the_failure_cache_expires(agent, runner, monkeypatch):
    runner.fail["catalog"] = "broken"
    ask(agent, "catalog")
    del runner.fail["catalog"]
    monkeypatch.setattr(agent_module, "CATALOG_FAILURE_TTL_S", 0.0)
    assert ask(agent, "catalog")["ok"] is True


# -- up and readiness ---------------------------------------------------------


def test_up_starts_and_reaches_running_once_broadcasters_are_active(agent, runner, store):
    run_id = up_running(agent)
    assert valid_run_id(run_id)
    assert run_id.endswith("-mujoco-openarm-empty-idle-7")
    assert ("start", run_id, SPEC) in runner.calls
    status = agent.status()
    assert status["run_id"] == run_id and status["spec"] == SPEC
    assert store.read(run_id)["ended_at"] is None


def test_a_run_stays_starting_until_controller_manager_answers(agent, runner):
    runner.ready.clear()
    ask(agent, "up", SPEC)
    assert agent.status()["state"] == "starting"
    runner.ready.set()
    wait_for(lambda: agent.status()["state"] == "running")


def test_an_inactive_broadcaster_holds_the_run_in_starting(agent, runner):
    runner.controllers["joint_state_broadcaster"] = "inactive"
    ask(agent, "up", SPEC)
    wait_for(lambda: runner.names().count("list_controllers") >= 2)
    assert agent.status()["state"] == "starting"
    runner.controllers["joint_state_broadcaster"] = "active"
    wait_for(lambda: agent.status()["state"] == "running")


def test_a_run_whose_broadcasters_never_activate_is_stopped(agent, runner, store):
    runner.controllers = {}
    agent.startup_timeout_s = 0.05
    run_id = ask(agent, "up", SPEC)["run_id"]
    wait_for(lambda: agent.status()["state"] == "error")
    assert agent.status()["error"] == "not ready in time"
    assert ("stop", run_id) in runner.calls
    assert store.read(run_id)["ended_at"] is not None


def test_the_seed_is_applied_through_reset_once_ready(agent, runner, store):
    run_id = up_running(agent)
    assert ("reset", run_id, 7, RESET_TIMEOUT_S) in runner.calls
    names = runner.names()
    assert names.index("has_service") < names.index("reset")
    assert store.read(run_id)["metrics"]["seed_applied"] is True


def test_a_graph_without_reset_records_the_seed_as_not_applied(agent, runner):
    runner.serves_reset = False
    up_running(agent)
    assert "reset" not in runner.names()
    assert ask(agent, "down")["summary"]["metrics"]["seed_applied"] is False


def test_seed_zero_without_reset_counts_as_applied(agent, runner):
    runner.serves_reset = False
    up_running(agent, seed=0)
    assert ask(agent, "down")["summary"]["metrics"]["seed_applied"] is True


def test_a_refused_startup_reset_fails_the_run(agent, runner):
    runner.fail["reset"] = "scenario rejected the seed"
    run_id = ask(agent, "up", SPEC)["run_id"]
    wait_for(lambda: agent.status()["state"] == "error")
    assert agent.status()["error"] == "start failed"
    assert ("stop", run_id) in runner.calls


def test_up_is_busy_while_another_process_holds_a_run_under_the_label(agent, runner):
    """The managed agent and an in-process ``fm sim up --host local`` share one label."""
    runner.live = ["20261016T120000-mujoco-openarm-empty-idle-3"]
    reply = ask(agent, "up", SPEC)
    assert code(reply) == "busy"
    assert "20261016T120000-mujoco-openarm-empty-idle-3" in reply["error"]["message"]
    assert "start" not in runner.names()
    runner.live = []
    assert ask(agent, "up", SPEC)["ok"] is True


def test_a_second_up_is_busy(agent, runner):
    run_id = up_running(agent)
    reply = ask(agent, "up", {**SPEC, "seed": 8})
    assert code(reply) == "busy"
    assert run_id in reply["error"]["message"]
    assert runner.names().count("start") == 1


@pytest.mark.parametrize(
    "spec",
    [
        {**SPEC, "backend": "MuJoCo"},
        {**SPEC, "robot": "../etc"},
        {**SPEC, "robot": "openarm\n"},
        {**SPEC, "scenario": "empty; rm -rf /"},
        {**SPEC, "controller": ""},
        {**SPEC, "controller": "a" * 65},
        {**SPEC, "seed": -1},
        {**SPEC, "seed": 2**32},
        {**SPEC, "seed": "7"},
        {**SPEC, "seed": True},
        {**SPEC, "seed": 7.0},
        {**SPEC, "record": "yes"},
        {**SPEC, "record": 1},
        {**SPEC, "extra": 1},
        # A params key that is not <dimension>.<name>.
        {**SPEC, "params": {"world_file": "tabletop.sdf"}},
        {k: v for k, v in SPEC.items() if k != "seed"},
        {k: v for k, v in SPEC.items() if k != "record"},
        {k: v for k, v in SPEC.items() if k != "backend"},
    ],
)
def test_up_refuses_an_invalid_spec_before_any_container(agent, runner, spec):
    assert code(ask(agent, "up", spec)) == "invalid_spec"
    assert runner.calls == []
    assert agent.status()["state"] == "idle"


def test_a_refusal_never_echoes_the_callers_keys(agent):
    reply = ask(agent, "up", {**SPEC, "\x1b[2Jevil": 1})
    assert "evil" not in reply["error"]["message"]


def test_up_refuses_a_name_the_catalog_lacks(agent, runner):
    reply = ask(agent, "up", {**SPEC, "scenario": "kitchen"})
    assert code(reply) == "unknown_entry"
    assert "empty, pick_cube" in reply["error"]["message"]
    assert runner.names() == []


def test_up_refuses_an_unavailable_entry(agent, runner):
    assert code(ask(agent, "up", {**SPEC, "backend": "isaac"})) == "unavailable_entry"
    assert runner.names() == []


def _pair_catalog(runner):
    """The fake catalog with the pair rules run.launch.py checks, on a copy."""
    doc = json.loads(json.dumps(runner.catalog_doc))
    doc["robots"][0]["supported_backends"] = ["mujoco"]
    doc["robots"][0]["launch_roles"] = {"servo": {"requires": ["right_arm"]}}
    launch = {"package": "fm_skills_sim", "path": "launch/sim.launch.py"}
    doc["controllers"] += [
        {**entry("fm_skills"), "activate": [], "launch": launch},
        {**entry("policy_serve"), "activate": ["right_arm", "servo"], "launch": launch},
    ]
    runner.catalog_doc = doc


@pytest.mark.parametrize(
    ("change", "reason"),
    [
        ({"backend": "gazebo"}, "robot openarm is not supported on backend gazebo; supported: mujoco"),
        ({"robot": "so101", "controller": "fm_skills"}, "controller fm_skills needs a robot profile"),
        ({"robot": "so101", "controller": "policy_serve"}, "controller policy_serve needs a robot profile"),
    ],
)
def test_up_refuses_a_combination_the_launch_refuses(agent, runner, change, reason):
    _pair_catalog(runner)
    reply = ask(agent, "up", {**SPEC, **change})
    assert code(reply) == "unavailable_entry"
    assert reason in reply["error"]["message"]
    assert runner.names() == []  # refused before any container starts


def test_up_refuses_a_controller_activating_a_role_the_robot_lacks(agent, runner):
    _pair_catalog(runner)
    runner.catalog_doc["robots"][1]["robot_profile"] = "fm_so101"
    reply = ask(agent, "up", {**SPEC, "robot": "so101", "controller": "policy_serve"})
    assert code(reply) == "unavailable_entry"
    assert "activates right_arm, servo, which robot so101 does not have" in reply["error"]["message"]


#: The launch's own accept/refuse table; test_run.py checks run.py against it.
#: A path, not a package resource: the table lives beside the launch's tests.
COMBINATION_CASES = json.loads(
    (Path(__file__).parents[2] / "fm_sim_bringup" / "test" / "combination_cases.json").read_text()
)["cases"]


@pytest.mark.parametrize("case", COMBINATION_CASES, ids=lambda case: case["name"])
def test_combination_refusal_matches_the_launch(case):
    entries = {
        "backend": entry(case["backend"]),
        "robot": {**entry("robot"), **case["robot"]},
        "scenario": entry(case.get("scenario", "empty")),
        "controller": {**entry("controller"), **case["controller"]},
    }
    refusal = combination_refusal(entries)
    assert (refusal is not None) == case["refused"]
    assert case.get("reason", "") in (refusal or "")


def test_combination_refusal_refuses_a_robot_listing_no_backends():
    # run.py reads supported_backends with no default, so a robot without it fails at launch.
    entries = {"backend": entry("mujoco"), "robot": entry("robot"), "controller": entry("idle")}
    assert combination_refusal(entries) == "robot robot is not supported on backend mujoco; supported: none"


def test_up_refuses_a_robot_with_no_broadcasters_listed(agent, runner):
    assert code(ask(agent, "up", {**SPEC, "robot": "bare"})) == "catalog_unavailable"
    assert runner.names() == []


def test_up_without_a_catalog_is_refused(agent, runner):
    runner.fail["image_id"] = "no such image"
    assert code(ask(agent, "up", SPEC)) == "catalog_unavailable"


def test_a_failed_start_still_writes_its_summary(agent, runner, store, capsys):
    runner.fail["start"] = "port 8765 in use by pid 4242"
    reply = ask(agent, "up", SPEC)
    assert code(reply) == "start_failed"
    assert "4242" not in reply["error"]["message"] and "4242" not in str(agent.status())
    assert "4242" in capsys.readouterr().err
    (summary,) = store.recent(10)
    assert summary["ended_at"] is not None
    assert agent.status()["state"] == "error"
    del runner.fail["start"]
    assert ask(agent, "up", {**SPEC, "seed": 8})["ok"] is True


def test_an_unexpected_failure_after_the_claim_hands_the_run_back(agent, store, monkeypatch, capsys):
    """A bug past the busy check must not leave the host busy with no run."""
    real_write = store.write
    calls = []

    def flaky(summary):
        calls.append(summary["run_id"])
        if len(calls) == 1:
            raise OSError("disk full")
        return real_write(summary)

    monkeypatch.setattr(store, "write", flaky)
    reply = ask(agent, "up", SPEC)
    assert code(reply) == "internal_error"
    assert "disk" not in reply["error"]["message"]
    assert "disk full" in capsys.readouterr().err
    status = agent.status()
    assert status["run_id"] is None and status["state"] == "error"
    assert ask(agent, "up", {**SPEC, "seed": 9})["ok"] is True


def test_a_container_that_exits_finalises_the_run(agent, runner, store):
    run_id = up_running(agent)
    runner.exit()
    wait_for(lambda: agent.status()["state"] == "error")
    status = agent.status()
    assert status["run_id"] is None and "exited" in status["error"]
    assert store.read(run_id)["ended_at"] is not None
    assert code(ask(agent, "down")) == "not_running"


# -- down -----------------------------------------------------------------------


def test_down_returns_and_writes_the_summary(agent, runner, store):
    run_id = up_running(agent)
    reply = ask(agent, "down", {"run_id": run_id})
    assert reply["ok"] is True
    summary = reply["summary"]
    assert set(summary) == SUMMARY_KEYS
    assert summary["run_id"] == run_id and summary["host"] == "fm_ws_01" and summary["spec"] == SPEC
    assert summary["ended_at"] is not None
    assert summary["outcome"] == {"success": True, "reason": "placed", "elapsed_s": 4.5}
    assert summary["mcap"] is None
    assert isinstance(summary["metrics"]["wall_s"], float)
    assert store.read(run_id) == summary
    assert agent.status()["state"] == "idle"
    assert runner.names().index("outcome") < runner.names().index("stop")


def test_down_without_a_run_id_stops_the_active_run(agent):
    run_id = up_running(agent)
    assert ask(agent, "down")["summary"]["run_id"] == run_id


def test_a_recorded_run_names_its_mcap_relative_to_the_runs_dir(agent, runner, store):
    run_id = up_running(agent, record=True)
    assert ("start_recording", run_id) in runner.calls
    summary = ask(agent, "down")["summary"]
    assert summary["mcap"] == f"{run_id}/bag/recording/recording_0.mcap"
    assert (store.root / summary["mcap"]).is_file()
    assert summary["metrics"]["mcap_files"] == 2
    assert summary["metrics"]["mcap_bytes"] == 8 + 10
    names = runner.names()
    assert names.index("stop_recording") < names.index("stop")


def test_an_unrecorded_run_never_starts_the_recorder(agent, runner):
    up_running(agent)
    ask(agent, "down")
    assert "start_recording" not in runner.names() and "stop_recording" not in runner.names()


def test_a_run_with_no_outcome_reports_null(agent, runner):
    runner.outcome_value = None
    up_running(agent)
    assert ask(agent, "down")["summary"]["outcome"] is None


def test_an_outcome_from_before_a_reset_is_not_reported(agent, runner):
    up_running(agent)
    assert ask(agent, "reset", {"seed": 42}) == {"ok": True}
    # Stopped before seed 42's episode ended: the latch still holds seed 7's verdict.
    assert ask(agent, "down")["summary"]["outcome"] is None


def test_an_outcome_under_the_reset_seed_is_reported(agent, runner):
    up_running(agent)
    assert ask(agent, "reset", {"seed": 42}) == {"ok": True}
    runner.outcome_value = {**runner.outcome_value, "seed": 42}
    assert ask(agent, "down")["summary"]["outcome"] == {
        "success": True, "reason": "placed", "elapsed_s": 4.5,
    }  # fmt: skip


def test_a_refused_reset_keeps_the_spec_seed_for_the_outcome(agent, runner):
    up_running(agent)
    runner.fail["reset"] = "engine said no"
    assert code(ask(agent, "reset", {"seed": 42})) == "reset_failed"
    assert ask(agent, "down")["summary"]["outcome"]["success"] is True


def test_an_outcome_for_another_scenario_is_not_reported(agent, runner):
    runner.outcome_value = {**runner.outcome_value, "scenario": "pick_cube"}
    up_running(agent)
    assert ask(agent, "down")["summary"]["outcome"] is None


def test_down_with_no_run_is_not_running(agent):
    assert code(ask(agent, "down")) == "not_running"


def test_down_naming_another_run_is_refused(agent):
    up_running(agent)
    assert (
        code(ask(agent, "down", {"run_id": "20260101T000000-mujoco-openarm-empty-idle-1"})) == "run_mismatch"
    )
    assert agent.status()["state"] == "running"


@pytest.mark.parametrize("run_id", ["../../etc", 7, "x", "20260101T000000-mujoco-openarm-empty-idle-1\n"])
def test_down_refuses_what_is_not_a_run_id(agent, run_id):
    up_running(agent)
    assert code(ask(agent, "down", {"run_id": run_id})) == "invalid_request"


def test_down_while_starting_stops_the_run(agent, runner, store):
    runner.ready.clear()
    run_id = ask(agent, "up", SPEC)["run_id"]
    assert ask(agent, "down")["ok"] is True
    assert agent.status()["state"] == "idle"
    assert [c for c in runner.calls if c[0] == "stop"] == [("stop", run_id)]  # docker run had returned
    assert store.read(run_id)["ended_at"] is not None


# -- reset ----------------------------------------------------------------------


def test_reset_reseeds_a_running_run(agent, runner):
    run_id = up_running(agent)
    assert ask(agent, "reset", {"seed": 42}) == {"ok": True}
    assert ("reset", run_id, 42, RESET_TIMEOUT_S) in runner.calls


def test_reset_with_no_run_is_not_running(agent):
    assert code(ask(agent, "reset", {"seed": 1})) == "not_running"


def test_reset_on_a_graph_without_the_service_is_refused(agent, runner):
    runner.serves_reset = False
    up_running(agent)
    assert code(ask(agent, "reset", {"seed": 1})) == "reset_unavailable"


@pytest.mark.parametrize("body", [{}, {"seed": -3}, {"seed": "1"}, {"seed": False}])
def test_reset_refuses_a_bad_seed(agent, body):
    up_running(agent)
    assert code(ask(agent, "reset", body)) == "invalid_request"


def test_a_refused_reset_is_reset_failed(agent, runner):
    up_running(agent)
    runner.fail["reset"] = "scenario has no seed"
    assert code(ask(agent, "reset", {"seed": 1})) == "reset_failed"


# -- runs -----------------------------------------------------------------------


def test_runs_lists_summaries_newest_first(agent, store):
    for day in (1, 3, 2):
        store.write(
            summary_of(f"2026100{day}T000000-mujoco-openarm-empty-idle-{day}", f"2026-10-0{day}T00:00:00Z")
        )
    reply = ask(agent, "runs")
    assert reply["ok"] is True
    assert [r["run_id"][-1] for r in reply["runs"]] == ["3", "2", "1"]
    assert len(ask(agent, "runs", {"limit": 2})["runs"]) == 2


def test_runs_reads_only_as_many_summaries_as_it_returns(agent, store, monkeypatch):
    for day in range(1, 6):
        store.write(summary_of(f"2026100{day}T000000-mujoco-openarm-empty-idle-{day}", "x"))
    read = []
    real = store.read
    monkeypatch.setattr(store, "read", lambda run_id: read.append(run_id) or real(run_id))
    ask(agent, "runs", {"limit": 2})
    assert len(read) == 2


@pytest.mark.parametrize(
    "content",
    [
        "{not json",
        json.dumps({"run_id": "20261002T000000-mujoco-openarm-empty-idle-2"}),
        "[]",
        " " * (1 << 17),
    ],
)
def test_runs_skips_what_is_not_a_summary(agent, store, content):
    store.write(summary_of("20261001T000000-mujoco-openarm-empty-idle-1", "2026-10-01T00:00:00Z"))
    broken = store.root / "20261002T000000-mujoco-openarm-empty-idle-2"
    broken.mkdir()
    (broken / "summary.json").write_text(content)
    assert [r["run_id"][-1] for r in ask(agent, "runs")["runs"]] == ["1"]


def test_a_summary_filed_under_another_id_is_skipped(agent, store):
    summary = summary_of("20261001T000000-mujoco-openarm-empty-idle-1", "x")
    other = store.root / "20261002T000000-mujoco-openarm-empty-idle-2"
    other.mkdir(parents=True)
    (other / "summary.json").write_text(json.dumps(summary))
    assert ask(agent, "runs")["runs"] == []


@pytest.mark.parametrize("limit", [0, -1, "5", True, 10_001])
def test_runs_refuses_a_bad_limit(agent, limit):
    assert code(ask(agent, "runs", {"limit": limit})) == "invalid_request"


def test_a_finished_run_appears_in_runs(agent):
    run_id = up_running(agent)
    ask(agent, "down")
    assert ask(agent, "runs")["runs"][0]["run_id"] == run_id


# -- logs -----------------------------------------------------------------------


def test_log_lines_reach_the_sink_parsed(agent, runner, logs):
    runner.log_lines = ["[sim_loop-1] [WARN] [1760000000.5] [sim_loop]: slow step\n"]
    run_id = up_running(agent)
    wait_for(lambda: logs)
    assert logs[0] == (run_id, {"t": 1760000000.5, "level": "warn", "node": "sim_loop", "msg": "slow step"})


def test_a_flood_of_log_lines_is_capped_and_counted(agent, runner, logs, monkeypatch):
    monkeypatch.setattr(agent_module, "LOG_MAX_PER_S", 5)
    runner.log_lines = [f"line {n}\n" for n in range(50)]
    up_running(agent)
    runner.exit()
    wait_for(lambda: agent.status()["state"] == "error")
    messages = [record["msg"] for _, record in logs]
    assert messages[:5] == [f"line {n}" for n in range(5)]
    assert messages[-1] == "45 log lines dropped: more than 5 a second"
    assert len(messages) == 6


def test_a_failing_sink_does_not_end_the_pump(agent, runner):
    published = []

    def sink(run_id, record):
        if not published:
            published.append("failed")
            raise RuntimeError("session closed")
        published.append(record["msg"])

    agent.log_sink = sink
    runner.log_lines = ["one\n", "two\n"]
    up_running(agent)
    wait_for(lambda: "two" in published)


# -- concurrency, recovery, malformed input -----------------------------------


def test_status_answers_while_up_waits_on_docker(agent, runner):
    """The lock guards state only: a slow container start must not stall status."""
    runner.start_gate = threading.Event()
    replies = []
    upper = threading.Thread(target=lambda: replies.append(ask(agent, "up", SPEC)))
    upper.start()
    wait_for(lambda: runner.names() == ["start"])
    assert agent.status()["state"] == "starting"
    assert code(ask(agent, "up", SPEC)) == "busy"
    runner.start_gate.set()
    upper.join(5)
    assert replies[0]["ok"] is True


def test_recover_clears_stale_runs_and_closes_open_summaries(agent, runner, store):
    open_id = "20261001T000000-mujoco-openarm-empty-idle-1"
    store.write({**summary_of(open_id, "2026-10-01T00:00:00Z"), "ended_at": None})
    assert agent.recover() == ["fm-sim-run-old"]
    assert store.read(open_id)["ended_at"] is not None


def test_recover_under_a_shared_label_leaves_a_run_another_process_holds(agent, runner, store):
    """A second agent beside the managed one, or one beside an in-process run, removes nothing."""
    held = "20261001T000000-mujoco-openarm-empty-idle-1"
    stale = "20261001T000000-mujoco-openarm-empty-idle-2"
    for run_id in (held, stale):
        store.write({**summary_of(run_id, "2026-10-01T00:00:00Z"), "ended_at": None})
    agent.keep_foreign_runs = True
    runner.live = [held]
    assert agent.recover() == []
    assert "stop_stale" not in runner.names()
    assert store.read(held)["ended_at"] is None  # its holder closes it
    assert store.read(stale)["ended_at"] is not None


def test_recover_under_a_shared_label_removes_nothing_when_it_cannot_list(agent, runner):
    agent.keep_foreign_runs = True
    runner.fail["active_runs"] = "docker ps failed"
    assert agent.recover() == []
    assert "stop_stale" not in runner.names()


def test_recover_under_a_shared_label_with_no_live_run_clears_as_before(agent, runner):
    agent.keep_foreign_runs = True
    assert agent.recover() == ["fm-sim-run-old"]


def test_up_goes_ahead_when_the_label_cannot_be_listed(agent, runner):
    runner.fail["active_runs"] = "docker ps failed"
    assert ask(agent, "up", SPEC)["ok"] is True


def test_the_fakes_log_streams_are_per_run(runner):
    """A stopped run's end of stream must never end the next run's: it did, under load."""
    first, second = (
        "20261001T000000-mujoco-openarm-empty-idle-1",
        "20261001T000000-mujoco-openarm-empty-idle-2",
    )
    runner.start(first, SPEC)
    runner.start(second, SPEC)
    runner.stop(first)
    assert list(runner.logs(first)) == []  # ended
    assert runner._stream(second).empty()  # untouched
    runner.exit()  # by default the last run started
    assert list(runner.logs(second)) == []


@pytest.mark.parametrize("payload", [b"{not json", b"[1, 2]", b"\xff\xfe", b"x" * (1 << 17)])
def test_a_body_that_is_not_an_object_is_refused(agent, payload):
    assert code(agent.answer("up", payload)) == "invalid_request"


def test_an_unknown_verb_is_refused(agent):
    assert code(ask(agent, "teleport")) == "unknown_verb"


def test_a_bug_is_an_internal_error_not_an_exception(agent, store, monkeypatch, capsys):
    monkeypatch.setattr(store, "recent", lambda limit: 1 / 0)
    reply = ask(agent, "runs")
    assert code(reply) == "internal_error"
    assert "ZeroDivisionError" in capsys.readouterr().err


# -- round-2 lifecycle and limits ----------------------------------------------


def test_a_failed_log_stream_finalises_the_run(agent, runner, store):
    """Without its log stream the agent cannot see the container exit."""
    runner.fail["logs"] = "docker logs died"
    run_id = ask(agent, "up", SPEC)["run_id"]
    wait_for(lambda: agent.status()["state"] == "error")
    assert agent.status()["error"] == "the run's log stream failed"
    assert ("stop", run_id) in runner.calls
    assert store.read(run_id)["ended_at"] is not None


def test_a_bug_in_a_run_thread_finalises_the_run(agent, runner, store, monkeypatch, capsys):
    def broken(run_id):
        raise KeyError("bug")

    monkeypatch.setattr(runner, "list_controllers", broken)
    run_id = ask(agent, "up", SPEC)["run_id"]
    wait_for(lambda: agent.status()["state"] == "error")
    assert agent.status()["error"] == "internal error"
    assert ("stop", run_id) in runner.calls
    assert "KeyError" in capsys.readouterr().err


def test_a_failed_docker_run_is_stopped_too(agent, runner):
    """A timed-out `docker run` may still have created the container."""
    runner.fail["start"] = "docker run timed out after 90s"
    assert code(ask(agent, "up", SPEC)) == "start_failed"
    assert "stop" in runner.names()


def test_a_run_past_its_maximum_duration_is_brought_down(agent, runner, store):
    agent.max_run_s = 0.2
    run_id = up_running(agent)
    wait_for(lambda: agent.status()["state"] == "idle")
    summary = store.read(run_id)
    assert summary["ended_at"] is not None and summary["metrics"]["max_duration_reached"] is True
    assert ("stop", run_id) in runner.calls


def test_a_run_that_ends_first_is_not_expired(agent, runner, store):
    agent.max_run_s = 0.3
    run_id = up_running(agent)
    ask(agent, "down")
    time.sleep(0.5)
    assert "max_duration_reached" not in store.read(run_id)["metrics"]


def test_up_is_refused_when_the_disk_is_low(agent, runner):
    agent.min_free_bytes = 2**62
    assert code(ask(agent, "up", SPEC)) == "insufficient_space"
    assert "start" not in runner.names()


def test_an_empty_broadcaster_list_is_refused(agent, runner):
    runner.catalog_doc = json.loads(json.dumps(runner.catalog_doc))
    runner.catalog_doc["robots"][0]["controller_roles"]["broadcasters"] = []
    assert code(ask(agent, "up", SPEC)) == "catalog_unavailable"


def test_deeply_nested_json_is_invalid_request_not_a_crash(agent, capsys):
    reply = agent.answer("up", b"[" * 30_000 + b"]" * 30_000)
    assert code(reply) == "invalid_request"
    assert "Traceback" not in capsys.readouterr().err


def test_store_refuses_a_symlinked_summary_and_run_dir(agent, store, tmp_path):
    run_id = "20261001T000000-mujoco-openarm-empty-idle-1"
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    (elsewhere / "summary.json").write_text(json.dumps(summary_of(run_id, "x")))
    store.root.mkdir(parents=True)
    os.symlink(elsewhere, store.root / run_id)
    assert store.read(run_id) is None
    with pytest.raises(OSError):
        store.write(summary_of(run_id, "x"))
    real = store.root / "20261002T000000-mujoco-openarm-empty-idle-2"
    real.mkdir()
    os.symlink(elsewhere / "summary.json", real / "summary.json")
    assert store.read(real.name) is None


def test_recordings_never_follow_a_symlink(store, tmp_path):
    run_id = "20261001T000000-mujoco-openarm-empty-idle-1"
    recording = store.recording_dir(run_id)
    recording.mkdir(parents=True)
    secret = tmp_path / "secret.mcap"
    secret.write_bytes(b"x" * 100)
    os.symlink(secret, recording / "recording_0.mcap")
    (recording / "recording_1.mcap").write_bytes(b"ok")
    assert store.recordings(run_id) == [(f"{run_id}/bag/recording/recording_1.mcap", 2)]


def test_concurrent_writes_of_one_run_never_collide(store):
    summary = summary_of("20261001T000000-mujoco-openarm-empty-idle-1", "x")
    errors = []

    def write_many():
        try:
            for _ in range(50):
                store.write(summary)
        except OSError as exc:
            errors.append(exc)

    threads = [threading.Thread(target=write_many) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert errors == [] and store.read(summary["run_id"]) == summary
    assert not list(store.run_dir(summary["run_id"]).glob("*.tmp"))


# -- round-3 start-window races -------------------------------------------------


def run_threads(run_id: str, before: set) -> list[str]:
    """Threads started for ``run_id`` since ``before``. Earlier tests may share the id."""
    return [t.name for t in set(threading.enumerate()) - before if t.name.endswith(run_id)]


def test_a_down_during_docker_run_stops_the_container_once_it_exists(agent, runner):
    runner.start_gate = threading.Event()
    replies = []
    before = set(threading.enumerate())
    upper = threading.Thread(target=lambda: replies.append(ask(agent, "up", SPEC)))
    upper.start()
    wait_for(lambda: runner.names() == ["start"])
    down = ask(agent, "down")
    assert down["ok"] is True and agent.status()["state"] == "idle"
    runner.start_gate.set()
    upper.join(5)
    run_id = replies[0]["run_id"]
    # docker run finished after the down: the agent stops what it created, once.
    assert [c for c in runner.calls if c[0] == "stop"] == [("stop", run_id)]
    assert runner.names()[-1] == "stop"
    assert run_threads(run_id, before) == []
    assert "list_controllers" not in runner.names()
    assert agent.status()["state"] == "idle"
    runner.start_gate = None
    assert ask(agent, "up", {**SPEC, "seed": 8})["ok"] is True


def test_a_down_during_the_startup_reset_never_starts_the_recorder(agent, runner, capsys):
    runner.reset_gate = threading.Event()
    run_id = ask(agent, "up", {**SPEC, "record": True})["run_id"]
    wait_for(lambda: "reset" in runner.names())
    assert ask(agent, "down")["ok"] is True
    runner.reset_gate.set()
    wait_for(lambda: not any(t.name == f"start-{run_id}" for t in threading.enumerate()))
    assert "start_recording" not in runner.names()
    assert "skipped recording" in capsys.readouterr().err
    assert agent.status()["state"] == "idle"


def test_an_expiry_that_loses_to_a_down_does_not_mark_the_run(agent, runner, store, monkeypatch):
    runner.outcome_gate = threading.Event()
    agent.max_run_s = 0.3
    expired = []
    real_down = agent.down

    def spy(run_id=None, **kwargs):
        if kwargs.get("expired"):
            expired.append(run_id)
        return real_down(run_id, **kwargs)

    monkeypatch.setattr(agent, "down", spy)
    run_id = up_running(agent)
    downer = threading.Thread(target=lambda: ask(agent, "down"))
    downer.start()
    wait_for(lambda: "outcome" in runner.names())  # the manual down holds the run
    wait_for(lambda: expired)  # the expiry fired, and was refused as busy
    runner.outcome_gate.set()
    downer.join(5)
    assert "max_duration_reached" not in store.read(run_id)["metrics"]


def test_a_down_during_a_failing_docker_run_still_stops_the_container(agent, runner):
    """docker run may create the container and then fail; the overtaken run is still stopped."""
    runner.start_gate = threading.Event()
    runner.fail["start"] = "docker run timed out after 90s"
    replies = []
    before = set(threading.enumerate())
    upper = threading.Thread(target=lambda: replies.append(ask(agent, "up", SPEC)))
    upper.start()
    wait_for(lambda: runner.names() == ["start"])
    assert ask(agent, "down")["ok"] is True
    after_down = len(runner.calls)
    runner.start_gate.set()
    upper.join(5)
    assert code(replies[0]) == "start_failed"
    names = runner.names()
    assert names.count("stop") == 1 and names[after_down:] == ["stop"]
    # The error reply carries no run id; no run thread of any id was started.
    assert run_threads("-idle-7", before) == []
    assert agent.status()["state"] == "idle"
    runner.start_gate = None
    del runner.fail["start"]
    assert ask(agent, "up", {**SPEC, "seed": 8})["ok"] is True


# -- description and asset ------------------------------------------------------


def test_description_is_the_running_runs_urdf_read_once(agent, runner):
    run_id = up_running(agent)
    first = ask(agent, "description", {})
    assert first == {"ok": True, "run_id": run_id, "urdf": URDF}
    ask(agent, "description", {})
    assert runner.names().count("description") == 1


def test_description_with_no_run_is_not_running(agent):
    assert code(ask(agent, "description", {})) == "not_running"


def test_a_description_not_yet_published_is_not_running_and_asked_again(agent, runner):
    up_running(agent)
    runner.fail["description"] = "no /robot_description was echoed"
    assert code(ask(agent, "description", {})) == "not_running"
    del runner.fail["description"]
    assert ask(agent, "description", {})["ok"] is True


def test_asset_serves_a_visual_mesh_the_description_names(agent, runner):
    run_id = up_running(agent)
    assert agent.answer("asset", json.dumps({"uri": "package://desc/meshes/arm.dae"}).encode()) == {
        "ok": True, "data": b"mesh:desc/meshes/arm.dae",
    }  # fmt: skip
    # Every visual mesh in one read, never the collision mesh.
    reads = [call for call in runner.calls if call[0] == "assets"]
    assert reads == [("assets", run_id, [("desc", "meshes/arm.dae"), ("desc", "meshes/base.dae")])]


def test_meshes_are_read_once_across_runs_of_one_image(agent, runner):
    up_running(agent)
    ask(agent, "asset", {"uri": "package://desc/meshes/arm.dae"})
    ask(agent, "down", {})
    up_running(agent, seed=2)
    assert ask(agent, "asset", {"uri": "package://desc/meshes/base.dae"})["ok"] is True
    assert runner.names().count("assets") == 1


def test_a_new_image_reads_the_meshes_again(agent, runner):
    up_running(agent)
    ask(agent, "asset", {"uri": "package://desc/meshes/arm.dae"})
    ask(agent, "down", {})
    runner.image = "sha256:two"
    up_running(agent, seed=2)
    ask(agent, "asset", {"uri": "package://desc/meshes/arm.dae"})
    assert runner.names().count("assets") == 2


def test_a_failed_mesh_read_is_internal_error(agent, runner):
    up_running(agent)
    runner.fail["assets"] = "docker exec failed"
    assert code(ask(agent, "asset", {"uri": "package://desc/meshes/arm.dae"})) == "internal_error"


@pytest.mark.parametrize(
    "uri",
    [
        "package://desc/meshes/base.stl",  # collision, not drawn
        "package://desc/meshes/other.dae",
        "package://desc/../etc/passwd",
        "/etc/passwd",
    ],
)
def test_asset_refuses_any_file_the_description_does_not_draw(agent, runner, uri):
    up_running(agent)
    assert code(ask(agent, "asset", {"uri": uri})) == "invalid_request"


def test_asset_needs_a_uri(agent):
    up_running(agent)
    assert code(ask(agent, "asset", {})) == "invalid_request"


def test_combination_refusal_fails_closed_on_a_malformed_scenarios_list():
    entries = {
        "backend": entry("mujoco"),
        "robot": {**entry("robot"), "supported_backends": ["mujoco"]},
        "scenario": entry("can_on_table"),
        "controller": {**entry("controller"), "scenarios": "can_on_table"},
    }
    assert (
        combination_refusal(entries) == "controller controller runs only on scenarios none; not can_on_table"
    )
