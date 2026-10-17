"""Seeded trials: seed parsing, one driven run, and the report a trial writes."""

from __future__ import annotations

import pytest

from fm_sim_agent import trial
from fm_sim_agent.agent import error_body
from fm_sim_agent.spec import SpecError
from fm_sim_agent.trial import Refused, aggregate, drive, parse_seeds, row_of, run_trial, seed_run
from tests.fakes import SPEC

RUN_ID = "20261016T120000-mujoco-openarm-empty-idle-7"


def test_seeds_take_ranges_lists_and_mixes():
    assert parse_seeds("0..3") == [0, 1, 2, 3]
    assert parse_seeds("0,3,7") == [0, 3, 7]
    assert parse_seeds("0..2, 10") == [0, 1, 2, 10]
    assert seed_run(3, 10) == [10, 11, 12]


@pytest.mark.parametrize("text", ["5..1", "1,1", "0..2,2", "a", "-1", "", "0..1000", "4294967296"])
def test_seeds_refuse_what_is_not_a_seed_list(text):
    with pytest.raises(SpecError):
        parse_seeds(text)


@pytest.mark.parametrize(("count", "start"), [(0, 0), (1001, 0), (2, 2**32 - 1), (1, -1)])
def test_seed_run_refuses_out_of_range(count, start):
    with pytest.raises(SpecError):
        seed_run(count, start)


class ScriptedHost:
    """A host whose replies a test writes out; records every call."""

    def __init__(self, statuses, down=None, runs=(), up=None, outcome=None):
        self.statuses = list(statuses)
        self.down = down or {"ok": True, "summary": {"run_id": RUN_ID, "outcome": None, "metrics": {}}}
        self.runs = list(runs)
        self.up = up or {"ok": True, "run_id": RUN_ID}
        self.outcome_value = outcome
        self.calls: list[tuple] = []

    def call(self, verb, body=None):
        self.calls.append((verb, body))
        if verb == "up":
            return self.up
        if verb == "status":
            status = self.statuses.pop(0) if len(self.statuses) > 1 else self.statuses[0]
            if isinstance(status, BaseException):
                raise status
            return status
        if verb == "down":
            return self.down
        if verb == "runs":
            return {"ok": True, "runs": self.runs}
        raise AssertionError(verb)

    def outcome(self, run_id):
        return self.outcome_value

    def verbs(self):
        return [verb for verb, _ in self.calls]


RUNNING = {"ok": True, "state": "running", "run_id": RUN_ID}
IDLE = {"ok": True, "state": "error", "run_id": None}


def test_drive_ends_on_an_outcome_and_downs_its_own_run():
    host = ScriptedHost([RUNNING], outcome={"success": True})
    summary = drive(host, SPEC, timeout_s=None, sleep=lambda _s: None)
    assert host.verbs() == ["up", "status", "down"]
    assert host.calls[-1] == ("down", {"run_id": RUN_ID})
    assert summary["run_id"] == RUN_ID


def test_drive_without_an_outcome_waits_out_its_time():
    now = [0.0]

    def sleep(seconds):
        now[0] += seconds

    host = ScriptedHost([RUNNING])
    drive(host, SPEC, timeout_s=3.0, poll_s=1.0, clock=lambda: now[0], sleep=sleep)
    assert host.verbs() == ["up", "status", "status", "status", "down"]


def test_drive_finds_the_summary_of_a_run_that_ended_on_its_own():
    filed = {"run_id": RUN_ID, "outcome": None, "metrics": {"wall_s": 2.0}}
    host = ScriptedHost([IDLE], down=error_body("not_running", "no run"), runs=[{"run_id": "other"}, filed])
    assert drive(host, SPEC, timeout_s=None, sleep=lambda _s: None) == filed
    assert host.verbs() == ["up", "status", "down", "runs"]


def test_drive_downs_the_run_when_interrupted():
    host = ScriptedHost([KeyboardInterrupt()])
    with pytest.raises(KeyboardInterrupt):
        drive(host, SPEC, timeout_s=None, sleep=lambda _s: None)
    assert host.verbs() == ["up", "status", "down"]


def test_drive_raises_a_refused_up_without_downing():
    host = ScriptedHost([RUNNING], up=error_body("busy", "run x is active"))
    with pytest.raises(Refused) as caught:
        drive(host, SPEC, timeout_s=None)
    assert caught.value.code == "busy"
    assert host.verbs() == ["up"]


def test_row_reads_the_outcome_and_metrics():
    summary = {
        "run_id": RUN_ID,
        "outcome": {"success": True, "reason": "placed", "elapsed_s": 4.5},
        "metrics": {"wall_s": 9.0, "seed_applied": True},
    }
    assert row_of(7, summary) == {
        "seed": 7, "run_id": RUN_ID, "success": True, "reason": "placed",
        "elapsed_s": 4.5, "wall_s": 9.0, "seed_applied": True,
    }  # fmt: skip
    assert row_of(7, {"run_id": RUN_ID, "outcome": None})["success"] is None


def test_aggregate_counts_an_unscored_run_as_a_failure():
    rows = [
        row_of(0, {"outcome": {"success": True}}),
        row_of(1, {"outcome": {"success": False}}),
        row_of(2, {"outcome": None}),
        row_of(3, {"outcome": {"success": True}}),
    ]
    report = aggregate("fm_ws_01", SPEC, [0, 1, 2, 3, 4], rows, "2026-10-17T00:00:00Z")
    assert report["successes"] == 2 and report["unscored"] == 1 and report["attempted"] == 4
    assert report["success_rate"] == 0.5
    assert report["complete"] is False  # seed 4 never ran
    assert "seed" not in report["spec"] and report["spec"]["scenario"] == "empty"


def test_a_busy_host_stops_the_trial_with_what_ran_kept():
    host = ScriptedHost([RUNNING], outcome={"success": True})
    ups = iter([{"ok": True, "run_id": RUN_ID}, error_body("busy", "taken")])
    host.call = (
        lambda original: lambda verb, body=None: next(ups) if verb == "up" else original(verb, body)
    )(host.call)
    report, stopped = run_trial(host, "fm_ws_01", SPEC, [0, 1, 2], sleep=lambda _s: None)
    assert isinstance(stopped, Refused) and stopped.code == "busy"
    assert report["attempted"] == 1 and report["complete"] is False


def test_poll_interval_is_read_when_a_run_is_driven(monkeypatch):
    slept = []
    monkeypatch.setattr(trial, "POLL_S", 0.25)
    host = ScriptedHost([{"ok": True, "state": "starting", "run_id": RUN_ID}, IDLE])
    drive(host, SPEC, timeout_s=None, sleep=slept.append)
    assert slept == [0.25]


def test_a_refused_down_with_no_summary_says_so_in_the_row():
    host = ScriptedHost([IDLE], down=error_body("not_running", "no run"), runs=[])
    summary = drive(host, SPEC, timeout_s=None, sleep=lambda _s: None)
    row = row_of(7, summary)
    assert row["success"] is None and row["reason"] == "down refused: not_running"
