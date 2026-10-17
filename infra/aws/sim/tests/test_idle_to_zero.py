"""Host idle check: run detection, the idle clock, and leaving the group."""

from __future__ import annotations

import io
import re
import subprocess
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

import idle_to_zero
from idle_to_zero import active_run, check, decide, instance_id, read_last_active, write_last_active

NOW = datetime(2026, 10, 17, 12, 0, tzinfo=timezone.utc)
REPO = Path(__file__).resolve().parents[4]


class FakeRun:
    """Stands in for subprocess.run: answers docker ps, records every argv."""

    def __init__(
        self, docker_stdout: str = "", docker_rc: int = 0, aws_rc: int = 0, raises: bool = False
    ) -> None:
        self.docker_stdout, self.docker_rc, self.aws_rc, self.raises = (
            docker_stdout,
            docker_rc,
            aws_rc,
            raises,
        )
        self.calls: list[list[str]] = []

    def __call__(self, argv: list[str]) -> subprocess.CompletedProcess:
        self.calls.append(argv)
        if argv[0] == "docker":
            if self.raises:
                raise FileNotFoundError("docker")
            return subprocess.CompletedProcess(argv, self.docker_rc, self.docker_stdout, "")
        return subprocess.CompletedProcess(argv, self.aws_rc, "", "denied" if self.aws_rc else "")


def test_run_label_matches_the_agent():
    runner = (REPO / "agent/src/fm_sim_agent/runner.py").read_text()
    assert re.search(r'^RUN_LABEL = "([^"]+)"$', runner, re.M).group(1) == idle_to_zero.RUN_LABEL


def test_active_run_asks_docker_for_running_run_containers():
    run = FakeRun(docker_stdout="3f2a\n")
    assert active_run(run) is True
    assert run.calls == [
        ["docker", "ps", "--quiet", "--filter", "label=fm.sim.run_id", "--filter", "status=running"]
    ]


@pytest.mark.parametrize(
    "run, expected",
    [(FakeRun(docker_stdout=""), False), (FakeRun(docker_rc=1), None), (FakeRun(raises=True), None)],
)
def test_active_run_outcomes(run, expected):
    assert active_run(run) is expected


def test_first_check_starts_the_clock():
    assert decide(NOW, None, False, 15) == (NOW, False)


def test_a_run_resets_the_clock():
    assert decide(NOW, NOW - timedelta(hours=3), True, 15) == (NOW, False)


@pytest.mark.parametrize("active", [False, None])
def test_idle_period_ends_in_leaving(active):
    since = NOW - timedelta(minutes=15)
    assert decide(NOW, since, active, 15) == (since, True)
    assert decide(NOW - timedelta(seconds=1), since, active, 15) == (since, False)


def test_state_round_trip_and_corrupt_state(tmp_path):
    state = tmp_path / "idle.json"
    write_last_active(state, NOW)
    assert read_last_active(state) == NOW
    state.write_text("{not json")
    assert read_last_active(state) is None
    state.write_text('{"last_active": "2026-10-17T12:00:00"}')  # no zone: refused
    assert read_last_active(state) is None


def test_check_leaves_after_the_idle_period(tmp_path):
    state = tmp_path / "idle.json"
    write_last_active(state, NOW - timedelta(minutes=16))
    run = FakeRun()
    assert (
        check(
            max_lifetime_minutes=480,
            uptime_s=600.0,
            region="us-test-1",
            idle_minutes=15,
            state_file=state,
            now=NOW,
            run=run,
            get_instance_id=lambda: "i-0abc",
        )
        == 0
    )
    assert run.calls[-1] == [
        "aws", "--region", "us-test-1", "autoscaling", "terminate-instance-in-auto-scaling-group",
        "--instance-id", "i-0abc", "--should-decrement-desired-capacity",
    ]  # fmt: skip


def test_check_stays_while_a_run_is_active(tmp_path):
    state = tmp_path / "idle.json"
    write_last_active(state, NOW - timedelta(hours=2))
    run = FakeRun(docker_stdout="3f2a")
    assert (
        check(
            region="r",
            idle_minutes=15,
            max_lifetime_minutes=480,
            uptime_s=600.0,
            state_file=state,
            now=NOW,
            run=run,
            get_instance_id=pytest.fail,
        )
        == 0
    )
    assert [c[0] for c in run.calls] == ["docker"]
    assert read_last_active(state) == NOW


def test_check_reports_a_failed_terminate(tmp_path):
    state = tmp_path / "idle.json"
    write_last_active(state, NOW - timedelta(hours=1))
    run = FakeRun(aws_rc=255)
    assert (
        check(
            max_lifetime_minutes=480,
            uptime_s=600.0,
            region="r",
            idle_minutes=15,
            state_file=state,
            now=NOW,
            run=run,
            get_instance_id=lambda: "i-0abc",
        )
        == 1
    )


class FakeResponse(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def test_instance_id_uses_imdsv2():
    seen = []

    def opener(request, timeout):
        seen.append(request)
        return FakeResponse(b"token" if request.get_method() == "PUT" else b"i-0123abcd\n")

    assert instance_id(opener) == "i-0123abcd"
    assert seen[0].get_method() == "PUT"
    assert seen[1].get_header("X-aws-ec2-metadata-token") == "token"


def test_instance_id_refuses_an_odd_answer():
    with pytest.raises(RuntimeError):
        instance_id(lambda request, timeout: FakeResponse(b"i-0abc; rm -rf /"))


@pytest.mark.parametrize(
    "environ",
    [
        {},
        {"FM_SIM_CLOUD_REGION": "r", "FM_SIM_IDLE_MINUTES": "x"},
        {"FM_SIM_CLOUD_REGION": "r", "FM_SIM_IDLE_MINUTES": "0"},
        {"FM_SIM_CLOUD_REGION": "r", "FM_SIM_IDLE_MINUTES": "15"},
        {"FM_SIM_CLOUD_REGION": "r", "FM_SIM_IDLE_MINUTES": "15", "FM_SIM_MAX_LIFETIME_MINUTES": "x"},
    ],
)
def test_main_refuses_bad_config(monkeypatch, environ):
    for key in ("FM_SIM_CLOUD_REGION", "FM_SIM_IDLE_MINUTES", "FM_SIM_MAX_LIFETIME_MINUTES"):
        monkeypatch.delenv(key, raising=False)
    for key, value in environ.items():
        monkeypatch.setenv(key, value)
    assert idle_to_zero.main(["check"]) == 2


def test_check_leaves_at_the_lifetime_even_while_busy(tmp_path):
    run = FakeRun(docker_stdout="3f2a")
    code = check(
        region="r",
        idle_minutes=15,
        max_lifetime_minutes=480,
        uptime_s=480 * 60,
        state_file=tmp_path / "idle.json",
        now=NOW,
        run=run,
        get_instance_id=lambda: "i-0abc",
    )
    assert code == 0
    assert [c[0] for c in run.calls] == ["aws"]
    assert "--should-decrement-desired-capacity" in run.calls[0]


def test_main_check_reads_its_config_from_the_environment(monkeypatch, tmp_path):
    seen = {}
    monkeypatch.setenv("FM_SIM_CLOUD_REGION", "us-test-1")
    monkeypatch.setenv("FM_SIM_IDLE_MINUTES", "15")
    monkeypatch.setenv("FM_SIM_MAX_LIFETIME_MINUTES", "480")
    monkeypatch.setenv("FM_SIM_IDLE_STATE_FILE", str(tmp_path / "idle.json"))
    monkeypatch.setattr(idle_to_zero, "read_uptime_s", lambda: 60.0)
    monkeypatch.setattr(idle_to_zero, "check", lambda **kwargs: seen.update(kwargs) or 0)
    assert idle_to_zero.main(["check"]) == 0
    assert (seen["region"], seen["idle_minutes"], seen["max_lifetime_minutes"]) == ("us-test-1", 15, 480)
    assert seen["state_file"] == tmp_path / "idle.json"
    assert seen["uptime_s"] == 60.0


def test_read_uptime(tmp_path):
    uptime = tmp_path / "uptime"
    uptime.write_text("1234.56 9876.54\n")
    assert idle_to_zero.read_uptime_s(uptime) == 1234.56
