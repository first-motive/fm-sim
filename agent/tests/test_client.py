"""``fm sim``: its flags, a remote host over a real Zenoh pair, and ``--host local``.

The remote half runs against the same :class:`Fabric` the agent serves, on two
in-process peer sessions (as ``test_service`` does). The local half runs the
in-process agent :class:`Local` wraps, with the fake runner in place of Docker.
"""

from __future__ import annotations

import json
import threading

import pytest

from fm_sim_agent import client, trial
from fm_sim_agent.agent import SimAgent
from fm_sim_agent.client import Local, Remote, main, parse_args
from fm_sim_agent.runner import DockerRunner
from fm_sim_agent.service import Fabric
from tests.fakes import SPEC, FakeRunner, wait_for
from tests.test_service import HOST, free_port, peer

OTHER_RUN = "20261016T110000-mujoco-openarm-empty-idle-0"


@pytest.fixture(autouse=True)
def fast_polls(monkeypatch):
    monkeypatch.setattr(trial, "POLL_S", 0.01)


# -- flags -------------------------------------------------------------------


def test_host_is_required_and_becomes_a_key_segment():
    with pytest.raises(SystemExit):
        parse_args(["status"])
    assert parse_args(["status", "--host", "fm-ws-01"]).host == "fm_ws_01"
    assert parse_args(["status", "--host", "local"]).host == "local"


@pytest.mark.parametrize(
    "argv",
    [
        ["status", "--host", "fm/*"],
        ["up", "--host", "fm-ws-01", "--image", "x"],
        ["up", "--host", "fm-ws-01", "--duration", "5"],
        ["up", "--host", "local", "--duration", "0"],
        ["down", "--host", "local", "--run-id", "../etc"],
        ["trial", "--host", "local"],
        ["trial", "--host", "local", "--seeds", "0..2", "--count", "3"],
        ["trial", "--host", "local", "--seeds", "1,1"],
        ["trial", "--host", "local", "--count", "3", "--run-timeout", "0"],
        ["nope", "--host", "local"],
        ["up", "--host", "local", "--scenario", "../etc"],
        ["trial", "--host", "local", "--count", "2", "--robot", "Open-Arm"],
    ],
)
def test_bad_flags_are_usage_errors(argv):
    with pytest.raises(SystemExit) as caught:
        parse_args(argv)
    assert caught.value.code == 2


def test_spec_flags_default_to_the_old_sim_verbs_robot_and_backend():
    args = parse_args(["up", "--host", "local"])
    assert client.spec_of(args, args.seed) == {
        "backend": "mujoco", "robot": "openarm", "scenario": "empty", "controller": "idle",
        "seed": 0, "record": False,
    }  # fmt: skip


def test_trial_seeds_come_from_either_flag():
    assert parse_args(["trial", "--host", "local", "--seeds", "0..2,5"]).seed_list == [0, 1, 2, 5]
    assert parse_args(["trial", "--host", "local", "--count", "3", "--seed-start", "10"]).seed_list == [
        10,
        11,
        12,
    ]


# -- a remote host -----------------------------------------------------------


class KeptOpen(Remote):
    """The fixture owns the session; ``main`` closing it would end the next call."""

    def close(self) -> None:
        pass


@pytest.fixture
def remote(store):
    endpoint = f"tcp/127.0.0.1:{free_port()}"
    server = peer(listen=endpoint)
    session = peer(connect=endpoint)
    runner = FakeRunner(store)
    agent = SimAgent(HOST, runner, store)
    agent.ready_poll_s = 0.01
    agent.min_free_bytes = 0
    fabric = Fabric(server, agent, HOST, status_period_s=0.05)
    host = KeptOpen(session, HOST)

    def answers() -> bool:
        try:
            return host.call("status").get("ok") is True
        except client.NoAnswer:
            return False

    wait_for(answers, timeout_s=10)
    try:
        yield host, agent, runner
    finally:
        fabric.close()
        session.close()
        server.close()


def run(argv, host, capsys):
    code = main(argv, connect=lambda _args: host)
    out, err = capsys.readouterr()
    return code, out, err


def test_remote_up_status_down_go_through_the_agents_verbs(remote, capsys):
    host, agent, runner = remote
    code, out, _ = run(["up", "--host", "fm-ws-01", "--seed", "7"], host, capsys)
    assert code == 0 and out.startswith("started ")
    run_id = out.split()[1]
    assert runner.calls[0] == ("start", run_id, SPEC)
    wait_for(lambda: agent.status()["state"] == "running")

    code, out, _ = run(["status", "--host", "fm-ws-01", "--json"], host, capsys)
    assert code == 0 and json.loads(out)["run_id"] == run_id

    code, out, _ = run(["down", "--host", "fm-ws-01"], host, capsys)
    assert code == 0 and out.strip() == f"{run_id}: success: placed (4.5 s)"
    assert agent.status()["state"] == "idle"


def test_remote_refusal_is_exit_one_with_its_code(remote, capsys):
    host, _, _ = remote
    code, _, err = run(["down", "--host", "fm-ws-01"], host, capsys)
    assert code == 1 and err.startswith("refused (not_running)")


def test_remote_catalog_lists_each_dimension(remote, capsys):
    host, _, _ = remote
    code, out, _ = run(["catalog", "--host", "fm-ws-01"], host, capsys)
    assert code == 0
    assert "backends: mujoco, gazebo, isaac (unavailable)" in out.splitlines()


def test_a_host_that_does_not_answer_is_a_precondition_failure(remote, capsys, monkeypatch):
    host, _, _ = remote
    monkeypatch.setattr(client, "QUERY_TIMEOUT_S", 0.3)
    code, _, err = run(["status", "--host", "fm-ws-02"], KeptOpen(host.session, "fm_ws_02"), capsys)
    assert code == 3 and "did not answer" in err


def test_remote_logs_follow_what_the_agent_publishes(remote):
    host, agent, _ = remote
    seen, stop = [], threading.Event()
    follower = threading.Thread(target=host.follow_logs, args=(OTHER_RUN, seen.append, stop))
    follower.start()
    try:
        wait_for(lambda: (agent.log_sink(OTHER_RUN, {"t": 1.0, "msg": "up"}), bool(seen))[1])
    finally:
        stop.set()
        follower.join(timeout=5)
    assert seen[0]["msg"] == "up"


def test_remote_trial_gives_each_run_its_time_then_downs_it(remote, capsys, tmp_path):
    host, _, runner = remote
    report_path = tmp_path / "report.json"
    argv = [
        "trial",
        "--host",
        "fm-ws-01",
        "--seeds",
        "0,1",
        "--run-timeout",
        "0.3",
        "--report",
        str(report_path),
    ]
    code, out, _ = run(argv, host, capsys)
    assert code == 0
    report = json.loads(report_path.read_text())
    assert [row["seed"] for row in report["runs"]] == [0, 1]
    assert report["success_rate"] == 1.0  # the fake latches a success; down reads it
    assert runner.names().count("stop") == 2
    assert "success 2/2 (100.0%), 0 unscored" in out


# -- --host local ------------------------------------------------------------


@pytest.fixture
def local(store, runner):
    agent = SimAgent("local", runner, store)
    agent.ready_poll_s = 0.01
    agent.min_free_bytes = 0
    return Local(agent)


def test_local_mode_builds_the_hosts_agent_and_docker_runner_without_host_network(tmp_path):
    host = client.local_host("fm-sim:jazzy", tmp_path / "runs")
    assert isinstance(host.runner, DockerRunner)
    assert host.agent.host == host.runner.host == "local"
    assert host.runner.host_network is False and host.runner.image == "fm-sim:jazzy"
    assert host.agent.store.root == (tmp_path / "runs").resolve() == host.runner.runs_dir


def test_local_up_runs_in_the_foreground_until_its_outcome(local, runner, capsys):
    runner.log_lines = ["[sim_loop-1] [INFO] [1.5] [sim_loop]: up\n"]
    code, out, _ = run(["up", "--host", "local", "--seed", "7"], local, capsys)
    assert code == 0
    assert "success: placed (4.5 s)" in out.splitlines()[-1]
    assert "[info] sim_loop: up" in out
    names = runner.names()
    assert names[0] == "start" and "stop" in names
    assert local.agent.status()["run_id"] is None


def test_local_up_without_an_outcome_ends_on_its_duration(local, runner, capsys):
    runner.outcome_value = None
    code, out, _ = run(["up", "--host", "local", "--duration", "0.2"], local, capsys)
    assert code == 0 and out.splitlines()[-1].endswith(": no outcome")
    assert "stop" in runner.names()


def test_local_up_refuses_while_another_process_holds_a_run(local, runner, capsys):
    runner.live = [OTHER_RUN]
    code, _, err = run(["up", "--host", "local"], local, capsys)
    assert code == 1 and "refused (busy)" in err and OTHER_RUN in err
    assert "start" not in runner.names()


def test_local_status_shows_another_processs_run(local, runner, capsys):
    runner.live = [OTHER_RUN]
    code, out, _ = run(["status", "--host", "local"], local, capsys)
    assert code == 0 and out.startswith(f"local: running {OTHER_RUN}")


def test_local_down_clears_another_processs_run(local, runner, store, capsys):
    store.write({
        "run_id": OTHER_RUN, "host": "local", "spec": SPEC, "started_at": "2026-10-16T11:00:00Z",
        "ended_at": None, "outcome": None, "mcap": None, "metrics": {},
    })  # fmt: skip
    runner.live = [OTHER_RUN]
    code, out, _ = run(["down", "--host", "local", "--json"], local, capsys)
    reply = json.loads(out)
    assert code == 0 and reply["removed"] == ["fm-sim-run-old"]
    assert reply["summary"]["run_id"] == OTHER_RUN and reply["summary"]["ended_at"] is not None
    assert "stop_stale" in runner.names()


def test_local_down_with_nothing_running_is_refused(local, capsys):
    code, _, err = run(["down", "--host", "local"], local, capsys)
    assert code == 1 and "not_running" in err


def test_local_logs_read_the_containers_own_stream(local, runner, capsys):
    runner.live = [OTHER_RUN]
    runner.log_lines = ["[talker-1] [WARN] [2.0] [talker]: slow\n"]
    runner.start(OTHER_RUN, SPEC)  # queues the lines
    runner.exit()
    code, out, _ = run(["logs", "--host", "local"], local, capsys)
    assert code == 0 and "[warn] talker: slow" in out


def test_closing_local_downs_a_run_it_still_holds(local, runner):
    local.agent.up(dict(SPEC))
    local.close()
    assert local.agent.status()["run_id"] is None
    assert "stop" in runner.names()


def test_local_trial_runs_seeds_one_at_a_time_and_scores_them(local, runner, capsys, tmp_path):
    def outcome(run_id):
        seed = int(run_id.rsplit("-", 1)[1])
        return {
            "success": seed % 2 == 0,
            "reason": "placed" if seed % 2 == 0 else "dropped",
            "elapsed_s": 1.0,
        }

    runner.outcome = outcome
    report_path = tmp_path / "trial.json"
    argv = ["trial", "--host", "local", "--count", "4", "--report", str(report_path)]
    code, out, _ = run(argv, local, capsys)
    assert code == 0
    report = json.loads(report_path.read_text())
    assert [row["success"] for row in report["runs"]] == [True, False, True, False]
    assert report["success_rate"] == 0.5 and report["complete"] is True
    # One run per host: every start after the first follows the previous run's stop.
    lifecycle = [name for name in runner.names() if name in ("start", "stop")]
    assert lifecycle == ["start", "stop"] * 4
    assert "success 2/4 (50.0%), 0 unscored" in out
    assert f"report: {report_path}" in out


def test_local_down_with_another_run_id_leaves_that_run_alone(local, runner, capsys):
    runner.live = [OTHER_RUN]
    wrong = "20261016T120000-mujoco-openarm-empty-idle-9"
    code, _, err = run(["down", "--host", "local", "--run-id", wrong], local, capsys)
    assert code == 1 and "refused (run_mismatch)" in err
    assert "stop_stale" not in runner.names() and "stop" not in runner.names()


def test_interrupting_a_local_up_mid_start_still_downs_the_run(local, runner, capsys, monkeypatch):
    original = runner.start

    def interrupted(run_id, spec):
        original(run_id, spec)
        raise KeyboardInterrupt

    monkeypatch.setattr(runner, "start", interrupted)
    code, _, _ = run(["up", "--host", "local"], local, capsys)
    assert code == 130
    assert "stop" in runner.names()
    assert local.agent.status()["run_id"] is None


def test_safe_keeps_a_field_on_one_unreordered_line():
    assert client.safe("a\nb\tc\x1b[2Jd\u202eе\u2066f\u2069") == "ab" + "c[2Jd" + "е" + "f"


class Payload:
    def __init__(self, raw: bytes):
        self.raw = raw

    def __len__(self):
        return len(self.raw)

    def to_bytes(self):
        return self.raw


def test_an_oversized_or_non_object_payload_is_not_decoded():
    assert client.decode(Payload(b'{"ok": true}')) == {"ok": True}
    assert client.decode(Payload(b"[1]")) is None
    assert client.decode(Payload(b"x" * (client.MAX_PAYLOAD_BYTES + 1))) is None
    assert client.decode(None) is None


class NamesABadRun:
    host = "fm_ws_01"

    def call(self, verb, body=None):
        return {"ok": True, "state": "running", "run_id": "*"}

    def follow_logs(self, *_args):
        raise AssertionError("must not subscribe")

    def close(self):
        pass


def test_logs_refuse_a_run_id_the_host_made_up(capsys):
    code, _, err = run(["logs", "--host", "fm-ws-01"], NamesABadRun(), capsys)
    assert code == 1 and "invalid_reply" in err
