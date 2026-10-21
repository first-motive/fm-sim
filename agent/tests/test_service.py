"""The verb set over a real Zenoh session pair on localhost.

Two peer sessions in one process, joined over TCP on 127.0.0.1 with scouting
off, so nothing leaves the machine and no router is needed. The service side is
the same :class:`Fabric` the agent's ``main`` declares.
"""

from __future__ import annotations

import json
import socket
import threading
from pathlib import Path

import pytest
import zenoh

from fm_sim_agent import bridge
from fm_sim_agent.agent import VERBS, SimAgent
from fm_sim_agent.agent import MAX_BODY_BYTES
from fm_sim_agent.service import Fabric, Lane, bridge_of, build_parser, key_of, serve
from tests.fakes import SPEC, FakeRunner, wait_for

HOST = "fm_ws_01"
TIMEOUT_S = 5.0


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def peer(listen: str | None = None, connect: str | None = None) -> zenoh.Session:
    config = zenoh.Config()
    config.insert_json5("mode", json.dumps("peer"))
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5("listen/endpoints", json.dumps([listen] if listen else []))
    config.insert_json5("connect/endpoints", json.dumps([connect] if connect else []))
    return zenoh.open(config)


@pytest.fixture
def wire(store):
    endpoint = f"tcp/127.0.0.1:{free_port()}"
    server = peer(listen=endpoint)
    client = peer(connect=endpoint)
    runner = FakeRunner(store)
    agent = SimAgent(HOST, runner, store)
    agent.ready_poll_s = 0.01
    fabric = Fabric(server, agent, HOST, status_period_s=0.05)
    agent.fabric = fabric  # for the lane tests, which fill a lane directly
    # The two sessions find each other asynchronously; a status query answering
    # is the signal the route is up.
    wait_for(lambda: bool(get_all(client, "status", timeout_s=0.5)), timeout_s=10)
    try:
        yield client, agent, runner
    finally:
        fabric.close()
        client.close()
        server.close()


def get_all(
    client, verb: str, body: dict | None = None, timeout_s: float = TIMEOUT_S
) -> list[tuple[bool, str, dict]]:
    payload = json.dumps(body).encode() if body is not None else None
    replies = []
    for reply in client.get(key_of(HOST, verb), payload=payload, timeout=timeout_s):
        if reply.ok is not None:
            replies.append((True, str(reply.ok.key_expr), json.loads(reply.ok.payload.to_bytes())))
        else:
            raw = reply.err.payload.to_bytes()
            try:
                replies.append((False, "", json.loads(raw)))
            except ValueError:  # Zenoh's own error, a timeout: not the agent's reply
                replies.append(
                    (False, "", {"ok": False, "error": {"code": "zenoh", "message": raw.decode()}})
                )
    return replies


def get(client, verb, body=None):
    (reply,) = get_all(client, verb, body)
    return reply


def test_every_verb_has_a_queryable(wire):
    client, _agent, _runner = wire
    for verb in VERBS:
        assert get_all(client, verb), f"{verb} did not answer"


def test_a_full_run_over_the_wire(wire, store):
    client, agent, _runner = wire
    ok, key, status = get(client, "status")
    assert ok and key == f"fm/sim/{HOST}/status" and status["state"] == "idle"

    ok, key, catalog = get(client, "catalog")
    assert ok and key == f"fm/sim/{HOST}/catalog" and catalog["catalog"]["schema_version"] == 1

    ok, _, up = get(client, "up", SPEC)
    assert ok and up["ok"] is True
    run_id = up["run_id"]
    wait_for(lambda: agent.status()["state"] == "running")

    assert get(client, "reset", {"seed": 3}) == (True, f"fm/sim/{HOST}/reset", {"ok": True})

    ok, _, down = get(client, "down", {"run_id": run_id})
    assert ok and down["summary"]["run_id"] == run_id and down["summary"]["ended_at"]

    ok, _, runs = get(client, "runs", {"limit": 5})
    assert ok and [r["run_id"] for r in runs["runs"]] == [run_id]
    assert store.read(run_id) == down["summary"]


def test_a_refusal_is_an_error_reply_with_a_code(wire):
    client, _agent, _runner = wire
    ok, _, body = get(client, "up", {**SPEC, "robot": "../x"})
    assert ok is False
    assert body["ok"] is False and body["error"]["code"] == "invalid_spec" and body["error"]["message"]


def test_busy_over_the_wire(wire):
    client, agent, _runner = wire
    get(client, "up", SPEC)
    ok, _, body = get(client, "up", SPEC)
    assert ok is False and body["error"]["code"] == "busy"


def test_status_is_published(wire):
    client, _agent, _runner = wire
    seen = []
    subscriber = client.declare_subscriber(
        key_of(HOST, "status"), lambda s: seen.append(json.loads(s.payload.to_bytes()))
    )
    wait_for(lambda: seen)
    subscriber.undeclare()
    assert seen[0]["host"] == HOST and seen[0]["state"] == "idle"


def test_log_lines_are_published_under_the_run(wire):
    client, agent, runner = wire
    seen = []
    subscriber = client.declare_subscriber(
        f"fm/sim/{HOST}/logs/**", lambda s: seen.append((str(s.key_expr), json.loads(s.payload.to_bytes())))
    )
    runner.log_lines = ["[sim_loop-1] [INFO] [2.0] [sim_loop]: hello"]
    _, _, up = get(client, "up", SPEC)
    wait_for(lambda: seen)
    subscriber.undeclare()
    assert seen[0] == (
        f"fm/sim/{HOST}/logs/{up['run_id']}",
        {"t": 2.0, "level": "info", "node": "sim_loop", "msg": "hello"},
    )


def test_a_slow_up_does_not_hold_back_status(wire):
    """``up`` waits on Docker from the pool; status answers on Zenoh's thread meanwhile."""
    client, _agent, runner = wire
    runner.start_gate = threading.Event()
    replies = []
    upper = threading.Thread(target=lambda: replies.append(get(client, "up", SPEC)))
    upper.start()
    wait_for(lambda: runner.names() == ["start"])
    ok, _, status = get(client, "status")
    assert ok and status["state"] == "starting"
    runner.start_gate.set()
    upper.join(TIMEOUT_S)
    assert replies[0][0] is True


def fill(lane: Lane, gate: threading.Event) -> None:
    """Hold every slot of ``lane`` until ``gate`` is set."""
    while lane.submit(lambda: gate.wait(TIMEOUT_S)):
        pass


def test_a_full_lane_answers_busy_at_once(wire):
    """Past the lane's ceiling a caller is told so, not queued behind work it cannot see."""
    client, agent, runner = wire
    gate = threading.Event()
    fill(agent.fabric.lanes["work"], gate)
    try:
        ok, _, body = get(client, "runs")
        assert ok is False and body["error"]["code"] == "busy"
    finally:
        gate.set()
    wait_for(lambda: get(client, "runs")[0])


def test_down_does_not_queue_behind_the_work_lane(wire):
    client, agent, runner = wire
    _, _, up = get(client, "up", SPEC)
    wait_for(lambda: agent.status()["state"] == "running")
    gate = threading.Event()
    fill(agent.fabric.lanes["work"], gate)
    try:
        assert get(client, "runs")[2]["error"]["code"] == "busy"
        ok, _, down = get(client, "down")
        assert ok and down["summary"]["run_id"] == up["run_id"]
    finally:
        gate.set()


def test_serve_brings_the_active_run_down_on_the_way_out(store):
    """What main does after the stop signal, on a real client session."""
    endpoint = f"tcp/127.0.0.1:{free_port()}"
    router = peer(listen=endpoint)
    runner = FakeRunner(store)
    agent = SimAgent(HOST, runner, store)
    agent.ready_poll_s = 0.01
    seen = {}

    def wait() -> None:
        seen["run_id"] = agent.answer("up", json.dumps(SPEC).encode())["run_id"]
        wait_for(lambda: agent.status()["state"] == "running")

    try:
        serve(agent, HOST, endpoint, wait=wait)
    finally:
        router.close()
    assert ("stop", seen["run_id"]) in runner.calls
    assert store.read(seen["run_id"])["ended_at"] is not None
    assert agent.status()["state"] == "idle"


def test_an_oversized_body_is_refused_before_it_is_copied(wire):
    client, _agent, runner = wire
    (reply,) = list(client.get(key_of(HOST, "up"), payload=b"x" * (MAX_BODY_BYTES + 1), timeout=TIMEOUT_S))
    body = json.loads(reply.err.payload.to_bytes())
    assert body["error"]["code"] == "invalid_request"
    assert runner.calls == []


def test_a_lane_logs_a_failing_worker_and_frees_its_slot(capsys):
    lane = Lane("t", 1, 1)
    done = threading.Event()

    def broken():
        try:
            raise RuntimeError("worker bug")
        finally:
            done.set()

    assert lane.submit(broken)
    done.wait(5)
    wait_for(lambda: lane.submit(lambda: None))  # the one slot came back
    lane.shutdown()
    assert "worker bug" in capsys.readouterr().err


def test_a_lane_that_cannot_submit_frees_its_slot():
    lane = Lane("t", 1, 1)
    lane.shutdown()
    with pytest.raises(RuntimeError):
        lane.submit(lambda: None)
    assert lane.slots.acquire(blocking=False)


# -- the bridge config ---------------------------------------------------------


def bridge_files(directory: Path) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "bridge-sim.json5").write_text("{}")
    (directory / "cyclonedds.xml").write_text("<CycloneDDS/>")
    return directory / "bridge-sim.json5"


def test_a_run_on_the_host_network_gets_no_bridge(tmp_path, monkeypatch):
    monkeypatch.setenv(bridge.CONFIG_ENV, str(bridge_files(tmp_path)))
    assert bridge_of(build_parser().parse_args([])) is None


def test_without_its_files_the_agent_says_live_topics_are_off(tmp_path, monkeypatch, capsys):
    config = bridge_files(tmp_path)
    (tmp_path / "cyclonedds.xml").unlink()
    monkeypatch.setenv(bridge.CONFIG_ENV, str(config))
    assert bridge_of(build_parser().parse_args(["--no-host-network"])) is None
    assert capsys.readouterr().out == (
        f"fm-sim-agent: live topics are off: no {tmp_path / 'cyclonedds.xml'} (fm-comms renders them)\n"
    )


def test_the_flag_names_the_bridge_config_before_the_environment(tmp_path, monkeypatch):
    from_env = bridge_files(tmp_path / "env")
    from_flag = bridge_files(tmp_path / "flag")
    monkeypatch.setenv(bridge.CONFIG_ENV, str(from_env))
    args = build_parser().parse_args(["--no-host-network"])
    assert bridge_of(args) == from_env.resolve()
    args = build_parser().parse_args(["--no-host-network", "--bridge-config", str(from_flag)])
    assert bridge_of(args) == from_flag.resolve()


def test_the_default_bridge_config_is_fm_comms_user_config(monkeypatch):
    monkeypatch.delenv(bridge.CONFIG_ENV, raising=False)
    assert bridge.config_path() == (Path.home() / ".config/fm-comms/bridge-sim.json5").resolve()
