"""``fm sim host status|pull|start|stop local``, against a stand-in ``docker``
and a stand-in agent.

The stand-in ``docker`` is a real executable first on ``PATH`` that records its
argv and answers from a scenario in the environment. The stand-in agent is a
real process, spawned, signalled and identified through ``ps`` exactly as the
real one is; it runs as the ``fm-sim-agent`` entry point
(``<python> …/fm-sim-agent --host <this machine> --label local …``) instead of
``-m fm_sim_agent.service``, the other form an agent's command line takes. Nothing reaches Docker or a router.
"""

from __future__ import annotations

import json
import os
import signal
import stat
import subprocess
import sys
import textwrap
import threading
import time

import pytest

from fm_sim_agent import card as card_module
from fm_sim_agent import client, service, this_machine
from fm_sim_agent.client import main, parse_args

REF = "ghcr.io/first-motive/fm-sim:jazzy"
REPO = "ghcr.io/first-motive/fm-sim"
PIN = "sha256:" + "a" * 64
OLD = "sha256:" + "b" * 64
RUN_ID = "20261016T120000-mujoco-openarm-empty-idle-7"
#: The machine card every test but the naming ones reads: ``nish-mbp``.
MACHINE = "nish_mbp"

FAKE_DOCKER = textwrap.dedent(
    """\
    import json, os, sys
    args = sys.argv[1:]
    with open(os.environ["FAKE_DOCKER_LOG"], "a") as log:
        log.write(json.dumps(args) + "\\n")
    scenario = json.loads(os.environ["FAKE_DOCKER"])
    pulled = os.environ["FAKE_DOCKER_LOG"] + ".pulled"
    if args[:2] == ["context", "show"]:
        print(scenario.get("context", "orbstack"))
    elif args[0] == "info":
        if scenario.get("info_error"):
            print(scenario["info_error"], file=sys.stderr)
            sys.exit(1)
        print(scenario.get("os", "OrbStack"))
    elif args[:2] == ["image", "inspect"]:
        digests = json.load(open(pulled)) if os.path.exists(pulled) else scenario.get("digests")
        if digests is None:
            print("Error: No such image: " + args[-1], file=sys.stderr)
            sys.exit(1)
        print(json.dumps(digests))
    elif args[0] == "pull":
        for line in scenario.get("pull_lines", []):
            print(line, flush=True)
        if scenario.get("pull_rc", 0) == 0 and "pulled_digests" in scenario:
            json.dump(scenario["pulled_digests"], open(pulled, "w"))
        sys.exit(scenario.get("pull_rc", 0))
    elif args[0] == "tag":
        sys.exit(scenario.get("tag_rc", 0))
    elif args[0] == "ps":
        print("\\n".join(scenario.get("ps_all" if "--all" in args else "ps", [])))
    elif args[:2] == ["rm", "--force"] or args[0] == "stop":
        pass
    else:
        print("unknown command", file=sys.stderr)
        sys.exit(1)
    """
)

#: Serves until SIGTERM, then "downs" its run and exits; FAKE_AGENT_MODE picks
#: an agent that exits at once or one that ignores SIGTERM.
FAKE_AGENT = textwrap.dedent(
    """\
    import os, signal, sys, time
    mode = os.environ.get("FAKE_AGENT_MODE", "serve")
    if mode == "exit":
        print("fm-sim-agent: FM_ROUTER_ENDPOINT is set neither in the environment nor in /etc/fm-comms.env")
        sys.exit(1)
    def down(*_):
        print("fake-agent: down on stop", flush=True)
        sys.exit(0)
    signal.signal(signal.SIGTERM, signal.SIG_IGN if mode == "stubborn" else down)
    print("fm-sim-agent: serving fm/sim/local/* via tcp/router:7447", flush=True)
    while True:
        time.sleep(0.05)
    """
)


@pytest.fixture
def docker(tmp_path, monkeypatch):
    """A stand-in ``docker`` first on PATH; returns a setter for its scenario and its call log."""
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    script = bin_dir / "docker"
    script.write_text(f"#!{sys.executable}\n{FAKE_DOCKER}")
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    log = tmp_path / "docker.log"
    monkeypatch.setenv("PATH", f"{bin_dir}{os.pathsep}{os.environ['PATH']}")
    monkeypatch.setenv("FAKE_DOCKER_LOG", str(log))
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("FM_ROUTER_ENDPOINT", "tcp/router:7447")
    card = tmp_path / "machine.json"
    card.write_text(json.dumps({"schema_version": 1, "name": "nish-mbp"}))
    monkeypatch.setenv("FM_MACHINE_FILE", str(card))
    for name in (service.IMAGE_ENV, service.IMAGE_DIGEST_ENV):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(service, "DEFAULT_IMAGE_DIGEST", None)  # each test names its pin

    class Fake:
        bin = bin_dir

        def scenario(self, **values) -> None:
            monkeypatch.setenv("FAKE_DOCKER", json.dumps(values))

        def calls(self) -> list[list[str]]:
            return [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []

    fake = Fake()
    fake.scenario(digests=[f"{REPO}@{OLD}"])
    return fake


@pytest.fixture
def agent(tmp_path, monkeypatch):
    """Swap the managed agent's script for the stand-in; kill whatever it left at the end."""
    (tmp_path / "agent-bin").mkdir()
    script = tmp_path / "agent-bin" / "fm-sim-agent"  # the entry point's form: <python> …/fm-sim-agent
    script.write_text(FAKE_AGENT)
    real = this_machine.agent_command
    monkeypatch.setattr(
        this_machine,
        "agent_command",
        lambda image, host: [sys.executable, str(script), *real(image, host)[3:]],
    )
    monkeypatch.setattr(this_machine, "STOP_GRACE_S", 1.0)
    spawned: list[int] = []
    yield spawned
    record = this_machine.this_machine(REF, None, MACHINE)._read_pidfile()
    for pid in [*spawned, *([record.pid] if record else [])]:
        try:
            os.kill(pid, signal.SIGKILL)
            os.waitpid(pid, 0)
        except (ProcessLookupError, ChildProcessError):
            pass


def run_json(argv: list[str], capsys) -> tuple[int, list[dict]]:
    code = main(["host", *argv, "local", "--json"])
    return code, [json.loads(line) for line in capsys.readouterr().out.splitlines()]


def state_dir():
    return this_machine.default_state_dir()


def assert_local_status(status: dict) -> None:
    """The LocalStatus shape of contracts §8, key for key."""
    assert set(status) == {"schema_version", "runtime", "image", "agent"}
    assert status["schema_version"] == 1
    assert set(status["runtime"]) == {"available", "name", "message"}
    assert status["runtime"]["name"] in ("orbstack", "docker", None)
    assert set(status["image"]) == {"ref", "pinned_digest", "present", "local_digest", "update_available"}
    assert set(status["agent"]) == {"running", "pid", "since", "host", "log_path"}
    assert status["agent"]["host"] == MACHINE
    for key in ("available",):
        assert isinstance(status["runtime"][key], bool)
    for key in ("present", "update_available"):
        assert isinstance(status["image"][key], bool)


# -- flags and the pin --------------------------------------------------------


@pytest.mark.parametrize(
    "argv",
    [
        ["host", "pull", "aws", "--stack", "s"],
        ["host", "status", "local", "--stack", "s"],
        ["host", "start", "local", "--region", "eu-west-1"],
        ["host", "status", "mars"],
    ],
)
def test_bad_local_host_flags_are_usage_errors(argv):
    with pytest.raises(SystemExit) as caught:
        parse_args(argv)
    assert caught.value.code == 2


@pytest.mark.parametrize(
    ("env", "value"),
    [
        (service.IMAGE_ENV, "--privileged"),
        (service.IMAGE_ENV, f"{REPO}@{PIN}"),
        (service.IMAGE_DIGEST_ENV, "sha256:short"),
        (service.IMAGE_DIGEST_ENV, "latest"),
    ],
)
def test_a_bad_image_or_pin_is_a_usage_error(env, value, monkeypatch):
    monkeypatch.setenv(env, value)
    with pytest.raises(SystemExit) as caught:
        parse_args(["host", "status", "local"])
    assert caught.value.code == 2


def test_the_pin_belongs_to_the_default_image_unless_the_environment_names_one(monkeypatch):
    monkeypatch.delenv(service.IMAGE_DIGEST_ENV, raising=False)
    monkeypatch.delenv(service.IMAGE_ENV, raising=False)
    monkeypatch.setattr(service, "DEFAULT_IMAGE_DIGEST", PIN)
    args = parse_args(["host", "status", "local"])
    assert (args.image, args.pin) == (REF, PIN)
    monkeypatch.setenv(service.IMAGE_ENV, "fm-sim:jazzy")
    assert parse_args(["host", "status", "local"]).pin is None
    monkeypatch.setenv(service.IMAGE_DIGEST_ENV, OLD)
    assert parse_args(["host", "status", "local"]).pin == OLD


def test_the_shipped_pin_is_a_digest_of_the_default_image(monkeypatch):
    monkeypatch.delenv(service.IMAGE_ENV, raising=False)
    monkeypatch.delenv(service.IMAGE_DIGEST_ENV, raising=False)
    assert service.image_ref() == service.DEFAULT_IMAGE
    assert service.image_pin(service.DEFAULT_IMAGE) == service.DEFAULT_IMAGE_DIGEST
    assert service.DEFAULT_IMAGE_DIGEST is None or service.DIGEST_PATTERN.fullmatch(
        service.DEFAULT_IMAGE_DIGEST
    )


def test_the_managed_agent_is_the_service_as_host_local_off_the_host_network():
    argv = this_machine.agent_command(REF, MACHINE)
    assert argv[:3] == [sys.executable, "-m", "fm_sim_agent.service"]
    args = service.build_parser().parse_args(argv[3:])
    assert (args.host, args.label, args.no_host_network, args.image) == (MACHINE, "local", True, REF)
    assert args.dds_range == "LOCALHOST"
    assert this_machine.is_agent_for(" ".join(argv), MACHINE)


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        ("/x/python -m fm_sim_agent.service --host nish_mbp --label local", True),
        ("/x/python -m fm_sim_agent.service --host nish-mbp", True),
        ("/x/python -m fm_sim_agent.service --host local --no-host-network", True),
        ("/x/.venv/bin/fm-sim-agent --host=local", True),
        ("/x/.venv/bin/fm-sim-agent", True),  # serves the card's name: this machine
        ("/x/.venv/bin/fm-sim-agent --host fm_ws_01", False),
        ("/x/python -m fm_sim_agent.client host start local", False),
        ("sleep 30", False),
        ("grep fm-sim-agent", False),
        ("grep -m fm_sim_agent.service --host local", False),
        ("/usr/bin/python3 /x/.venv/bin/fm-sim-agent --host nish_mbp", True),
    ],
)
def test_is_agent_for(command, expected):
    assert this_machine.is_agent_for(command, MACHINE) is expected


# -- status -------------------------------------------------------------------


def test_status_json_is_a_local_status(docker, capsys, monkeypatch):
    monkeypatch.setenv(service.IMAGE_DIGEST_ENV, PIN)
    code, lines = run_json(["status"], capsys)
    assert code == 0 and len(lines) == 1
    reply = lines[0]
    assert set(reply) == {"ok", "status"} and reply["ok"] is True
    status = reply["status"]
    assert_local_status(status)
    assert status["runtime"] == {
        "available": True,
        "name": "orbstack",
        "message": "OrbStack (context orbstack)",
    }
    assert status["image"] == {
        "ref": REF,
        "pinned_digest": PIN,
        "present": True,
        "local_digest": OLD,
        "update_available": True,
    }
    assert status["agent"] == {
        "running": False,
        "pid": None,
        "since": None,
        "host": MACHINE,
        "log_path": str(state_dir() / "agent.log"),
    }
    assert not state_dir().exists()  # status only reads


def test_status_at_the_pin_has_no_update(docker, capsys, monkeypatch):
    monkeypatch.setenv(service.IMAGE_DIGEST_ENV, PIN)
    docker.scenario(digests=[f"other/repo@{OLD}", f"{REPO}@{OLD}", f"{REPO}@{PIN}"])
    _, [reply] = run_json(["status"], capsys)
    assert reply["status"]["image"]["local_digest"] == PIN
    assert reply["status"]["image"]["update_available"] is False


def test_status_without_a_pin_never_offers_an_update(docker, capsys):
    _, [reply] = run_json(["status"], capsys)
    image = reply["status"]["image"]
    assert image["pinned_digest"] is None and image["local_digest"] == OLD
    assert image["update_available"] is False


def test_a_locally_built_image_has_no_registry_digest(docker, capsys):
    docker.scenario(digests=[])
    _, [reply] = run_json(["status"], capsys)
    assert reply["status"]["image"]["present"] is True
    assert reply["status"]["image"]["local_digest"] is None


def test_docker_desktop_is_named_docker(docker, capsys):
    docker.scenario(context="desktop-linux", os="Docker Desktop", digests=None)
    _, [reply] = run_json(["status"], capsys)
    assert reply["status"]["runtime"] == {
        "available": True,
        "name": "docker",
        "message": "Docker Desktop (context desktop-linux)",
    }
    assert reply["status"]["image"]["present"] is False


def test_a_stopped_runtime_is_unavailable_and_its_text_is_cleaned(docker, capsys):
    docker.scenario(info_error="Cannot connect to the Docker daemon\x1b[2J‮ at unix:///x.sock")
    code, [reply] = run_json(["status"], capsys)
    assert code == 0
    runtime = reply["status"]["runtime"]
    assert runtime["available"] is False and runtime["name"] == "orbstack"
    assert runtime["message"] == (
        "orbstack is not running (context orbstack): Cannot connect to the Docker daemon[2J at unix:///x.sock"
    )
    assert reply["status"]["image"]["present"] is False
    assert ["image", "inspect"] not in [call[:2] for call in docker.calls()]


def test_no_docker_cli_names_no_runtime(docker, capsys, monkeypatch):
    monkeypatch.setenv("PATH", "/nonexistent")
    _, [reply] = run_json(["status"], capsys)
    assert reply["status"]["runtime"] == {
        "available": False,
        "name": None,
        "message": "docker is not on PATH; install OrbStack",
    }


def test_status_lines(docker, capsys, monkeypatch):
    monkeypatch.setenv(service.IMAGE_DIGEST_ENV, PIN)
    assert main(["host", "status", "local"]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out[0] == "runtime: orbstack, available (OrbStack (context orbstack))"
    assert out[1] == f"image: {REF} present, {OLD}, pin {PIN}, update available"
    assert out[2].startswith("agent: not running, log ")


# -- pull ---------------------------------------------------------------------

PULL_LINES = [
    f"{PIN}: Pulling from first-motive/fm-sim",
    "4f4fb700ef54: Pulling fs layer",
    "4f4fb700ef54: Downloading  12.5MB/45MB",
    "",
    "4f4fb700ef54: Pull complete",
    f"Digest: {PIN}",
    f"Status: Downloaded newer image for {REPO}@{PIN}",
]


def test_progress_events():
    assert this_machine.progress_event("  \n") is None
    assert this_machine.progress_event("4f4fb700ef54: Downloading  12.5MB/45MB\n") == {
        "event": "progress",
        "layer": "4f4fb700ef54",
        "current": 12_500_000,
        "total": 45_000_000,
        "message": "Downloading  12.5MB/45MB",
    }
    assert this_machine.progress_event("abc: Waiting") == {"event": "progress", "message": "abc: Waiting"}
    assert this_machine.progress_event("Downloading 512B/1.5kB") == {
        "event": "progress",
        "current": 512,
        "total": 1500,
        "message": "Downloading 512B/1.5kB",
    }


def test_pull_takes_the_pin_then_tags_it_as_the_ref(docker, capsys, monkeypatch):
    monkeypatch.setenv(service.IMAGE_DIGEST_ENV, PIN)
    docker.scenario(digests=None, pull_lines=PULL_LINES, pulled_digests=[f"{REPO}@{PIN}"])
    code, lines = run_json(["pull"], capsys)
    assert code == 0
    *progress, final = lines
    assert all(set(e) <= {"event", "layer", "current", "total", "message"} for e in progress)
    assert all(e["event"] == "progress" for e in progress)
    assert len(progress) == len(PULL_LINES) - 1  # the blank line is no event
    assert progress[2] == {
        "event": "progress",
        "layer": "4f4fb700ef54",
        "current": 12_500_000,
        "total": 45_000_000,
        "message": "Downloading  12.5MB/45MB",
    }
    assert set(final) == {"ok", "status"} and final["ok"] is True
    assert_local_status(final["status"])
    assert final["status"]["image"]["present"] is True
    assert final["status"]["image"]["local_digest"] == PIN
    assert final["status"]["image"]["update_available"] is False
    calls = docker.calls()
    assert ["pull", f"{REPO}@{PIN}"] in calls
    assert ["tag", f"{REPO}@{PIN}", REF] in calls


def test_pull_without_a_pin_pulls_the_tag(docker, capsys):
    docker.scenario(digests=None, pull_lines=["jazzy: Pulling from x"], pulled_digests=[f"{REPO}@{OLD}"])
    code, lines = run_json(["pull"], capsys)
    assert code == 0 and lines[-1]["ok"] is True
    calls = docker.calls()
    assert ["pull", REF] in calls
    assert not [c for c in calls if c[0] == "tag"]


def test_a_failed_pull_is_refused_with_its_last_line(docker, capsys):
    docker.scenario(digests=None, pull_lines=["Error response from daemon: denied\x07"], pull_rc=1)
    code, lines = run_json(["pull"], capsys)
    assert code == 1
    final = lines[-1]
    assert set(final) == {"ok", "error", "status"}
    assert final["error"]["code"] == "pull_failed"
    assert final["error"]["message"].startswith("Error response from daemon: denied; ")
    assert final["error"]["message"].endswith("run: docker login ghcr.io")
    assert_local_status(final["status"])


def test_a_pull_that_fails_for_another_reason_gets_no_login_hint():
    message = this_machine.pull_failure("no space left on device", "ghcr.io/first-motive/fm-sim:jazzy")
    assert message == "no space left on device"


def test_a_pull_that_misses_the_pin_is_a_digest_mismatch(docker, capsys, monkeypatch):
    monkeypatch.setenv(service.IMAGE_DIGEST_ENV, PIN)
    docker.scenario(digests=None, pulled_digests=[f"{REPO}@{OLD}"])
    code, lines = run_json(["pull"], capsys)
    assert code == 1 and lines[-1]["error"]["code"] == "digest_mismatch"


def test_pull_needs_a_runtime(docker, capsys):
    docker.scenario(info_error="no daemon")
    code, [reply] = run_json(["pull"], capsys)
    assert code == 1 and reply["error"]["code"] == "runtime_unavailable"
    assert not [c for c in docker.calls() if c[0] == "pull"]


def test_pull_lines_without_json(docker, capsys):
    docker.scenario(digests=None, pull_lines=["4f4fb700ef54: Pull complete"], pulled_digests=[])
    assert main(["host", "pull", "local"]) == 0
    assert capsys.readouterr().out.splitlines()[0] == "4f4fb700ef54: Pull complete"


# -- start and stop -----------------------------------------------------------


def test_start_runs_the_agent_detached_and_status_finds_it(docker, agent, capsys):
    code, [reply] = run_json(["start"], capsys)
    assert code == 0 and reply["ok"] is True
    assert_local_status(reply["status"])
    running = reply["status"]["agent"]
    assert running["running"] is True and isinstance(running["pid"], int) and running["since"]
    assert stat.S_IMODE(os.stat(state_dir()).st_mode) == 0o700
    record = json.loads((state_dir() / "agent.pid").read_text())
    assert record["pid"] == running["pid"] and record["argv"][2:6] == ["--host", MACHINE, "--label", "local"]
    assert stat.S_IMODE(os.stat(state_dir() / "agent.pid").st_mode) == 0o600
    assert os.getsid(running["pid"]) == running["pid"]  # a session of its own
    assert "serving fm/sim/local/*" in (state_dir() / "agent.log").read_text()

    _, [again] = run_json(["start"], capsys)
    assert again["status"]["agent"]["pid"] == running["pid"]  # a no-op
    _, [status] = run_json(["status"], capsys)
    assert status["status"]["agent"] == running


def test_stop_terms_the_agent_which_downs_its_run(docker, agent, capsys):
    _, [started] = run_json(["start"], capsys)
    pid = started["status"]["agent"]["pid"]
    code, [reply] = run_json(["stop"], capsys)
    assert code == 0 and reply["ok"] is True
    assert reply["status"]["agent"] == {
        **started["status"]["agent"],
        "running": False,
        "pid": None,
        "since": None,
    }
    assert "fake-agent: down on stop" in (state_dir() / "agent.log").read_text()
    assert not (state_dir() / "agent.pid").exists()
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)
    assert not [c for c in docker.calls() if c[:2] == ["rm", "--force"]]


def test_an_agent_that_ignores_sigterm_is_killed_and_its_containers_cleared(
    docker, agent, capsys, monkeypatch
):
    monkeypatch.setenv("FAKE_AGENT_MODE", "stubborn")
    _, [started] = run_json(["start"], capsys)
    docker.scenario(digests=[f"{REPO}@{OLD}"], ps=[RUN_ID])  # the agent's run, up when stop begins
    code, [reply] = run_json(["stop"], capsys)
    assert code == 0 and reply["status"]["agent"]["running"] is False
    calls = docker.calls()
    assert ["rm", "--force", f"fm-sim-run-{RUN_ID}"] in calls  # only that run, by name
    assert not [c for c in calls if c[:2] == ["ps", "--all"]]  # never everything under the label
    with pytest.raises(ProcessLookupError):
        os.kill(started["status"]["agent"]["pid"], 0)


def test_stop_with_nothing_running_is_a_no_op(docker, agent, capsys):
    code, [reply] = run_json(["stop"], capsys)
    assert code == 0 and reply["ok"] is True and reply["status"]["agent"]["running"] is False


def test_an_agent_that_exits_at_once_is_start_failed(docker, agent, capsys, monkeypatch):
    monkeypatch.setenv("FAKE_AGENT_MODE", "exit")
    code, [reply] = run_json(["start"], capsys)
    assert code == 1
    assert reply["error"]["code"] == "start_failed"
    assert "exited with 1: fm-sim-agent: FM_ROUTER_ENDPOINT is set neither" in reply["error"]["message"]
    assert reply["status"]["agent"]["running"] is False
    assert not (state_dir() / "agent.pid").exists()


def write_pidfile(pid: int, argv: list[str]) -> None:
    state_dir().mkdir(parents=True, exist_ok=True)
    record = {"pid": pid, "since": "2026-10-17T10:00:00Z", "argv": argv, "host": MACHINE}
    (state_dir() / "agent.pid").write_text(json.dumps(record))


def test_a_stale_pidfile_reads_as_not_running_and_stop_clears_it(docker, agent, capsys):
    dead = subprocess.Popen([sys.executable, "-c", "pass"])
    dead.wait()
    write_pidfile(dead.pid, this_machine.agent_command(REF, MACHINE))
    _, [reply] = run_json(["status"], capsys)
    assert reply["status"]["agent"]["running"] is False
    run_json(["stop"], capsys)
    assert not (state_dir() / "agent.pid").exists()


def test_a_reused_pid_is_not_the_agent_and_is_never_signalled(docker, agent, capsys):
    stranger = subprocess.Popen(["sleep", "30"])
    agent.append(stranger.pid)
    write_pidfile(stranger.pid, this_machine.agent_command(REF, MACHINE))
    _, [reply] = run_json(["status"], capsys)
    assert reply["status"]["agent"] == {
        "running": False,
        "pid": None,
        "since": None,
        "host": MACHINE,
        "log_path": str(state_dir() / "agent.log"),
    }
    run_json(["stop"], capsys)
    assert stranger.poll() is None  # still alive: stop never touched it


def test_a_garbled_pidfile_reads_as_not_running(docker, agent, capsys):
    state_dir().mkdir(parents=True)
    (state_dir() / "agent.pid").write_text('{"pid": true, "argv": []}')
    _, [reply] = run_json(["status"], capsys)
    assert reply["status"]["agent"]["running"] is False


@pytest.mark.parametrize(
    ("scenario", "code"),
    [
        ({"info_error": "no daemon"}, "runtime_unavailable"),
        ({"digests": None}, "image_missing"),
        ({"digests": [], "ps": [RUN_ID]}, "already_running_elsewhere"),
    ],
)
def test_start_refusals(scenario, code, docker, agent, capsys):
    docker.scenario(**scenario)
    exit_code, [reply] = run_json(["start"], capsys)
    assert exit_code == 1
    assert set(reply) == {"ok", "error", "status"} and reply["ok"] is False
    assert reply["error"]["code"] == code
    assert_local_status(reply["status"])
    assert reply["status"]["agent"]["running"] is False
    assert not (state_dir() / "agent.pid").exists()


def test_start_refuses_beside_an_agent_it_did_not_start(docker, agent, tmp_path, capsys):
    other = subprocess.Popen(this_machine.agent_command(REF, MACHINE), stdout=subprocess.DEVNULL)
    agent.append(other.pid)
    deadline = time.monotonic() + 5
    while not this_machine.LocalHost(REF, None, tmp_path, MACHINE)._command_of(other.pid):
        assert time.monotonic() < deadline
        time.sleep(0.05)
    exit_code, [reply] = run_json(["start"], capsys)
    assert exit_code == 1
    assert reply["error"]["code"] == "already_running_elsewhere"
    assert f"pid {other.pid}" in reply["error"]["message"]


def test_start_needs_a_router(docker, agent, capsys, monkeypatch):
    monkeypatch.delenv("FM_ROUTER_ENDPOINT")
    monkeypatch.setenv("FM_COMMS_ENV_FILE", "/nonexistent/fm-comms.env")
    exit_code, [reply] = run_json(["start"], capsys)
    assert exit_code == 1 and reply["error"]["code"] == "router_unavailable"


def test_a_refusal_without_json_goes_to_stderr(docker, agent, capsys):
    docker.scenario(digests=None)
    assert main(["host", "start", "local"]) == 1
    captured = capsys.readouterr()
    assert captured.err.splitlines()[-1].startswith("refused (image_missing): ")
    assert captured.out.splitlines()[1].startswith(f"image: {REF} missing")


def test_a_state_dir_that_is_a_symlink_is_refused(docker, agent, tmp_path, capsys):
    target = tmp_path / "elsewhere"
    target.mkdir()
    state_dir().parent.mkdir(parents=True)
    state_dir().symlink_to(target)
    exit_code, [reply] = run_json(["start"], capsys)
    assert exit_code == 1 and reply["error"]["code"] == "state_unavailable"


# -- this machine's name, and --host local while the agent runs --------------


def test_a_card_that_names_no_key_segment_is_a_usage_error(docker, tmp_path, monkeypatch):
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"schema_version": 1, "name": "fm/*"}))
    monkeypatch.setenv("FM_MACHINE_FILE", str(bad))
    with pytest.raises(SystemExit) as caught:
        parse_args(["host", "status", "local"])
    assert caught.value.code == 2


def test_with_no_card_the_agent_serves_as_the_hostname(docker, tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("FM_MACHINE_FILE", str(tmp_path / "absent.json"))
    monkeypatch.setattr(card_module.socket, "gethostname", lambda: "Studio-2.local")
    _, [reply] = run_json(["status"], capsys)
    assert reply["status"]["agent"]["host"] == "studio_2"


def test_host_local_reaches_the_managed_agent_by_this_machines_name(docker, agent, capsys, monkeypatch):
    run_json(["start"], capsys)
    assert this_machine.managed_host() == MACHINE
    reached = []
    monkeypatch.setattr(client, "remote_host", lambda host: reached.append(host) or object())
    assert isinstance(client.connect(parse_args(["status", "--host", "local"])), object)
    assert reached == [MACHINE]
    with pytest.raises(client.UsageError):
        client.connect(parse_args(["status", "--host", "local", "--image", "fm-sim:jazzy"]))
    run_json(["stop"], capsys)
    assert this_machine.managed_host() is None
    assert isinstance(client.connect(parse_args(["status", "--host", "local"])), client.Local)


START_AS_A_CHILD = textwrap.dedent(
    """\
    import sys
    from fm_sim_agent import client, this_machine
    real = this_machine.agent_command
    this_machine.agent_command = lambda image, host: [sys.executable, sys.argv[1], *real(image, host)[3:]]
    sys.exit(client.main(["host", "start", "local", "--json"]))
    """
)


def test_the_agent_outlives_a_runner_that_kills_starts_process_group(docker, agent, tmp_path, capsys):
    """The app's runner SIGKILLs the command's whole process group once it exits."""
    child = subprocess.Popen(
        [sys.executable, "-c", START_AS_A_CHILD, str(tmp_path / "agent-bin" / "fm-sim-agent")],
        stdout=subprocess.PIPE,
        start_new_session=True,  # a group of its own, as the app's runner gives it
    )
    out, _ = child.communicate(timeout=30)
    assert child.returncode == 0
    started = json.loads(out.decode().splitlines()[-1])["status"]["agent"]
    agent.append(started["pid"])
    try:
        os.killpg(child.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass  # nothing of start's group is left: the agent is in a group of its own
    time.sleep(0.2)
    assert os.getpgid(started["pid"]) == started["pid"] != child.pid
    _, [reply] = run_json(["status"], capsys)
    assert reply["status"]["agent"]["running"] is True
    assert reply["status"]["agent"]["pid"] == started["pid"]


def test_a_pidfile_whose_host_is_not_a_key_segment_reads_as_not_running(docker, agent, capsys):
    _, [started] = run_json(["start"], capsys)
    record = json.loads((state_dir() / "agent.pid").read_text())
    (state_dir() / "agent.pid").write_text(json.dumps({**record, "host": "fm/*"}))
    _, [reply] = run_json(["status"], capsys)
    assert reply["status"]["agent"]["running"] is False and reply["status"]["agent"]["host"] == MACHINE
    (state_dir() / "agent.pid").write_text(json.dumps(record))  # so the fixture ends it


def test_a_slow_stop_never_stalls_a_start(docker, agent, capsys, monkeypatch):
    monkeypatch.setenv("FAKE_AGENT_MODE", "stubborn")
    monkeypatch.setattr(this_machine, "STOP_GRACE_S", 3.0)
    _, [started] = run_json(["start"], capsys)
    host = this_machine.this_machine(REF, None, MACHINE)
    stopping = threading.Thread(target=host.stop)
    stopping.start()
    time.sleep(0.5)  # stop has sent SIGTERM and is in its grace
    began = time.monotonic()
    status = host.start()  # the agent is still up: a no-op
    assert time.monotonic() - began < 1.5
    assert status["agent"]["pid"] == started["status"]["agent"]["pid"]
    stopping.join(timeout=15)
    assert host.agent_state()["running"] is False


def test_start_moves_a_big_log_aside(docker, agent, capsys, monkeypatch):
    monkeypatch.setattr(this_machine, "MAX_LOG_BYTES", 10)
    state_dir().mkdir(parents=True)
    (state_dir() / "agent.log").write_text("an old agent's long log\n")
    run_json(["start"], capsys)
    assert (state_dir() / "agent.log.1").read_text() == "an old agent's long log\n"
    assert "an old agent" not in (state_dir() / "agent.log").read_text()


def test_an_unwritable_pidfile_ends_the_new_agent(docker, agent, capsys, monkeypatch):
    spawned = []

    def fail(self, pid, argv):
        spawned.append(pid)
        raise OSError("disk full")

    monkeypatch.setattr(this_machine.LocalHost, "_write_pidfile", fail)
    code, [reply] = run_json(["start"], capsys)
    assert code == 1 and reply["error"]["code"] == "start_failed" and "disk full" in reply["error"]["message"]
    with pytest.raises(ProcessLookupError):
        os.kill(spawned[0], 0)


def test_an_interrupt_still_ends_with_a_reply_line(docker, capsys, monkeypatch):
    def interrupted(self, on_progress):
        on_progress({"event": "progress", "message": "4f4fb700ef54: Downloading"})
        raise KeyboardInterrupt

    monkeypatch.setattr(this_machine.LocalHost, "pull", interrupted)
    code, lines = run_json(["pull"], capsys)
    assert code == 130
    assert lines[-1]["ok"] is False and lines[-1]["error"]["code"] == "interrupted"
    assert_local_status(lines[-1]["status"])


def test_an_unpinned_image_is_warned_on_stderr_only(docker, capsys, monkeypatch):
    assert main(["host", "status", "local", "--json"]) == 0
    captured = capsys.readouterr()
    assert "is not pinned to a digest" in captured.err
    json.loads(captured.out)  # stdout is still exactly the reply
    monkeypatch.setenv(service.IMAGE_DIGEST_ENV, PIN)
    main(["host", "status", "local", "--json"])
    assert "not pinned" not in capsys.readouterr().err


def test_a_bad_image_is_refused_by_the_in_process_verbs_and_the_service(monkeypatch, capsys):
    with pytest.raises(SystemExit) as caught:
        parse_args(["status", "--host", "local", "--image", "-x"])
    assert caught.value.code == 2
    monkeypatch.setenv(service.IMAGE_ENV, "--privileged")
    monkeypatch.setenv("FM_ROUTER_ENDPOINT", "tcp/router:7447")
    assert service.main(["--host", "local"]) == 1
    assert "is not an image reference" in capsys.readouterr().err
