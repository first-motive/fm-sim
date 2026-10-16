"""DockerRunner's command lines and parsers, run against a stand-in ``docker``.

The stand-in is a real executable on disk that records its argv and prints what
Docker would. Every RunnerError path and every argv the agent sends goes through
the same ``subprocess`` call the agent makes on a host.
"""

from __future__ import annotations

import io
import json
import stat
import sys
import textwrap

import pytest

from fm_sim_agent.logline import MAX_LINE_BYTES
from fm_sim_agent.runner import (
    CATALOG,
    ENTRYPOINT,
    LAUNCH,
    RECORD_SCRIPT,
    DockerRunner,
    RunnerError,
    capped_lines,
    launch_args,
    parse_outcome,
    parse_reset,
)
from tests.fakes import CATALOG as CATALOG_DOC
from tests.fakes import SPEC

RUN_ID = "20261016T120000-mujoco-openarm-empty-idle-7"

FAKE_DOCKER = textwrap.dedent(
    """\
    import json, os, sys
    args = sys.argv[1:]
    with open(os.environ["FAKE_DOCKER_LOG"], "a") as log:
        log.write(json.dumps(args) + "\\n")
    if os.environ.get("FAKE_DOCKER_FAIL") == args[0]:
        print("Error response from daemon: boom", file=sys.stderr)
        sys.exit(1)
    if args[:2] == ["image", "inspect"]:
        print("sha256:abc")
    elif args[:2] == ["run", "--rm"]:
        print(os.environ["FAKE_CATALOG"])
    elif args[0] == "exec" and "list_controllers" in " ".join(args):
        print("response:\\ncontroller_manager_msgs.srv.ListControllers_Response(controller=[controller_manager_msgs.msg.ControllerState(name='joint_state_broadcaster', state='active', type='x'), controller_manager_msgs.msg.ControllerState(name='openarm_right_arm_controller', state='inactive', type='y')])")
    elif args[0] == "exec" and args[-2:] == ["service", "list"]:
        print("/controller_manager/list_controllers\\n/sim/reset")
    elif args[0] == "exec" and "service" in args:
        print("requester: making request: fm_sim_msgs.srv.Reset_Request(seed=7)\\n")
        print("response:\\n" + os.environ.get("FAKE_RESET", "fm_sim_msgs.srv.Reset_Response(ok=True, message='seeded')"))
    elif args[0] == "exec" and "echo" in args:
        print("header:\\n  stamp:\\n    sec: 1\\nscenario: empty\\nseed: 7\\nsuccess: false\\nreason: 'dropped: cube'\\nelapsed_s: 3\\n---")
    elif args[0] == "logs":
        print("[INFO] [launch]: All log files can be found below /tmp/.ros/log")
        print("[sim_loop-1] [INFO] [1.5] [sim_loop]: up")
    elif args[0] == "ps":
        print("c1\\nc2")
    """
)


@pytest.fixture
def docker(tmp_path, monkeypatch):
    script = tmp_path / "docker"
    script.write_text(f"#!{sys.executable}\n{FAKE_DOCKER}")
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    log = tmp_path / "docker.log"
    monkeypatch.setenv("FAKE_DOCKER_LOG", str(log))
    monkeypatch.setenv("FAKE_CATALOG", json.dumps(CATALOG_DOC))
    for name in (
        "ROS_DOMAIN_ID",
        "RMW_IMPLEMENTATION",
        "ROS_LOCALHOST_ONLY",
        "FM_COMMS_PROFILE",
        "FM_CYCLONEDDS_XML",
    ):
        monkeypatch.delenv(name, raising=False)

    def calls() -> list[list[str]]:
        return [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []

    runner = DockerRunner("fm-sim:jazzy", tmp_path / "runs", "fm_ws_01", docker=str(script))
    runner.calls = calls
    return runner


def test_start_runs_the_contract_launch_as_argv(docker):
    docker.start(RUN_ID, SPEC)
    (argv,) = docker.calls()
    assert argv[:2] == ["run", "--detach"]
    assert argv[argv.index("--name") + 1] == f"fm-sim-run-{RUN_ID}"
    assert "fm.sim.agent=fm_ws_01" in argv and f"fm.sim.run_id={RUN_ID}" in argv
    tail = argv[argv.index("fm-sim:jazzy") :]
    assert tail == [
        "fm-sim:jazzy", ENTRYPOINT, *LAUNCH,
        "backend:=mujoco", "robot:=openarm", "scenario:=empty", "controller:=idle", "use_sim:=true",
    ]  # fmt: skip
    assert ["--network", "host"] == argv[argv.index("--network") : argv.index("--network") + 2]
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in argv
    # Only this run's directory is mounted, never the whole runs dir.
    # Only this run's bag directory is mounted: not the runs dir, not its summary.
    assert f"{docker.runs_dir / RUN_ID / 'bag'}:/fm-run/bag" in argv
    assert not any(a.startswith((f"{docker.runs_dir}:", f"{docker.runs_dir / RUN_ID}:")) for a in argv)
    assert (docker.runs_dir / RUN_ID / "bag").is_dir()
    # A missing image fails fast instead of pulling under the start timeout.
    assert "--pull=never" in argv[: argv.index("fm-sim:jazzy")]


def test_every_container_is_confined(docker):
    docker.catalog()
    catalog_argv = docker.calls()[-1]
    for argv in (docker.run_argv(RUN_ID, SPEC), catalog_argv):
        image = argv.index("fm-sim:jazzy")
        assert argv[argv.index("--cap-drop") + 1] == "ALL" and argv.index("--cap-drop") < image
        assert argv[argv.index("--security-opt") + 1] == "no-new-privileges"
        assert argv[argv.index("--user") + 1] != "0:0"
        assert argv[argv.index("--pids-limit") + 1] == "4096"
        assert argv[argv.index("--memory") + 1] == "16g"


def test_limits_are_the_operators(docker):
    docker.pids_limit, docker.memory = 512, "64g"
    argv = docker.run_argv(RUN_ID, SPEC)
    assert argv[argv.index("--pids-limit") + 1] == "512" and argv[argv.index("--memory") + 1] == "64g"


def test_host_network_is_the_operators_choice(docker):
    docker.host_network = False
    assert "--network" not in docker.run_argv(RUN_ID, SPEC)


def test_transport_env_is_passed_by_name_not_value(docker, monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    xml = tmp_path / "cyclone.xml"
    monkeypatch.setenv("FM_CYCLONEDDS_XML", str(xml))
    argv = docker.run_argv(RUN_ID, SPEC)
    assert "ROS_DOMAIN_ID" in argv and "ROS_DOMAIN_ID=42" not in argv
    assert "RMW_IMPLEMENTATION" in argv and "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" not in argv
    assert f"{xml}:/etc/fm-comms/cyclonedds.xml:ro" in argv


def test_gpus_and_extra_args_land_before_the_image(docker):
    docker.gpus = True
    docker.docker_args = ("--shm-size", "1g")
    argv = docker.run_argv(RUN_ID, SPEC)
    image = argv.index("fm-sim:jazzy")
    assert argv.index("--gpus") < image and argv.index("--shm-size") < image


def test_catalog_runs_the_catalog_command_in_the_image(docker):
    assert docker.catalog() == CATALOG_DOC
    inspect, argv = docker.calls()
    assert inspect[:2] == ["image", "inspect"]
    assert argv[:3] == ["run", "--rm", "--pull=never"]
    assert argv[argv.index("--name") + 1] == "fm-sim-catalog-abc"
    assert "fm.sim.agent=fm_ws_01" in argv
    assert argv[argv.index("fm-sim:jazzy") :] == ["fm-sim:jazzy", ENTRYPOINT, *CATALOG]


def test_image_id_reads_the_local_image(docker):
    assert docker.image_id() == "sha256:abc"


def test_a_docker_failure_is_a_runner_error_with_its_reason(docker, monkeypatch):
    monkeypatch.setenv("FAKE_DOCKER_FAIL", "run")
    with pytest.raises(RunnerError, match="boom"):
        docker.start(RUN_ID, SPEC)


def test_a_missing_docker_is_a_runner_error(docker):
    docker.docker = "/nonexistent/docker"
    with pytest.raises(RunnerError, match="cannot run docker"):
        docker.image_id()


def test_reset_calls_the_service_with_the_seed(docker):
    assert docker.reset(RUN_ID, 7, 5.0) == "seeded"
    (argv,) = docker.calls()
    assert argv[:2] == ["exec", f"fm-sim-run-{RUN_ID}"]
    assert argv[-3:] == ["/sim/reset", "fm_sim_msgs/srv/Reset", '{"seed": 7}']


def test_a_refused_reset_raises(docker, monkeypatch):
    monkeypatch.setenv("FAKE_RESET", "fm_sim_msgs.srv.Reset_Response(ok=False, message='no scenario')")
    with pytest.raises(RunnerError, match="no scenario"):
        docker.reset(RUN_ID, 7, 5.0)


def test_outcome_reads_the_latched_message(docker):
    assert docker.outcome(RUN_ID) == {"success": False, "reason": "dropped: cube", "elapsed_s": 3.0}


def test_recording_writes_mcap_into_the_runs_mount(docker):
    docker.start_recording(RUN_ID)
    (argv,) = docker.calls()
    assert argv[:3] == ["exec", "--detach", f"fm-sim-run-{RUN_ID}"]
    assert argv[3:6] == ["bash", "-c", RECORD_SCRIPT]
    assert argv[-10:] == [
        "record", "-s", "mcap", "-a",
        "--max-bag-duration", "600", "--max-bag-size", str(4 * 2**30),
        "-o", "/fm-run/bag/recording",
    ]  # fmt: skip


def test_a_catalog_that_times_out_is_removed(docker, monkeypatch):
    monkeypatch.setenv("FAKE_DOCKER_FAIL", "run")
    with pytest.raises(RunnerError):
        docker.catalog()
    assert docker.calls()[-1] == ["rm", "--force", "fm-sim-catalog-abc"]


def test_stop_stops_then_removes(docker):
    docker.stop(RUN_ID)
    assert [argv[0] for argv in docker.calls()] == ["stop", "rm"]


def test_logs_stream_the_container_output(docker):
    lines = list(docker.logs(RUN_ID))
    assert lines[1] == "[sim_loop-1] [INFO] [1.5] [sim_loop]: up\n"


def test_stop_stale_removes_only_this_hosts_containers(docker):
    assert docker.stop_stale() == ["c1", "c2"]
    ps, rm = docker.calls()
    assert "label=fm.sim.agent=fm_ws_01" in ps
    assert rm == ["rm", "--force", "c1", "c2"]


def test_launch_args_are_the_four_dimensions_and_use_sim():
    assert launch_args(SPEC) == [
        "backend:=mujoco", "robot:=openarm", "scenario:=empty", "controller:=idle", "use_sim:=true",
    ]  # fmt: skip


def test_list_controllers_reads_the_controller_manager(docker):
    states = docker.list_controllers(RUN_ID)
    assert states == {"joint_state_broadcaster": "active", "openarm_right_arm_controller": "inactive"}
    (argv,) = docker.calls()
    assert argv[-2:] == [
        "/controller_manager/list_controllers",
        "controller_manager_msgs/srv/ListControllers",
    ]


def test_has_service_matches_whole_lines(docker):
    assert docker.has_service(RUN_ID, "/sim/reset") is True
    assert docker.has_service(RUN_ID, "/sim") is False


def test_a_container_name_is_built_only_from_a_run_id(docker):
    with pytest.raises(RunnerError):
        docker.stop("../../x")


def test_an_over_long_log_line_is_cut_and_the_rest_dropped():
    stream = io.StringIO("short\n" + "x" * (3 * MAX_LINE_BYTES) + "\nnext\n")
    lines = list(capped_lines(stream))
    assert lines[0] == "short\n"
    assert lines[1] == "x" * MAX_LINE_BYTES
    assert lines[2] == "next\n"
    assert len(lines) == 3


@pytest.mark.parametrize(
    "text",
    [
        "",
        "not: [yaml",
        "success: maybe\nreason: x\nelapsed_s: 1",
        "success: true\nreason: x\nelapsed_s: true",
    ],
)
def test_an_unusable_outcome_is_none(text):
    assert parse_outcome(text) is None


def test_parse_reset_without_a_response_is_a_refusal():
    assert parse_reset("waiting for service to become available...") == (False, "no response from /sim/reset")


def test_the_runner_never_reaches_a_shell(monkeypatch, docker):
    """Every subprocess call is an argv list with shell left off."""
    import subprocess

    seen = []
    real = subprocess.run

    def spy(*args, **kwargs):
        seen.append((args, kwargs))
        return real(*args, **kwargs)

    monkeypatch.setattr(subprocess, "run", spy)
    docker.start(RUN_ID, SPEC)
    docker.reset(RUN_ID, 7, 5.0)
    assert seen and all(isinstance(a[0], list) and not k.get("shell") for a, k in seen)
