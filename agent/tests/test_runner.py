"""DockerRunner's command lines and parsers, run against a stand-in ``docker``.

The stand-in is a real executable on disk that records its argv and prints what
Docker would. Every RunnerError path and every argv the agent sends goes through
the same ``subprocess`` call the agent makes on a host.
"""

from __future__ import annotations

import io
import json
import os
import stat
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

from fm_sim_agent import bridge, isaac
from fm_sim_agent.logline import MAX_LINE_BYTES
from fm_sim_agent.runner import (
    CATALOG,
    ENTRYPOINT,
    LAUNCH,
    MODELS_MOUNT,
    RECORD_SCRIPT,
    DockerRunner,
    RunnerError,
    capped_lines,
    check_mount_path,
    launch_args,
    mark_isaac_unavailable,
    parse_description,
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
    if os.environ.get("FAKE_DOCKER_FAIL") == args[0] or os.environ.get("FAKE_DOCKER_FAIL_ARG", "-") in args:
        print(os.environ.get("FAKE_DOCKER_FAIL_MESSAGE", "Error response from daemon: boom"), file=sys.stderr)
        sys.exit(1)
    if args[:2] == ["image", "inspect"]:
        print("sha256:abc")
    elif args[:2] == ["run", "--rm"] and "/bin/ls" in args:
        print(os.environ.get("FAKE_IMAGE_ROOT", "bin\\netc\\nws\\nws_ctl"))
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
        import time
        time.sleep(float(os.environ.get("FAKE_LOGS_SLEEP", "0")))
        print("[INFO] [launch]: All log files can be found below /tmp/.ros/log")
        print("[sim_loop-1] [INFO] [1.5] [sim_loop]: up")
    elif args[0] == "ps" and "--format" in args:
        print(os.environ.get("FAKE_PS_RUNS", ""))
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
    monkeypatch.delenv("FAKE_DOCKER_FAIL_ARG", raising=False)
    monkeypatch.delenv("FAKE_DOCKER_FAIL_MESSAGE", raising=False)
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


def test_dds_discovery_is_host_local_by_default(docker):
    docker.catalog()
    for argv in (docker.run_argv(RUN_ID, SPEC), docker.calls()[-1]):
        image = argv.index("fm-sim:jazzy")
        assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in argv[:image]
        assert not any(a.startswith("ROS_LOCALHOST_ONLY") for a in argv)


def test_only_the_operator_flag_changes_the_dds_range(tmp_path):
    runner = DockerRunner("fm-sim:jazzy", tmp_path, "fm_ws_01", dds_range="SUBNET")
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" in runner.run_argv(RUN_ID, SPEC)
    with pytest.raises(ValueError):
        DockerRunner("fm-sim:jazzy", tmp_path, "fm_ws_01", dds_range="EVERYWHERE")


def test_the_host_environment_cannot_widen_the_dds_range(docker, monkeypatch):
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "0")
    argv = docker.run_argv(RUN_ID, SPEC)
    ranges = [a for a in argv if a.startswith("ROS_AUTOMATIC_DISCOVERY_RANGE")]
    assert ranges == ["ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"]
    assert "ROS_LOCALHOST_ONLY" not in argv


def test_the_cli_flag_reaches_the_runner():
    from fm_sim_agent.service import build_parser

    assert build_parser().parse_args([]).dds_range == "LOCALHOST"
    assert build_parser().parse_args(["--dds-range", "SUBNET"]).dds_range == "SUBNET"
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--dds-range", "EVERYWHERE"])


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


def isaac_listed(monkeypatch) -> None:
    doc = json.loads(json.dumps(CATALOG_DOC))
    doc["backends"] = [{**b, "available": True} if b["name"] == "isaac" else b for b in doc["backends"]]
    monkeypatch.setenv("FAKE_CATALOG", json.dumps(doc))


def test_a_host_with_no_isaac_image_lists_isaac_unavailable(docker, monkeypatch):
    isaac_listed(monkeypatch)
    backends = {b["name"]: b for b in docker.catalog()["backends"]}
    assert backends["isaac"]["available"] is False
    assert backends["isaac"]["description"].endswith("no Isaac Sim image to run the stage in")
    assert backends["mujoco"]["available"] is True


def test_a_host_with_an_isaac_image_lists_isaac_as_the_image_does(docker, monkeypatch):
    isaac_listed(monkeypatch)
    docker.isaac_image = "nvcr.io/nvidia/isaac-sim@sha256:" + "0" * 64
    backends = {b["name"]: b for b in docker.catalog()["backends"]}
    assert backends["isaac"]["available"] is True and backends["isaac"]["description"] == "isaac"


@pytest.mark.parametrize(
    "doc",
    [
        [],
        {"backends": "isaac"},
        {"backends": ["isaac"]},
        {"backends": [{"name": "isaac", "available": False}]},
    ],
    ids=["not an object", "backends not a list", "a backend not an object", "isaac already unavailable"],
)
def test_marking_isaac_leaves_any_other_doc_as_it_is(doc):
    before = json.loads(json.dumps(doc))
    assert mark_isaac_unavailable(doc) == before


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
    assert docker.outcome(RUN_ID) == {
        "scenario": "empty", "seed": 7, "success": False, "reason": "dropped: cube", "elapsed_s": 3.0,
    }  # fmt: skip


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
    # The run's bridge and Isaac first, if any, whatever this runner's own config.
    assert docker.calls() == [
        ["rm", "--force", f"fm-sim-bridge-{RUN_ID}"],
        ["stop", "--time", "20", f"fm-sim-isaac-{RUN_ID}"],
        ["rm", "--force", f"fm-sim-isaac-{RUN_ID}"],
        ["stop", "--time", "20", f"fm-sim-run-{RUN_ID}"],
        ["rm", "--force", f"fm-sim-run-{RUN_ID}"],
    ]


def test_stop_tolerates_a_run_with_no_bridge(docker, monkeypatch):
    monkeypatch.setenv("FAKE_DOCKER_FAIL_ARG", f"fm-sim-bridge-{RUN_ID}")
    monkeypatch.setenv("FAKE_DOCKER_FAIL_MESSAGE", "Error: No such container: x")
    docker.stop(RUN_ID)
    assert docker.calls()[-1] == ["rm", "--force", f"fm-sim-run-{RUN_ID}"]


def test_logs_stream_the_container_output(docker):
    lines = list(docker.logs(RUN_ID))
    assert lines[1] == "[sim_loop-1] [INFO] [1.5] [sim_loop]: up\n"


def test_stop_stale_removes_only_this_hosts_containers(docker):
    assert docker.stop_stale() == ["c1", "c2"]
    ps, rm = docker.calls()
    assert "label=fm.sim.agent=fm_ws_01" in ps
    assert rm == ["rm", "--force", "c1", "c2"]


def test_active_runs_reads_run_ids_off_this_hosts_live_run_containers(docker, monkeypatch):
    monkeypatch.setenv("FAKE_PS_RUNS", f"{RUN_ID}\nnot-a-run-id")
    assert docker.active_runs() == [RUN_ID]
    (ps,) = docker.calls()
    # Live only (no --all), this host's label, run containers only (not a catalog fetch).
    assert "--all" not in ps
    assert "label=fm.sim.agent=fm_ws_01" in ps and "label=fm.sim.run_id" in ps
    assert ps[-1] == '{{.Label "fm.sim.run_id"}}'


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
        "scenario: empty\nsuccess: true\nreason: x\nelapsed_s: 1",
        "scenario: empty\nseed: true\nsuccess: true\nreason: x\nelapsed_s: 1",
        "seed: 7\nsuccess: true\nreason: x\nelapsed_s: 1",
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


def test_a_label_other_than_the_host_marks_runs_and_scopes_listing(docker, monkeypatch):
    docker.label = "local"
    argv = docker.run_argv(RUN_ID, SPEC)
    assert argv[argv.index("--label") + 1] == "fm.sim.agent=local"
    monkeypatch.setenv("FAKE_PS_RUNS", RUN_ID)
    docker.active_runs()
    docker.stop_stale()
    live, every, _rm = docker.calls()
    assert "label=fm.sim.agent=local" in live and "label=fm.sim.agent=local" in every


def test_the_label_defaults_to_the_host(tmp_path):
    assert DockerRunner("fm-sim:jazzy", tmp_path, "fm_ws_01").label == "fm_ws_01"


def test_parse_description_is_the_echoed_urdf_without_its_separator():
    assert (
        parse_description('<?xml version="1.0" ?><robot name="r"/>\n---\n')
        == '<?xml version="1.0" ?><robot name="r"/>'
    )


def test_parse_description_with_nothing_echoed_raises():
    with pytest.raises(RunnerError):
        parse_description("")


# -- a run's bridge, off the host network --------------------------------------


@pytest.fixture
def bridged(docker, tmp_path):
    """The runner as a Mac's agent builds it: off the host network, with a bridge config."""
    config = tmp_path / "fm-comms" / "bridge-sim.json5"
    docker.host_network = False
    docker.bridge_config = config
    return docker


def test_a_run_off_the_host_network_starts_its_bridge_in_its_network(bridged, tmp_path):
    bridged.start(RUN_ID, SPEC)
    run, bridge_run = bridged.calls()
    assert run[run.index("--name") + 1] == f"fm-sim-run-{RUN_ID}"
    argv = bridge_run
    assert argv[:2] == ["run", "--detach"] and "--pull=never" in argv
    assert argv[argv.index("--name") + 1] == f"fm-sim-bridge-{RUN_ID}"
    # Its run's namespace, never the host's.
    assert argv[argv.index("--network") + 1] == f"container:fm-sim-run-{RUN_ID}"
    assert argv.count("--network") == 1 and "host" not in argv
    # Labelled as its run's, so stop_stale and active_runs find it.
    assert "fm.sim.agent=fm_ws_01" in argv and f"fm.sim.run_id={RUN_ID}" in argv
    # Exactly the two files, read-only; never the directory.
    config = tmp_path / "fm-comms" / "bridge-sim.json5"
    assert "--volume" not in argv
    mounts = [argv[i + 1] for i, a in enumerate(argv) if a == "--mount"]
    assert mounts == [
        f"type=bind,source={config},target=/fm-bridge/bridge-sim.json5,readonly",
        f"type=bind,source={config.parent / 'cyclonedds.xml'},target=/fm-bridge/cyclonedds.xml,readonly",
    ]
    # Bounded: a bridge that can never reach the router stops trying.
    assert argv[argv.index("--restart") + 1] == "on-failure:5"
    assert "ROS_DOMAIN_ID" not in argv
    assert argv[argv.index("--cap-drop") + 1] == "ALL" and argv[argv.index("--user") + 1] != "0:0"
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in argv and "ROS_DISTRO=jazzy" in argv
    assert argv[-3:] == [bridge.IMAGE, "--config", "/fm-bridge/bridge-sim.json5"]
    assert "@sha256:" in bridge.IMAGE


def test_a_mount_path_with_a_comma_or_colon_stays_one_field():
    value = bridge.mount(Path("/a,b:c/bridge-sim.json5"), "/fm-bridge/bridge-sim.json5")
    assert value == 'type=bind,"source=/a,b:c/bridge-sim.json5",target=/fm-bridge/bridge-sim.json5,readonly'


def test_the_bridge_gets_the_domain_by_name_as_the_run_does(bridged, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    argv = bridged.bridge_argv(RUN_ID, bridged.bridge_config)
    assert argv[argv.index("ROS_DOMAIN_ID") - 1] == "--env"
    assert not any(a.startswith("ROS_DOMAIN_ID=") for a in argv)


def test_a_run_without_a_bridge_config_starts_alone(docker):
    docker.host_network = False
    docker.start(RUN_ID, SPEC)
    assert len(docker.calls()) == 1


def test_a_bridge_that_does_not_start_fails_the_run_and_says_how_to_get_it(bridged, monkeypatch):
    monkeypatch.setenv("FAKE_DOCKER_FAIL_ARG", bridge.IMAGE)
    with pytest.raises(RunnerError, match="fm sim host pull local"):
        bridged.start(RUN_ID, SPEC)


def test_stop_removes_the_bridge_before_its_run(bridged):
    bridged.stop(RUN_ID)
    assert bridged.calls() == [
        ["rm", "--force", f"fm-sim-bridge-{RUN_ID}"],
        ["stop", "--time", "20", f"fm-sim-isaac-{RUN_ID}"],
        ["rm", "--force", f"fm-sim-isaac-{RUN_ID}"],
        ["stop", "--time", "20", f"fm-sim-run-{RUN_ID}"],
        ["rm", "--force", f"fm-sim-run-{RUN_ID}"],
    ]


def test_a_run_and_its_bridge_are_one_active_run(bridged, monkeypatch):
    monkeypatch.setenv("FAKE_PS_RUNS", f"{RUN_ID}\n{RUN_ID}")
    assert bridged.active_runs() == [RUN_ID]


ISAAC_RUN_ID = "20261016T120000-isaac-openarm-can_on_table-idle-7"
ISAAC_SPEC = {**SPEC, "backend": "isaac", "scenario": "can_on_table"}
ISAAC_IMAGE = "nvcr.io/nvidia/isaac-sim:4.5.0"


@pytest.fixture
def with_isaac(docker, tmp_path):
    docker.isaac_image = ISAAC_IMAGE
    docker.isaac_cache = tmp_path / "isaac-cache"
    return docker


def test_an_isaac_run_without_an_isaac_image_starts_alone(docker):
    docker.start(ISAAC_RUN_ID, ISAAC_SPEC)
    (argv,) = docker.calls()
    assert not any(a.startswith("TMPDIR=") for a in argv)


def test_an_isaac_run_shares_its_stage_and_starts_isaac_beside_it(with_isaac):
    with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)
    probe, run, sidecar = with_isaac.calls()
    assert probe[-3:] == ["/bin/ls", "fm-sim:jazzy", "/"] and "--network" in probe
    stage = with_isaac.runs_dir / ISAAC_RUN_ID / "stage"
    # The launch writes its stage under TMPDIR, the directory the Isaac container reads.
    assert f"{stage}:{isaac.STAGE_MOUNT}" in run and f"TMPDIR={isaac.STAGE_MOUNT}" in run
    assert run[run.index("--name") + 1] == f"fm-sim-run-{ISAAC_RUN_ID}"
    assert sidecar[sidecar.index("--name") + 1] == f"fm-sim-isaac-{ISAAC_RUN_ID}"
    assert f"fm.sim.run_id={ISAAC_RUN_ID}" in sidecar and "fm.sim.agent=fm_ws_01" in sidecar
    assert f"{stage}:{isaac.STAGE_MOUNT}" in sidecar
    assert (stage / isaac.CYCLONE_NAME).read_text() == isaac.CYCLONE
    for name, _ in isaac.CACHES:
        assert (with_isaac.isaac_cache / name).is_dir()


def test_the_isaac_container_reads_the_sim_image_at_its_own_paths(with_isaac):
    argv = with_isaac.isaac_argv(ISAAC_RUN_ID, ("ws", "ws_ctl"))
    mounts = [argv[i + 1] for i, a in enumerate(argv) if a == "--mount"]
    assert mounts == [
        "type=image,source=fm-sim:jazzy,target=/ws,image-subpath=ws",
        "type=image,source=fm-sim:jazzy,target=/ws_ctl,image-subpath=ws_ctl",
    ]
    assert argv[argv.index(ISAAC_IMAGE) - 1] == "/bin/bash"
    assert argv[argv.index(ISAAC_IMAGE) + 1 :] == [
        "-c", isaac.SCRIPT, "isaac-stage", str(isaac.STAGE_WAIT_S), isaac.STAGE_MOUNT, isaac.RUNNER,
    ]  # fmt: skip


def test_a_workspace_the_sim_image_lacks_is_not_mounted(with_isaac, monkeypatch):
    monkeypatch.setenv("FAKE_IMAGE_ROOT", "bin\nws")
    with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)
    sidecar = with_isaac.calls()[-1]
    assert [a for a in sidecar if a.startswith("type=image")] == [
        "type=image,source=fm-sim:jazzy,target=/ws,image-subpath=ws"
    ]


def test_the_isaac_container_is_bounded_and_on_cyclonedds(with_isaac, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    argv = with_isaac.isaac_argv(ISAAC_RUN_ID, ("ws",))
    image = argv.index(ISAAC_IMAGE)
    assert argv[argv.index("--cap-drop") + 1] == "ALL" and argv.index("--cap-drop") < image
    # The agent's own user, as a run, and nothing given back: never root, no host IPC.
    assert argv[argv.index("--user") + 1] == f"{os.getuid()}:{os.getgid()}"
    assert "--cap-add" not in argv and "--ipc" not in argv
    # The URDF importer writes into its working directory: the run's stage, not /isaac-sim.
    assert argv[argv.index("--workdir") + 1] == isaac.STAGE_MOUNT
    assert argv[argv.index("--security-opt") + 1] == "no-new-privileges"
    assert argv[argv.index("--memory") + 1] == isaac.MEMORY == argv[argv.index("--memory-swap") + 1]
    assert ["--gpus", "all"] == argv[argv.index("--gpus") : argv.index("--gpus") + 2]
    assert "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in argv
    assert f"CYCLONEDDS_URI=file://{isaac.STAGE_MOUNT}/{isaac.CYCLONE_NAME}" in argv
    assert argv[argv.index("ROS_DOMAIN_ID") - 1] == "--env"
    assert "--pull=never" in argv[:image]


def test_off_the_host_network_isaac_joins_the_runs_network(with_isaac):
    with_isaac.host_network = False
    argv = with_isaac.isaac_argv(ISAAC_RUN_ID, ("ws",))
    assert argv[argv.index("--network") + 1] == f"container:fm-sim-run-{ISAAC_RUN_ID}"
    assert "--ipc" not in argv


def test_an_isaac_run_needs_a_cache_directory(with_isaac):
    with_isaac.isaac_cache = None
    with pytest.raises(RunnerError, match="--isaac-cache"):
        with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)
    assert with_isaac.calls() == []


def test_an_isaac_container_that_does_not_start_fails_the_run(with_isaac, monkeypatch):
    monkeypatch.setenv("FAKE_DOCKER_FAIL_ARG", ISAAC_IMAGE)
    with pytest.raises(RunnerError):
        with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)


def test_the_stage_wait_gives_up_and_says_where_it_looked(tmp_path):
    done = subprocess.run(
        ["bash", "-c", isaac.SCRIPT, "isaac-stage", "1", str(tmp_path), "/nowhere.py"],
        capture_output=True, text=True, timeout=30,
    )  # fmt: skip
    assert done.returncode == 1 and f"no stage under {tmp_path}" in done.stderr


def test_the_cli_flags_reach_the_isaac_runner():
    from fm_sim_agent.service import build_parser

    args = build_parser().parse_args(["--isaac-image", ISAAC_IMAGE, "--isaac-memory", "32g"])
    assert args.isaac_image == ISAAC_IMAGE and args.isaac_memory == "32g"


def test_an_isaac_run_is_not_up_until_its_stage_plays(with_isaac):
    with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)
    with pytest.raises(RunnerError, match="not playing yet"):
        with_isaac.list_controllers(ISAAC_RUN_ID)
    (with_isaac.runs_dir / ISAAC_RUN_ID / "stage" / isaac.READY_NAME).write_text("playing\n")
    assert with_isaac.list_controllers(ISAAC_RUN_ID)["joint_state_broadcaster"] == "active"


def test_isaac_exiting_ends_the_runs_log_stream(with_isaac, monkeypatch):
    with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)
    monkeypatch.setenv("FAKE_LOGS_SLEEP", "30")  # the run would stream on; Isaac's wait returns at once
    lines = list(with_isaac.logs(ISAAC_RUN_ID))
    assert lines and "Isaac Sim exited" in lines[-1]
    assert ["wait", f"fm-sim-isaac-{ISAAC_RUN_ID}"] in with_isaac.calls()


def test_a_sidecar_that_will_not_go_still_lets_the_run_stop(docker, monkeypatch):
    monkeypatch.setenv("FAKE_DOCKER_FAIL_ARG", f"fm-sim-isaac-{RUN_ID}")
    monkeypatch.setenv("FAKE_DOCKER_FAIL_MESSAGE", "Error response from daemon: socket timeout")
    with pytest.raises(RunnerError, match="socket timeout"):
        docker.stop(RUN_ID)
    assert docker.calls()[-2:] == [
        ["stop", "--time", "20", f"fm-sim-run-{RUN_ID}"],
        ["rm", "--force", f"fm-sim-run-{RUN_ID}"],
    ]


def test_a_run_that_does_not_start_starts_no_isaac(with_isaac, monkeypatch):
    monkeypatch.setenv("FAKE_DOCKER_FAIL_ARG", f"fm-sim-run-{ISAAC_RUN_ID}")
    with pytest.raises(RunnerError):
        with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)
    assert not any(ISAAC_IMAGE in argv for argv in with_isaac.calls())


def test_the_stage_and_caches_are_the_agents_alone(with_isaac):
    with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)
    stage = with_isaac.runs_dir / ISAAC_RUN_ID / "stage"
    for path in (stage, with_isaac.isaac_cache, *(with_isaac.isaac_cache / n for n, _ in isaac.CACHES)):
        assert stat.S_IMODE(path.stat().st_mode) == 0o700, path
    assert stat.S_IMODE((stage / isaac.CYCLONE_NAME).stat().st_mode) == 0o600


@pytest.mark.parametrize("make", ["group_writable", "symlink"])
def test_a_cache_open_to_others_is_refused_not_changed(with_isaac, tmp_path, make):
    cache = with_isaac.isaac_cache
    if make == "group_writable":
        cache.mkdir(mode=0o770)
        cache.chmod(0o770)
    else:
        (tmp_path / "elsewhere").mkdir(mode=0o700)
        cache.symlink_to(tmp_path / "elsewhere")
    with pytest.raises(RunnerError, match="Isaac stage or caches"):
        with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)
    assert with_isaac.calls() == []


@pytest.fixture
def packs(tmp_path):
    """A models directory with one pack, as the robot image expects it."""
    root = tmp_path / "models"
    (root / "cube").mkdir(parents=True)
    for name in ("best.xml", "best.bin", "pack.yaml"):
        (root / "cube" / name).write_text("x")
    return root


def test_a_run_mounts_the_models_directory_read_only(docker, packs):
    docker.models_dir = packs
    docker.start(RUN_ID, SPEC)
    (argv,) = docker.calls()
    assert argv[argv.index(f"{packs}:/workspace/models:ro") - 1] == "--volume"


def test_no_models_directory_mounts_nothing(docker):
    assert not any(MODELS_MOUNT in arg for arg in docker.run_argv(RUN_ID, SPEC))


def test_only_a_run_mounts_the_models_directory(docker, packs):
    docker.models_dir = packs
    docker.catalog()
    assert not any(MODELS_MOUNT in arg for arg in docker.calls()[-1])
    assert not any(MODELS_MOUNT in arg for arg in docker.bridge_argv(RUN_ID, packs / "bridge-sim.json5"))


def test_an_isaac_container_mounts_the_assets_read_only_and_says_where(with_isaac, packs):
    with_isaac.assets_dir = packs
    argv = with_isaac.isaac_argv(ISAAC_RUN_ID, ("ws",))
    assert argv[argv.index(f"{packs}:{isaac.ASSETS_MOUNT}:ro") - 1] == "--volume"
    assert f"{isaac.ASSETS_DIR_ENV}={isaac.ASSETS_MOUNT}" in argv


def test_no_assets_directory_mounts_nothing(with_isaac):
    argv = with_isaac.isaac_argv(ISAAC_RUN_ID, ("ws",))
    assert not any(isaac.ASSETS_MOUNT in arg for arg in argv)


def test_only_the_isaac_container_mounts_the_assets(with_isaac, packs):
    with_isaac.assets_dir = packs
    assert not any(isaac.ASSETS_MOUNT in arg for arg in with_isaac.run_argv(ISAAC_RUN_ID, ISAAC_SPEC))
    assert not any(
        isaac.ASSETS_MOUNT in arg for arg in with_isaac.bridge_argv(ISAAC_RUN_ID, packs / "bridge.json5")
    )


def test_a_symlink_in_the_assets_refuses_an_isaac_run(with_isaac, packs):
    with_isaac.assets_dir = packs
    (packs / "evil").symlink_to("/etc")
    with pytest.raises(RunnerError, match="--assets-dir .* holds a symlink"):
        with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)
    assert with_isaac.calls() == []


def test_assets_replaced_since_startup_refuse_the_isaac_run(with_isaac, packs):
    with_isaac.assets_dir = packs
    packs.rename(packs.with_name("moved"))
    packs.symlink_to(packs.with_name("moved"))
    with pytest.raises(RunnerError, match="--assets-dir .* is a symlink"):
        with_isaac.start(ISAAC_RUN_ID, ISAAC_SPEC)
    assert with_isaac.calls() == []


def test_a_run_on_another_backend_does_not_walk_the_assets(docker, packs):
    docker.assets_dir = packs
    (packs / "evil").symlink_to("/etc")
    docker.start(RUN_ID, SPEC)
    assert len(docker.calls()) == 1


def _relative(root: Path) -> Path:
    return Path(os.path.relpath(root))


def _missing(root: Path) -> Path:
    return root / "absent"


def _file(root: Path) -> Path:
    return root / "cube" / "best.xml"


def _symlink(root: Path) -> Path:
    link = root.with_name("link")
    link.symlink_to(root)
    return link


def _comma(root: Path) -> Path:
    path = root.with_name("a,readonly=false")
    path.mkdir()
    return path


def _colon(root: Path) -> Path:
    path = root.with_name("a:rw")
    path.mkdir()
    return path


BAD_PATHS = [
    (_relative, "not an absolute path"),
    (_missing, "No such file"),
    (_file, "not a directory"),
    (_symlink, "is a symlink"),
    (_comma, "':' or ','"),
    (_colon, "':' or ','"),
]


@pytest.mark.parametrize(("make", "message"), BAD_PATHS)
def test_a_bad_models_path_is_refused(packs, make, message):
    with pytest.raises(RunnerError, match=message):
        check_mount_path(make(packs), "--models-dir")


@pytest.mark.parametrize(("make", "message"), BAD_PATHS)
def test_the_agent_will_not_start_on_a_bad_models_path(packs, monkeypatch, capsys, make, message):
    from fm_sim_agent import service

    monkeypatch.setenv("FM_ROUTER_ENDPOINT", "tcp/router:7447")
    assert service.main(["--host", "local", "--models-dir", str(make(packs))]) == 1
    assert message in capsys.readouterr().err


@pytest.mark.parametrize(("make", "message"), BAD_PATHS)
def test_the_agent_will_not_start_on_a_bad_assets_path(packs, monkeypatch, capsys, make, message):
    from fm_sim_agent import service

    monkeypatch.setenv("FM_ROUTER_ENDPOINT", "tcp/router:7447")
    assert service.main(["--host", "local", "--assets-dir", str(make(packs))]) == 1
    error = capsys.readouterr().err
    assert message in error and "--assets-dir" in error


def test_the_assets_flag_reaches_the_runner(packs, monkeypatch):
    from fm_sim_agent import service

    made = {}

    class Stop(Exception):
        pass

    def stand_in(*args, **kwargs):
        made.update(kwargs)
        raise Stop

    monkeypatch.setenv("FM_ROUTER_ENDPOINT", "tcp/router:7447")
    monkeypatch.setattr(service, "DockerRunner", stand_in)
    with pytest.raises(Stop):
        service.main(["--host", "local", "--assets-dir", str(packs)])
    assert made["assets_dir"] == packs
    assert service.build_parser().parse_args([]).assets_dir is None


def test_the_models_flag_reaches_the_runner(packs, monkeypatch):
    from fm_sim_agent import service

    made = {}

    class Stop(Exception):
        pass

    def stand_in(*args, **kwargs):
        made.update(kwargs)
        raise Stop

    monkeypatch.setenv("FM_ROUTER_ENDPOINT", "tcp/router:7447")
    monkeypatch.setattr(service, "DockerRunner", stand_in)
    with pytest.raises(Stop):
        service.main(["--host", "local", "--models-dir", str(packs)])
    assert made["models_dir"] == packs
    assert service.build_parser().parse_args([]).models_dir is None


@pytest.mark.parametrize("target", ["cube/best.bin", "/etc", "absent", "."], ids=str)
def test_a_symlink_in_a_pack_refuses_the_run_not_the_agent(docker, packs, target):
    (packs / "cube" / "nested").mkdir()
    (packs / "cube" / "nested" / "link").symlink_to(target)
    check_mount_path(packs, "--models-dir")  # its shape is fine: only a run looks inside
    docker.models_dir = packs
    with pytest.raises(RunnerError, match="nested/link"):
        docker.start(RUN_ID, SPEC)
    assert docker.calls() == []


def test_a_symlink_copied_in_after_a_clean_run_refuses_the_next(docker, packs):
    docker.models_dir = packs
    docker.start(RUN_ID, SPEC)
    (packs / "evil").symlink_to("/etc")
    with pytest.raises(RunnerError, match="evil"):
        docker.start(RUN_ID, SPEC)


def test_a_models_directory_replaced_since_startup_refuses_the_run(docker, packs):
    docker.models_dir = packs
    packs.rename(packs.with_name("moved"))
    packs.symlink_to(packs.with_name("moved"))
    with pytest.raises(RunnerError, match="is a symlink"):
        docker.start(RUN_ID, SPEC)
    assert docker.calls() == []


@pytest.mark.skipif(os.getuid() == 0, reason="root reads every directory")
def test_a_directory_the_walk_cannot_read_refuses_the_run(docker, packs):
    docker.models_dir = packs
    (packs / "cube").chmod(0o000)
    try:
        with pytest.raises(RunnerError, match="cannot read"):
            docker.start(RUN_ID, SPEC)
    finally:
        (packs / "cube").chmod(0o700)
    assert docker.calls() == []
