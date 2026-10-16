"""The container side of a run, behind one small interface.

:class:`Runner` is everything the agent asks of the sim stack: start it, ask
whether it is ready, reseed it, record it, read its outcome, stream its logs,
stop it. :class:`DockerRunner` does that with the fm-sim image; the suite swaps
in a fake.

Every command is an argv list handed to :func:`subprocess.run` or
:class:`subprocess.Popen` — never a shell string. RunSpec names are validated
before they reach here, and even so nothing a caller sends is ever parsed by a
shell. The two ``bash -c`` scripts below (the recorder's pid file) are constants;
the values they act on arrive as positional arguments.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
from collections.abc import Iterator
from pathlib import Path
from typing import Protocol

import yaml

from fm_sim_agent.logline import MAX_LINE_BYTES
from fm_sim_agent.runs import BAG_NAME, RECORDING_NAME
from fm_sim_agent.spec import DIMENSIONS, valid_run_id

#: What a run launches inside the image (contracts §1).
LAUNCH = ("ros2", "launch", "fm_sim_bringup", "run.launch.py")
CATALOG = ("ros2", "run", "fm_sim_bringup", "catalog", "--json")

#: The image has no ENTRYPOINT of its own to source ROS, so every command goes
#: through this, as run.sh does.
ENTRYPOINT = "/ros_entrypoint.sh"

#: Where the run's ``bag/`` directory is mounted inside its container, so a bag
#: recorded in there lands under the run's directory on the host. Only that one
#: directory is mounted: a run cannot read or overwrite any other run's files,
#: nor its own summary.
BAG_MOUNT = "/fm-run/bag"

#: The host's loopback Cyclone profile, mounted where fm-docker's compose base
#: mounts it, so CYCLONEDDS_URI reads the same on every host.
CYCLONE_MOUNT = "/etc/fm-comms/cyclonedds.xml"

#: Transport settings passed through from the agent's own environment, as the
#: fm-docker compose base passes them, so a run's DDS graph is the one the host's
#: zenoh bridge routes. Only names are on the command line (``-e NAME``); Docker
#: reads the values from the environment it inherits.
#:
#: Nothing that sets how far DDS discovery reaches is passed through: that is
#: ``--dds-range`` below, and a host env var must not widen it silently.
PASSTHROUGH_ENV = ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "FM_COMMS_PROFILE")
DEFAULT_RMW = "rmw_cyclonedds_cpp"

#: How far a run's DDS discovery reaches (Jazzy's ROS_AUTOMATIC_DISCOVERY_RANGE).
#: LOCALHOST by default: a sim host with --network host shares a LAN with real
#: robots, and on a shared ROS_DOMAIN_ID a wider range would let sim topics —
#: joint states, controller commands — reach a real robot's graph. The zenoh
#: bridge runs on the same host, so it still sees the run. Only an operator flag
#: changes it. ROS_LOCALHOST_ONLY is not set as well: the image is Jazzy-only,
#: where that variable is deprecated and setting both draws a warning.
DDS_RANGES = ("LOCALHOST", "SUBNET", "OFF", "SYSTEM_DEFAULT")
DEFAULT_DDS_RANGE = "LOCALHOST"

#: Labels that mark a container as this agent's, so a restarted agent can find
#: and remove a run its predecessor left behind — and only that.
AGENT_LABEL = "fm.sim.agent"
RUN_LABEL = "fm.sim.run_id"

#: The recorder runs beside the launch through ``docker exec``. It writes its pid
#: so ``stop_recording`` can send it SIGINT and let it close the MCAP cleanly; a
#: container stop would SIGKILL it and leave the file without its footer.
BAG_PID = "/tmp/fm-sim-bag.pid"
RECORD_SCRIPT = f'echo $$ > {BAG_PID} && exec "$@"'
STOP_RECORD_SCRIPT = (
    f'pid="$(cat {BAG_PID} 2>/dev/null)" || exit 0; '
    'kill -INT "$pid" 2>/dev/null || exit 0; '
    'for _ in $(seq 100); do kill -0 "$pid" 2>/dev/null || exit 0; sleep 0.1; done; exit 1'
)

LIST_CONTROLLERS = (
    "ros2", "service", "call",
    "/controller_manager/list_controllers", "controller_manager_msgs/srv/ListControllers",
)  # fmt: skip
SIM_RESET = "/sim/reset"

#: How long each container call may take. A catalog is a fresh container that
#: lists the resource index; the readiness probes are short because the agent
#: polls them.
CATALOG_TIMEOUT_S = 120.0
#: `docker run --detach` of a run: --pull=never keeps a missing image a fast
#: failure, and this leaves room for creating the container on a loaded host.
RUN_TIMEOUT_S = 90.0
COMMAND_TIMEOUT_S = 30.0
PROBE_TIMEOUT_S = 10.0
OUTCOME_TIMEOUT_S = 5.0
STOP_TIMEOUT_S = 20
RECORD_STOP_TIMEOUT_S = 15.0

#: tradeoff: per-run ceilings, overridable by the operator. 4096 processes is
#: far past what a launch spawns and well short of a fork bomb taking the host;
#: 16 GiB fits MuJoCo and Gazebo with room, and an Isaac host raises it.
DEFAULT_PIDS_LIMIT = 4096
DEFAULT_MEMORY = "16g"

#: tradeoff: the recorder splits its MCAP at whichever of these comes first,
#: so no one file grows past what a laptop opens comfortably. Splitting bounds
#: file size, not the run's total; that is bounded by the agent's maximum run
#: duration and its free-space check at ``up``.
DEFAULT_BAG_SPLIT_S = 600
DEFAULT_BAG_SPLIT_BYTES = 4 * 2**30

RESET_RESPONSE = re.compile(r"ok=(True|False)(?:,\s*message=(['\"])(.*?)\2)?", re.DOTALL)
CONTROLLER_STATE = re.compile(r"ControllerState\(name='([^']*)', state='([^']*)'")


class RunnerError(Exception):
    """The container side refused or failed. The message says why, for the agent's log."""


class Runner(Protocol):
    """What the agent needs from whatever hosts a run's ROS stack."""

    def image_id(self) -> str:
        """An id that changes whenever the image does. The catalog is cached on it."""

    def catalog(self) -> dict:
        """The image's catalog JSON (contracts §1), parsed but not yet checked."""

    def start(self, run_id: str, spec: dict) -> None:
        """Start the run's launch and return once it is started, not once it is ready."""

    def list_controllers(self, run_id: str) -> dict[str, str]:
        """``{controller name: state}`` from the run's controller_manager, or raise."""

    def has_service(self, run_id: str, name: str) -> bool:
        """Whether the run's graph currently serves ``name``."""

    def reset(self, run_id: str, seed: int, timeout_s: float) -> str:
        """Call ``/sim/reset`` with ``seed``."""

    def start_recording(self, run_id: str) -> None:
        """Record every topic as MCAP into the run's ``bag/`` directory."""

    def stop_recording(self, run_id: str) -> None:
        """Stop the recorder and let it finish the file."""

    def outcome(self, run_id: str) -> dict | None:
        """The latched ``/sim/outcome``, or ``None`` if none has been published."""

    def stop(self, run_id: str) -> None:
        """Stop the run and remove what it left. Stopping a stopped run is not an error."""

    def logs(self, run_id: str) -> Iterator[str]:
        """The run's log lines as they arrive. The iterator ends when the run exits."""

    def stop_stale(self) -> list[str]:
        """Remove runs a previous agent left behind; return what was removed."""


def parse_outcome(text: str) -> dict | None:
    """The fields RunSummary keeps from one ``ros2 topic echo`` of Outcome."""
    try:
        documents = [doc for doc in yaml.safe_load_all(text) if isinstance(doc, dict)]
    except yaml.YAMLError:
        return None
    if not documents:
        return None
    message = documents[0]
    success, reason, elapsed = message.get("success"), message.get("reason"), message.get("elapsed_s")
    if not isinstance(success, bool) or not isinstance(reason, str):
        return None
    if isinstance(elapsed, bool) or not isinstance(elapsed, (int, float)):
        return None
    return {"success": success, "reason": reason, "elapsed_s": float(elapsed)}


def parse_reset(text: str) -> tuple[bool, str]:
    """``ok`` and ``message`` from ``ros2 service call``'s printed response."""
    match = RESET_RESPONSE.search(text)
    if match is None:
        return False, "no response from /sim/reset"
    return match.group(1) == "True", match.group(3) or ""


def parse_controllers(text: str) -> dict[str, str]:
    """``{name: state}`` from ``ros2 service call``'s printed ListControllers response."""
    return dict(CONTROLLER_STATE.findall(text))


def launch_args(spec: dict) -> list[str]:
    """The ``name:=value`` arguments one RunSpec launches with."""
    return [*(f"{dimension}:={spec[dimension]}" for dimension in DIMENSIONS), "use_sim:=true"]


def capped_lines(stream) -> Iterator[str]:
    """Lines from a text stream, each cut at ``MAX_LINE_BYTES`` characters.

    ``readline`` with a size never holds more than that in memory, so one line
    with no newline cannot grow the agent without bound. The rest of an
    over-long line is read and dropped.
    """
    dropping = False
    while True:
        chunk = stream.readline(MAX_LINE_BYTES)
        if not chunk:
            return
        complete = chunk.endswith("\n")
        if not dropping:
            yield chunk
        dropping = not complete


class DockerRunner:
    """Runs each run as one detached container of the fm-sim image."""

    def __init__(
        self,
        image: str,
        runs_dir: Path,
        host: str,
        *,
        host_network: bool = True,
        gpus: bool = False,
        docker_args: tuple[str, ...] = (),
        pids_limit: int = DEFAULT_PIDS_LIMIT,
        memory: str = DEFAULT_MEMORY,
        bag_split_s: int = DEFAULT_BAG_SPLIT_S,
        bag_split_bytes: int = DEFAULT_BAG_SPLIT_BYTES,
        dds_range: str = DEFAULT_DDS_RANGE,
        docker: str = "docker",
    ) -> None:
        if dds_range not in DDS_RANGES:
            raise ValueError(f"dds_range must be one of {DDS_RANGES}")
        self.dds_range = dds_range
        self.image = image
        self.pids_limit = pids_limit
        self.memory = memory
        self.bag_split_s = bag_split_s
        self.bag_split_bytes = bag_split_bytes
        self.runs_dir = runs_dir
        self.host = host
        self.host_network = host_network
        self.gpus = gpus
        self.docker_args = tuple(docker_args)
        self.docker = docker

    def container(self, run_id: str) -> str:
        if not valid_run_id(run_id):
            raise RunnerError("not a run id")
        return f"fm-sim-run-{run_id}"

    def _run(self, args: list[str], timeout_s: float) -> subprocess.CompletedProcess:
        try:
            done = subprocess.run(
                [self.docker, *args], capture_output=True, text=True, timeout=timeout_s, check=False
            )
        except subprocess.TimeoutExpired as exc:
            raise RunnerError(f"docker {args[0]} timed out after {timeout_s:.0f}s") from exc
        except OSError as exc:
            raise RunnerError(f"cannot run docker: {exc}") from exc
        if done.returncode != 0:
            tail = (done.stderr or done.stdout).strip().splitlines()[-3:]
            raise RunnerError(f"docker {args[0]} failed: {' / '.join(tail) or done.returncode}")
        return done

    def image_id(self) -> str:
        return self._run(
            ["image", "inspect", "--format", "{{.Id}}", self.image], COMMAND_TIMEOUT_S
        ).stdout.strip()

    def catalog_container(self, image_id: str) -> str:
        return f"fm-sim-catalog-{image_id.removeprefix('sha256:')[:12]}"

    def catalog(self) -> dict:
        # Named and labelled, so a fetch that times out leaves nothing behind:
        # the name is what the cleanup below removes, the label what a
        # restarted agent's stop_stale finds.
        name = self.catalog_container(self.image_id())
        argv = [
            "run", "--rm", "--pull=never", "--name", name, "--label", f"{AGENT_LABEL}={self.host}",
            *self._confinement(), self.image, ENTRYPOINT, *CATALOG,
        ]  # fmt: skip
        try:
            done = self._run(argv, CATALOG_TIMEOUT_S)
        except RunnerError:
            try:
                self._run(["rm", "--force", name], COMMAND_TIMEOUT_S)
            except RunnerError:
                pass  # never created, or already gone
            raise
        try:
            return json.loads(done.stdout)
        except ValueError as exc:
            raise RunnerError(f"catalog is not JSON: {exc}") from exc

    def _confinement(self) -> list[str]:
        """What every container of the image runs under, whatever it is for.

        The agent's own account, never root, with every capability dropped and
        no way to gain one back through a setuid binary. HOME moves with the
        account: ROS writes ``~/.ros/log`` on start. DDS discovery stays within
        ``dds_range``; every ``docker exec`` into the container (reset, probes,
        the recorder) inherits the container's environment, so it does too.
        """
        return [
            "--user", f"{os.getuid()}:{os.getgid()}",
            "--env", "HOME=/tmp",
            "--cap-drop", "ALL",
            "--security-opt", "no-new-privileges",
            "--pids-limit", str(self.pids_limit),
            "--memory", self.memory,
            "--env", f"ROS_AUTOMATIC_DISCOVERY_RANGE={self.dds_range}",
        ]  # fmt: skip

    def run_argv(self, run_id: str, spec: dict) -> list[str]:
        """The ``docker run`` arguments for one run. Public so the suite can read them."""
        argv = [
            "run", "--detach", "--pull=never",
            "--name", self.container(run_id),
            "--label", f"{AGENT_LABEL}={self.host}",
            "--label", f"{RUN_LABEL}={run_id}",
            *self._confinement(),
            "--volume", f"{self.runs_dir / run_id / BAG_NAME}:{BAG_MOUNT}",
        ]  # fmt: skip
        # Host networking, IPC and GPUs widen what a run can reach. They are the
        # operator's flags on the agent, never a caller's choice on the wire.
        if self.host_network:
            argv += ["--network", "host", "--ipc", "host"]
        if self.gpus:
            argv += ["--gpus", "all"]
        for name in PASSTHROUGH_ENV:
            if os.environ.get(name):
                argv += ["--env", name]
        if not os.environ.get("RMW_IMPLEMENTATION"):
            argv += ["--env", f"RMW_IMPLEMENTATION={DEFAULT_RMW}"]
        cyclone_xml = os.environ.get("FM_CYCLONEDDS_XML", "").strip()
        if cyclone_xml:
            argv += [
                "--volume",
                f"{cyclone_xml}:{CYCLONE_MOUNT}:ro",
                "--env",
                f"CYCLONEDDS_URI=file://{CYCLONE_MOUNT}",
            ]
        argv += [*self.docker_args, self.image, ENTRYPOINT, *LAUNCH, *launch_args(spec)]
        return argv

    def start(self, run_id: str, spec: dict) -> None:
        (self.runs_dir / run_id / BAG_NAME).mkdir(parents=True, exist_ok=True)
        self._run(self.run_argv(run_id, spec), RUN_TIMEOUT_S)

    def _exec(self, run_id: str, command: list[str], timeout_s: float) -> subprocess.CompletedProcess:
        return self._run(["exec", self.container(run_id), *command], timeout_s)

    def list_controllers(self, run_id: str) -> dict[str, str]:
        return parse_controllers(self._exec(run_id, [ENTRYPOINT, *LIST_CONTROLLERS], PROBE_TIMEOUT_S).stdout)

    def has_service(self, run_id: str, name: str) -> bool:
        listed = self._exec(run_id, [ENTRYPOINT, "ros2", "service", "list"], PROBE_TIMEOUT_S).stdout
        return name in (line.strip() for line in listed.splitlines())

    def reset(self, run_id: str, seed: int, timeout_s: float) -> str:
        request = json.dumps({"seed": int(seed)})
        command = [ENTRYPOINT, "ros2", "service", "call", SIM_RESET, "fm_sim_msgs/srv/Reset", request]
        ok, message = parse_reset(self._exec(run_id, command, timeout_s).stdout)
        if not ok:
            raise RunnerError(f"/sim/reset refused: {message or 'no reason given'}")
        return message

    def start_recording(self, run_id: str) -> None:
        bag = f"{BAG_MOUNT}/{RECORDING_NAME}"
        command = [
            "bash",
            "-c",
            RECORD_SCRIPT,
            "fm-sim-bag",
            ENTRYPOINT,
            "ros2",
            "bag",
            "record",
            "-s",
            "mcap",
            "-a",
            "--max-bag-duration",
            str(self.bag_split_s),
            "--max-bag-size",
            str(self.bag_split_bytes),
            "-o",
            bag,
        ]
        self._run(["exec", "--detach", self.container(run_id), *command], COMMAND_TIMEOUT_S)

    def stop_recording(self, run_id: str) -> None:
        self._exec(run_id, ["bash", "-c", STOP_RECORD_SCRIPT], RECORD_STOP_TIMEOUT_S)

    def outcome(self, run_id: str) -> dict | None:
        command = [
            ENTRYPOINT, "ros2", "topic", "echo", "--once",
            "--qos-durability", "transient_local", "--qos-reliability", "reliable",
            "/sim/outcome", "fm_sim_msgs/msg/Outcome",
        ]  # fmt: skip
        try:
            return parse_outcome(self._exec(run_id, command, OUTCOME_TIMEOUT_S).stdout)
        except RunnerError:
            # No outcome latched yet is the common case for a run stopped early,
            # and the echo then waits until the timeout. Null, not an error.
            return None

    def stop(self, run_id: str) -> None:
        name = self.container(run_id)
        try:
            self._run(["stop", "--time", str(STOP_TIMEOUT_S), name], STOP_TIMEOUT_S + 10)
        except RunnerError:
            pass  # already gone; the removal below says whether that is a problem
        try:
            self._run(["rm", "--force", name], COMMAND_TIMEOUT_S)
        except RunnerError as exc:
            if "No such container" not in str(exc):
                raise

    def logs(self, run_id: str) -> Iterator[str]:
        try:
            process = subprocess.Popen(
                [self.docker, "logs", "--follow", self.container(run_id)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                errors="replace",
            )
        except OSError as exc:
            raise RunnerError(f"cannot run docker: {exc}") from exc
        assert process.stdout is not None
        try:
            yield from capped_lines(process.stdout)
        finally:
            process.kill()
            process.wait()

    def active_runs(self) -> list[str]:
        """The run ids of this host label's run containers that are up, newest first.

        Not part of :class:`Runner`: the agent tracks its own run. The CLI's
        local mode asks, because a run started by another ``fm sim`` process on
        this machine is visible only to Docker.
        """
        listed = self._run(
            [
                "ps", "--filter", f"label={AGENT_LABEL}={self.host}", "--filter", f"label={RUN_LABEL}",
                "--format", f'{{{{.Label "{RUN_LABEL}"}}}}',
            ],
            COMMAND_TIMEOUT_S,
        ).stdout.split()  # fmt: skip
        return [run_id for run_id in listed if valid_run_id(run_id)]

    def stop_stale(self) -> list[str]:
        listed = self._run(
            ["ps", "--all", "--quiet", "--filter", f"label={AGENT_LABEL}={self.host}"], COMMAND_TIMEOUT_S
        ).stdout.split()
        if listed:
            self._run(["rm", "--force", *listed], COMMAND_TIMEOUT_S)
        return listed
