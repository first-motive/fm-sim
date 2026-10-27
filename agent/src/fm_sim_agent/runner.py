"""The container side of a run, behind one small interface.

:class:`Runner` is everything the agent asks of the sim stack: start it, ask
whether it is ready, reseed it, record it, read its outcome, stream its logs,
stop it. :class:`DockerRunner` does that with the fm-sim image; the suite swaps
in a fake.

Every command is an argv list handed to :func:`subprocess.run` or
:class:`subprocess.Popen` — never a shell string. RunSpec names are validated
before they reach here, and even so nothing a caller sends is ever parsed by a
shell. The ``bash -c`` scripts (the recorder's pid file here, the Isaac stage
wait in :mod:`fm_sim_agent.isaac`) are constants; the values they act on arrive
as positional arguments.
"""

from __future__ import annotations

import io
import json
import os
import re
import subprocess
import tarfile
import threading
from collections.abc import Iterator
from pathlib import Path
from typing import Protocol

import yaml

from fm_sim_agent import bridge, isaac
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
#: and remove a run its predecessor left behind — and only that. A run's bridge
#: carries both, so it is found and removed with its run.
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
DESCRIPTION_TIMEOUT_S = 10.0
ASSETS_TIMEOUT_S = 60.0
#: Resolves each ``package path`` pair to a file under its share directory,
#: names it on stderr (``asset<TAB>package<TAB>path<TAB>member``), and tars the
#: ones that exist. A package or file that is not there is skipped, so one
#: unresolvable mesh does not cost the rest.
ASSETS_SCRIPT = r"""
set -e
IFS=: read -ra prefixes <<< "$AMENT_PREFIX_PATH"
members=()
while [ "$#" -ge 2 ]; do
  package="$1"; path="$2"; shift 2
  for prefix in "${prefixes[@]}"; do
    file="$prefix/share/$package/$path"
    if [ -d "$prefix/share/$package" ]; then
      if [ -f "$file" ]; then
        printf 'asset\t%s\t%s\t%s\n' "$package" "$path" "${file#/}" >&2
        members+=("${file#/}")
      fi
      break
    fi
  done
done
exec tar -chf - -C / -- "${members[@]}"
"""
#: tradeoff: 120 slots is past any run's node count; the cost is a few more
#: unicast discovery probes from the one process that asks.
PROBE_CYCLONE = "<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
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
        """The latched ``/sim/outcome``, or ``None`` if none has been published.

        It is the latest episode's verdict, whichever seed that episode ran
        under: ``scenario`` and ``seed`` say which, for the caller to match.
        """

    def description(self, run_id: str) -> str:
        """The run's latched ``/robot_description``, or raise when it has none yet."""

    def assets(self, run_id: str, files: list[tuple[str, str]], limit: int) -> dict[tuple[str, str], bytes]:
        """Each ``(package, path)`` in its package's share directory, read in one
        call, at most ``limit`` bytes in all. A file that is not there is left out."""

    def stop(self, run_id: str) -> None:
        """Stop the run and remove what it left. Stopping a stopped run is not an error."""

    def logs(self, run_id: str) -> Iterator[str]:
        """The run's log lines as they arrive. The iterator ends when the run exits."""

    def active_runs(self) -> list[str]:
        """Run ids of this label's run containers that are up, whoever started them."""

    def stop_stale(self) -> list[str]:
        """Remove runs a previous agent left behind; return what was removed."""


def parse_outcome(text: str) -> dict | None:
    """``scenario``, ``seed`` and the verdict from one ``ros2 topic echo`` of Outcome."""
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
    scenario, seed = message.get("scenario"), message.get("seed")
    if not isinstance(scenario, str) or isinstance(seed, bool) or not isinstance(seed, int):
        return None
    return {
        "scenario": scenario,
        "seed": seed,
        "success": success,
        "reason": reason,
        "elapsed_s": float(elapsed),
    }


def parse_description(text: str) -> str:
    """The URDF out of ``ros2 topic echo --field data``: everything before its
    closing ``---``. Raises when nothing was echoed."""
    urdf = text.rsplit("\n---", 1)[0].strip()
    if not urdf.startswith("<"):
        raise RunnerError("no /robot_description was echoed")
    return urdf


def parse_assets(archive: bytes, listing: str) -> dict[tuple[str, str], bytes]:
    """The files out of ``ASSETS_SCRIPT``'s tar, by the ``(package, path)`` its
    stderr maps each member to. A member the listing does not name is dropped."""
    names = {}
    for line in listing.splitlines():
        parts = line.split("\t")
        if len(parts) == 4 and parts[0] == "asset":
            names[parts[3]] = (parts[1], parts[2])
    found = {}
    with tarfile.open(fileobj=io.BytesIO(archive), mode="r:") as tar:
        for member in tar:
            key = names.get(member.name)
            if key is None or not member.isfile():
                continue
            data = tar.extractfile(member)
            if data is not None:
                found[key] = data.read()
    return found


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
        label: str | None = None,
        bridge_config: Path | None = None,
        isaac_image: str | None = None,
        isaac_cache: Path | None = None,
        isaac_memory: str = isaac.MEMORY,
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
        #: The ``fm.sim.agent`` label value on every container. The host name by
        #: default; this machine's managed agent uses ``local``, the label the
        #: in-process ``--host local`` verbs use, so each sees the other's run.
        self.label = label or host
        self.host_network = host_network
        self.gpus = gpus
        self.docker_args = tuple(docker_args)
        self.docker = docker
        #: With a run off the host network, the bridge config (its cyclonedds.xml
        #: beside it) each run's bridge reads; None runs no bridge.
        self.bridge_config = bridge_config
        #: The Isaac Sim image an isaac run's stage runs in, and where its shader
        #: caches persist; with no image an isaac run only logs the runner command.
        self.isaac_image = isaac_image
        self.isaac_cache = isaac_cache
        self.isaac_memory = isaac_memory
        #: The runs this runner started an Isaac container beside.
        self._isaac_runs: set[str] = set()

    def container(self, run_id: str) -> str:
        if not valid_run_id(run_id):
            raise RunnerError("not a run id")
        return f"fm-sim-run-{run_id}"

    def bridge_container(self, run_id: str) -> str:
        if not valid_run_id(run_id):
            raise RunnerError("not a run id")
        return f"fm-sim-bridge-{run_id}"

    def isaac_container(self, run_id: str) -> str:
        if not valid_run_id(run_id):
            raise RunnerError("not a run id")
        return f"fm-sim-isaac-{run_id}"

    def _runs_isaac(self, spec: dict) -> bool:
        return spec.get("backend") == "isaac" and bool(self.isaac_image)

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
            "run", "--rm", "--pull=never", "--name", name, "--label", f"{AGENT_LABEL}={self.label}",
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
            "--label", f"{AGENT_LABEL}={self.label}",
            "--label", f"{RUN_LABEL}={run_id}",
            *self._confinement(),
            "--volume", f"{self.runs_dir / run_id / BAG_NAME}:{BAG_MOUNT}",
        ]  # fmt: skip
        if self._runs_isaac(spec):
            # The launch writes the stage under TMPDIR, which the Isaac container reads.
            stage = isaac.stage_dir(self.runs_dir, run_id)
            argv += ["--volume", f"{stage}:{isaac.STAGE_MOUNT}", "--env", f"TMPDIR={isaac.STAGE_MOUNT}"]
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

    def bridge_argv(self, run_id: str, config: Path) -> list[str]:
        """The ``docker run`` arguments for a run's bridge. Public so the suite can read them.

        It joins the run's network namespace and nothing wider: never the
        host's network. It runs confined as a run does, discovers DDS as far as
        the run does (``dds_range``), and reads only its two files, read-only.
        ``--restart on-failure:5`` brings it back, a bounded number of times,
        if the router is not reachable the moment it starts, which makes the
        bridge exit.
        """
        argv = [
            "run", "--detach", "--pull=never", "--restart", "on-failure:5",
            "--name", self.bridge_container(run_id),
            "--label", f"{AGENT_LABEL}={self.label}",
            "--label", f"{RUN_LABEL}={run_id}",
            "--network", f"container:{self.container(run_id)}",
            "--user", f"{os.getuid()}:{os.getgid()}",
            "--cap-drop", "ALL",
            "--security-opt", "no-new-privileges",
            "--pids-limit", str(bridge.PIDS_LIMIT),
            "--memory", bridge.MEMORY,
            "--mount", bridge.mount(config, bridge.CONFIG_MOUNT),
            "--mount", bridge.mount(bridge.cyclone_path(config), bridge.CYCLONE_MOUNT),
            "--env", f"ROS_DISTRO={bridge.ROS_DISTRO}",
            "--env", f"CYCLONEDDS_URI=file://{bridge.CYCLONE_MOUNT}",
            "--env", f"ROS_AUTOMATIC_DISCOVERY_RANGE={self.dds_range}",
        ]  # fmt: skip
        if os.environ.get("ROS_DOMAIN_ID"):
            argv += ["--env", "ROS_DOMAIN_ID"]  # as the run gets it; the config's domain wins
        return [*argv, bridge.IMAGE, "--config", bridge.CONFIG_MOUNT]

    def isaac_argv(self, run_id: str, workspaces: tuple[str, ...]) -> list[str]:
        """The ``docker run`` arguments for a run's Isaac container. Public so the suite can read them."""
        if not (self.isaac_image and self.isaac_cache):
            raise RunnerError("an isaac run needs --isaac-image and --isaac-cache")
        # Its own IPC namespace: unlike a run, nothing in it shares memory with the host.
        if self.host_network:
            network: tuple[str, ...] = ("--network", "host")
        else:
            network = ("--network", f"container:{self.container(run_id)}")
        return isaac.argv(
            name=self.isaac_container(run_id),
            labels=(f"{AGENT_LABEL}={self.label}", f"{RUN_LABEL}={run_id}"),
            isaac_image=self.isaac_image,
            sim_image=self.image,
            workspaces=workspaces,
            stage=isaac.stage_dir(self.runs_dir, run_id),
            cache=self.isaac_cache,
            network=network,
            memory=self.isaac_memory,
        )

    def _workspaces(self) -> tuple[str, ...]:
        """The fm-sim image's workspaces an Isaac container mounts: those the image has."""
        listed = self._run(
            [
                "run",
                "--rm",
                "--pull=never",
                "--network",
                "none",
                "--label",
                f"{AGENT_LABEL}={self.label}",
                *self._confinement(),
                "--entrypoint",
                "/bin/ls",
                self.image,
                "/",
            ],  # fmt: skip
            COMMAND_TIMEOUT_S,
        ).stdout.split()
        return tuple(name for name in isaac.WORKSPACES if name in listed)

    def _prepare_isaac(self, run_id: str) -> tuple[str, ...]:
        """Make the run's stage and the caches; return the workspaces to mount."""
        if self.isaac_cache is None:
            raise RunnerError("an isaac run needs an Isaac cache directory (--isaac-cache)")
        try:
            isaac.prepare(isaac.stage_dir(self.runs_dir, run_id), self.isaac_cache)
        except (OSError, isaac.IsaacError) as exc:
            raise RunnerError(f"cannot prepare the Isaac stage or caches: {exc}") from None
        return self._workspaces()

    def start(self, run_id: str, spec: dict) -> None:
        (self.runs_dir / run_id / BAG_NAME).mkdir(parents=True, exist_ok=True)
        workspaces = self._prepare_isaac(run_id) if self._runs_isaac(spec) else None
        self._run(self.run_argv(run_id, spec), RUN_TIMEOUT_S)
        if workspaces is not None:
            # Started after the run, so it can join the run's network when off the host's.
            self._isaac_runs.add(run_id)
            try:
                self._run(self.isaac_argv(run_id, workspaces), RUN_TIMEOUT_S)
            except RunnerError as exc:
                raise RunnerError(
                    f"Isaac Sim did not start ({exc}); fm-setup's isaac-sim step pulls {self.isaac_image}"
                ) from None
        if self.bridge_config is None:
            return
        # A bridge that cannot start fails the run, which the agent then stops
        # (bridge and run both): a run asked for live topics shows them or says why.
        try:
            self._run(self.bridge_argv(run_id, self.bridge_config), RUN_TIMEOUT_S)
        except RunnerError as exc:
            raise RunnerError(
                f"the run's bridge did not start ({exc}); `fm sim host pull local` pulls {bridge.IMAGE}"
            ) from None

    def _exec(self, run_id: str, command: list[str], timeout_s: float) -> subprocess.CompletedProcess:
        return self._run(["exec", self.container(run_id), *command], timeout_s)

    def list_controllers(self, run_id: str) -> dict[str, str]:
        # The broadcasters go active before Isaac Sim plays its stage: an isaac
        # run is not up until the stage runner says so.
        if run_id in self._isaac_runs and not isaac.ready(isaac.stage_dir(self.runs_dir, run_id)):
            raise RunnerError("Isaac Sim's stage is not playing yet")
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

    def description(self, run_id: str) -> str:
        # A run fills Cyclone's default ten participant slots on its own, so a
        # probe joining it gets "no free participant index". The probe widens
        # only its own range, appended to whatever profile the run uses.
        script = (
            'export CYCLONEDDS_URI="${CYCLONEDDS_URI:+$CYCLONEDDS_URI,}' + PROBE_CYCLONE + '" && exec "$@"'
        )
        command = [
            ENTRYPOINT, "bash", "-c", script, "fm-sim-description",
            "ros2", "topic", "echo", "--once", "--full-length", "--field", "data",
            "--qos-durability", "transient_local", "--qos-reliability", "reliable",
            "/robot_description", "std_msgs/msg/String",
        ]  # fmt: skip
        return parse_description(self._exec(run_id, command, DESCRIPTION_TIMEOUT_S).stdout)

    def assets(self, run_id: str, files: list[tuple[str, str]], limit: int) -> dict[tuple[str, str], bytes]:
        # One exec for every mesh: a viewer asks for all of them at once, and
        # each exec costs a container entry and a ROS environment. Packages and
        # paths are arguments, never spliced into the script; the script finds
        # each package's share directory on AMENT_PREFIX_PATH and tars what is
        # there, following the install tree's symlinks.
        args = [part for pair in files for part in pair]
        command = [
            "exec",
            self.container(run_id),
            ENTRYPOINT,
            "bash",
            "-c",
            ASSETS_SCRIPT,
            "fm-sim-assets",
            *args,
        ]
        try:
            done = subprocess.run(
                [self.docker, *command], capture_output=True, timeout=ASSETS_TIMEOUT_S, check=False
            )
        except subprocess.TimeoutExpired as exc:
            raise RunnerError(f"docker exec timed out after {ASSETS_TIMEOUT_S:.0f}s") from exc
        except OSError as exc:
            raise RunnerError(f"cannot run docker: {exc}") from exc
        if done.returncode != 0:
            raise RunnerError(f"reading meshes failed: {done.stderr.decode(errors='replace').strip()[-200:]}")
        if len(done.stdout) > limit:
            raise RunnerError(f"the meshes are larger than {limit} bytes")
        return parse_assets(done.stdout, done.stderr.decode(errors="replace"))

    def stop(self, run_id: str) -> None:
        # Whether or not this agent bridges or runs Isaac: what an earlier agent
        # started goes too. First, while the run's network namespace is there.
        # A sidecar that will not go never keeps the run itself up.
        self._isaac_runs.discard(run_id)
        failures: list[RunnerError] = []
        for sidecar, stop_s in (
            (self.bridge_container(run_id), 0),
            # Stopped before removal, so Isaac Sim finishes writing its caches.
            (self.isaac_container(run_id), isaac.STOP_TIMEOUT_S),
        ):
            try:
                if stop_s:
                    self._stop(sidecar, stop_s)
                else:
                    self._remove(sidecar)
            except RunnerError as exc:
                failures.append(exc)
        self._stop(self.container(run_id), STOP_TIMEOUT_S)
        if failures:
            raise failures[0]

    def _stop(self, name: str, timeout_s: int) -> None:
        """Stop then remove a container; one already gone is not an error."""
        try:
            self._run(["stop", "--time", str(timeout_s), name], timeout_s + 10)
        except RunnerError:
            pass  # already gone; the removal below says whether that is a problem
        self._remove(name)

    def _remove(self, name: str) -> None:
        """Remove a container; one already gone is not an error."""
        try:
            self._run(["rm", "--force", name], COMMAND_TIMEOUT_S)
        except RunnerError as exc:
            # Gone, or going: another stop got there first.
            if "No such container" not in str(exc) and "already in progress" not in str(exc):
                raise

    def logs(self, run_id: str) -> Iterator[str]:
        """The run's log lines. For an isaac run the stream also ends when Isaac Sim exits."""
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
        isaac_exited = threading.Event()
        if run_id in self._isaac_runs:
            threading.Thread(
                target=self._end_with_isaac, args=(run_id, process, isaac_exited), daemon=True
            ).start()
        try:
            yield from capped_lines(process.stdout)
            if isaac_exited.is_set():
                yield f"[fm_sim_agent] Isaac Sim exited; its output is in {isaac.STAGE_NAME}/{isaac.LOG_NAME}"
        finally:
            process.kill()
            process.wait()

    def _end_with_isaac(self, run_id: str, logs: subprocess.Popen, exited: threading.Event) -> None:
        """Wait for the run's Isaac container, then end its log stream: a run with no stage is no run."""
        try:
            subprocess.run(
                [self.docker, "wait", self.isaac_container(run_id)], capture_output=True, check=False
            )
        except OSError:
            return
        if logs.poll() is None and run_id in self._isaac_runs:
            exited.set()
            logs.kill()

    def active_runs(self) -> list[str]:
        """The run ids of this label's run containers that are up, newest first.

        The agent tracks its own run; it asks this at ``up`` for one another
        process on this machine started under the same label, which only Docker
        can see.
        """
        listed = self._run(
            [
                "ps", "--filter", f"label={AGENT_LABEL}={self.label}", "--filter", f"label={RUN_LABEL}",
                "--format", f'{{{{.Label "{RUN_LABEL}"}}}}',
            ],
            COMMAND_TIMEOUT_S,
        ).stdout.split()  # fmt: skip
        # A run's bridge carries its run id too: each run is listed once.
        return list(dict.fromkeys(run_id for run_id in listed if valid_run_id(run_id)))

    def stop_stale(self) -> list[str]:
        listed = self._run(
            ["ps", "--all", "--quiet", "--filter", f"label={AGENT_LABEL}={self.label}"], COMMAND_TIMEOUT_S
        ).stdout.split()
        if listed:
            self._run(["rm", "--force", *listed], COMMAND_TIMEOUT_S)
        return listed
