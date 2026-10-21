"""This machine as a sim host: its container runtime, the pinned image, and a
managed sim agent serving under this machine's name (contracts §8).

``fm sim host status|pull|start|stop local`` comes here, as the ``aws`` verbs
go to :mod:`fm_sim_agent.cloud`. ``local`` is the CLI's word for this machine;
on the router the agent serves ``fm/sim/<this machine>/*``, the segment
:func:`fm_sim_agent.card.this_machine_host` derives, so two Macs never collide.
The desktop app runs the same commands through its ``CommandProbe`` and never
runs Docker itself.

- ``status`` only reads: ``docker context show`` and ``docker info`` say
  whether a runtime answers and which one (OrbStack or Docker), ``docker image
  inspect`` whether the image is here and at which registry digest, and the
  pidfile plus ``ps`` whether the managed agent is up; ``bridge`` whether
  runs get live topics (:mod:`fm_sim_agent.bridge`): its config is here and its
  image is pulled.
- ``pull`` pulls the image by its pinned digest and tags it as the ref, so the
  ``--pull=never`` runs of :class:`~fm_sim_agent.runner.DockerRunner` find it.
  With no pin it pulls the tag. With the bridge config here it also pulls the
  pinned bridge image, once, so a run started offline still bridges.
- ``start`` runs the service (``python -m fm_sim_agent.service --host <this
  machine> --label local --no-host-network``) detached, in a session and
  process group of its own, so it outlives whatever ran the command, with a
  pidfile and a log under ``$XDG_STATE_HOME/fm-sim/local-agent/``. The agent is
  the service a sim host runs, with the same verbs and the same DDS isolation
  (``ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST``). Its containers carry the
  ``local`` label, which the in-process ``--host local`` verbs use too, so
  each refuses a run while the other holds one.
- ``stop`` sends that agent SIGTERM, on which it brings its active run down
  through its own ``down`` before it exits, and SIGKILLs it after a grace.

Every command is an argv list; nothing here goes through a shell. Text that
comes from Docker or ``ps`` is returned raw; the CLI cleans every string it
prints (:func:`fm_sim_agent.client.clean`).
"""

from __future__ import annotations

import fcntl
import json
import os
import re
import signal
import stat
import subprocess
import sys
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from fm_sim_agent import bridge
from fm_sim_agent.env import EndpointError, router_endpoint
from fm_sim_agent.runner import DockerRunner, RunnerError, capped_lines
from fm_sim_agent.runs import default_runs_dir
from fm_sim_agent.service import DIGEST_PATTERN
from fm_sim_agent.spec import iso, valid_name

PROVIDER = "local"
ACTIONS = ("status", "pull", "start", "stop")
#: What the CLI calls this machine. Never a key segment on the router: the
#: agent serves under the machine's own name (contracts §8 amendments).
ALIAS = "local"
#: The ``fm.sim.agent`` label on the managed agent's containers: the one the
#: in-process ``--host local`` verbs use, so each refuses the other's run.
LABEL = "local"
SCHEMA_VERSION = 1

#: ``docker`` calls that answer at once; ``pull`` has no timeout of its own,
#: since a first pull is gigabytes, and ends when its caller kills it.
DOCKER_TIMEOUT_S = 15.0
PS_TIMEOUT_S = 10.0
#: How long ``start`` watches a new agent before answering: one that exits in
#: this time (no router, a bad flag) is ``start_failed``; one that prints its
#: serving line answers at once.
START_SETTLE_S = 3.0
SERVING_MARK = "fm-sim-agent: serving"
#: tradeoff: the agent downs its active run on SIGTERM, which takes stopping
#: the recorder (15 s at most) and ``docker stop`` (20 s). Past this it is
#: killed, and the run's container removed here instead.
STOP_GRACE_S = 60.0
KILL_WAIT_S = 5.0
POLL_S = 0.1
#: The log is moved aside to ``agent.log.1`` at ``start`` once it is this big.
MAX_LOG_BYTES = 10 * 2**20

LAYER_LINE = re.compile(r"^(?P<layer>[0-9a-f]{12,64}): (?P<message>.*)$")
SIZES = re.compile(
    r"(?P<current>\d+(?:\.\d+)?)\s*(?P<cunit>[kKMGT]?B)\s*/\s*(?P<total>\d+(?:\.\d+)?)\s*(?P<tunit>[kKMGT]?B)"
)
#: Docker prints sizes with ``units.HumanSize``: decimal multiples.
UNITS = {"B": 1, "kB": 10**3, "KB": 10**3, "MB": 10**6, "GB": 10**9, "TB": 10**12}


class LocalError(Exception):
    """A refusal: ``code`` is the reply's error code."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def default_state_dir() -> Path:
    """``$XDG_STATE_HOME/fm-sim/local-agent``, beside the runs directory."""
    return default_runs_dir().parent / "local-agent"


def repository_of(ref: str) -> str:
    """``ref`` without its tag: ``ghcr.io/first-motive/fm-sim:jazzy`` → ``ghcr.io/first-motive/fm-sim``."""
    head, sep, tail = ref.rpartition(":")
    return head if sep and "/" not in tail else ref


#: What a registry says when the pull needs credentials it was not given (a private
#: image, such as fm-sim-controllers, before ``docker login``).
_AUTH_REFUSAL = re.compile(r"denied|unauthorized|authentication required", re.IGNORECASE)


def pull_failure(message: str, image: str) -> str:
    """``message`` (capped), with how to log in when the registry refused credentials."""
    message = message[:300]
    if not _AUTH_REFUSAL.search(message):
        return message
    registry = image.split("/", 1)[0]
    return f"{message}; if {image} is private, run: docker login {registry}"


def agent_command(image: str, host: str) -> list[str]:
    """The managed agent's argv: the service as this machine, off the host network."""
    return [
        sys.executable, "-m", "fm_sim_agent.service",
        "--host", host, "--label", LABEL, "--no-host-network", "--image", image,
    ]  # fmt: skip


def is_agent_for(command: str, host: str) -> bool:
    """Whether a ``ps`` command line is a sim agent for this machine.

    An agent is ``<python> -m fm_sim_agent.service`` or the ``fm-sim-agent``
    entry point as the program itself (``<python> …/fm-sim-agent`` or
    ``…/fm-sim-agent``): a command that only mentions it, such as ``grep
    fm-sim-agent``, is not one. It is this machine's when it serves ``host``
    or ``local``, or names no ``--host``, which serves the card's name: here,
    ``host``.
    """
    tokens = command.split()
    if not tokens:
        return False
    program = [Path(token).name for token in tokens[:2]]
    python = program[0].lower().startswith("python")  # a framework build re-execs as Python
    is_agent = (
        (python and tokens[1:3] == ["-m", "fm_sim_agent.service"])
        or program[0] == "fm-sim-agent"
        or (python and program[1:] == ["fm-sim-agent"])
    )
    pairs = list(zip(tokens, tokens[1:]))
    named = [b for a, b in pairs if a == "--host"] + [
        t.partition("=")[2] for t in tokens if t.startswith("--host=")
    ]
    return is_agent and (not named or any(name.replace("-", "_") in (host, ALIAS) for name in named))


def progress_event(line: str) -> dict | None:
    """One ``docker pull`` output line as a progress event, or ``None`` for a blank one.

    Without a terminal ``docker pull`` prints ``<layer>: <status>`` per layer
    change and a few plain lines (``Digest: …``, ``Status: …``). A line that
    carries ``<n><unit>/<n><unit>`` gives ``current`` and ``total`` in bytes.
    """
    text = line.strip()
    if not text:
        return None
    event: dict = {"event": "progress"}
    match = LAYER_LINE.match(text)
    if match:
        event["layer"] = match["layer"]
        text = match["message"]
    sizes = SIZES.search(text)
    if sizes and sizes["cunit"] in UNITS and sizes["tunit"] in UNITS:
        event["current"] = int(float(sizes["current"]) * UNITS[sizes["cunit"]])
        event["total"] = int(float(sizes["total"]) * UNITS[sizes["tunit"]])
    event["message"] = text
    return event


@dataclass(frozen=True)
class Pidfile:
    pid: int
    since: str
    argv: tuple[str, ...]
    host: str


def _capture(argv: list[str], timeout_s: float) -> subprocess.CompletedProcess:
    """One short command's output, as text. A timeout reads as exit 124.

    Raises ``OSError`` when the program cannot be run at all, which a caller
    may need to tell apart (no ``docker`` on ``PATH``).
    """
    try:
        return subprocess.run(
            argv,
            capture_output=True,
            text=True,
            errors="replace",
            timeout=timeout_s,
            stdin=subprocess.DEVNULL,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(argv, 124, "", f"{Path(argv[0]).name} timed out")


class LocalHost:
    """This machine's runtime, image and managed agent."""

    def __init__(
        self,
        image: str,
        pin: str | None,
        state_dir: Path,
        host: str,
        *,
        docker: str = "docker",
        bridge_config: Path | None = None,
    ) -> None:
        self.image = image
        self.host = host
        self.pin = pin
        self.state_dir = state_dir
        self.docker = docker
        #: Where the managed agent reads the bridge config: it inherits the
        #: environment that names it.
        self.bridge_config = bridge.config_path(bridge_config)

    @property
    def log_path(self) -> Path:
        return self.state_dir / "agent.log"

    @property
    def pid_path(self) -> Path:
        return self.state_dir / "agent.pid"

    # -- reading ---------------------------------------------------------

    def _docker(self, *args: str) -> subprocess.CompletedProcess:
        return _capture([self.docker, *args], DOCKER_TIMEOUT_S)

    def runtime(self) -> dict:
        try:
            context = self._docker("context", "show")
        except OSError:
            return {"available": False, "name": None, "message": "docker is not on PATH; install OrbStack"}
        context_name = context.stdout.strip() if context.returncode == 0 else ""
        info = self._docker("info", "--format", "{{.OperatingSystem}}")
        system = info.stdout.strip() if info.returncode == 0 else ""
        name = "orbstack" if "orbstack" in f"{context_name} {system}".lower() else "docker"
        where = f" (context {context_name})" if context_name else ""
        if info.returncode != 0 or not system:
            lines = (info.stderr or info.stdout).strip().splitlines()
            reason = lines[-1][:300] if lines else "no answer"
            return {"available": False, "name": name, "message": f"{name} is not running{where}: {reason}"}
        return {"available": True, "name": name, "message": f"{system}{where}"}

    def image_state(self, runtime_available: bool) -> dict:
        state = {
            "ref": self.image,
            "pinned_digest": self.pin,
            "present": False,
            "local_digest": None,
            "update_available": False,
        }
        if not runtime_available:
            return state
        try:
            inspected = self._docker("image", "inspect", "--format", "{{json .RepoDigests}}", self.image)
        except OSError:
            return state
        if inspected.returncode != 0:
            return state
        try:
            repo_digests = json.loads(inspected.stdout)
        except ValueError:
            repo_digests = None
        repository = repository_of(self.image)
        entries = repo_digests if isinstance(repo_digests, list) else []
        digests = [
            digest
            for repo, _, digest in (e.partition("@") for e in entries if isinstance(e, str))
            if repo == repository and DIGEST_PATTERN.fullmatch(digest)
        ]
        local = self.pin if self.pin in digests else (digests[0] if digests else None)
        state["present"] = True
        state["local_digest"] = local
        state["update_available"] = self.pin is not None and local != self.pin
        return state

    def bridge_image_present(self) -> bool:
        try:
            return self._docker("image", "inspect", "--format", "{{.Id}}", bridge.IMAGE).returncode == 0
        except OSError:
            return False

    def bridge_state(self, runtime_available: bool) -> dict:
        """Whether runs get live topics: ``config`` when the file is here, ``ready`` once all is."""
        return {
            "config": str(self.bridge_config) if self.bridge_config.is_file() else None,
            "ready": self.bridges() and runtime_available and self.bridge_image_present(),
        }

    def bridges(self) -> bool:
        """Whether the agent's runs get a bridge: both its files are here."""
        return bridge.active_config(self.bridge_config) is not None

    def _read_pidfile(self) -> Pidfile | None:
        try:
            if not stat.S_ISREG(os.lstat(self.pid_path).st_mode):
                return None
            record = json.loads(self.pid_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None
        if not isinstance(record, dict):
            return None
        pid, since, argv, host = (record.get(k) for k in ("pid", "since", "argv", "host"))
        if not isinstance(pid, int) or isinstance(pid, bool) or pid <= 1 or not isinstance(since, str):
            return None
        if not valid_name(host):
            return None  # it is reported, and matched against ps: only a key segment will do
        if not isinstance(argv, list) or len(argv) < 2 or not all(isinstance(a, str) for a in argv):
            return None
        return Pidfile(pid, since, tuple(argv), host)

    def _command_of(self, pid: int) -> str | None:
        """``pid``'s command line, or ``None`` if no live process has it.

        A child of this process that has exited is reaped first, so it reads
        as gone rather than as a zombie.
        """
        try:
            os.waitpid(pid, os.WNOHANG)
        except ChildProcessError:
            pass  # not ours: someone else reaps it
        try:
            done = _capture(["ps", "-ww", "-o", "stat=,command=", "-p", str(pid)], PS_TIMEOUT_S)
        except OSError:
            return None
        state, _, command = done.stdout.strip().partition(" ")
        if done.returncode != 0 or not state or state.startswith("Z"):
            return None
        return command.strip()

    def _is_managed(self, record: Pidfile) -> bool:
        """Whether the pidfile's pid is still the agent it names, not a reused pid.

        The interpreter path is not compared: a framework Python re-execs
        itself under another name. Everything after it must match.
        """
        command = self._command_of(record.pid)
        if command is None:
            return False
        return is_agent_for(command, record.host) and command.endswith(" ".join(record.argv[1:]))

    def _managed(self) -> Pidfile | None:
        record = self._read_pidfile()
        return record if record is not None and self._is_managed(record) else None

    def agent_state(self) -> dict:
        record = self._managed()
        return {
            "running": record is not None,
            "pid": record.pid if record else None,
            "since": record.since if record else None,
            "host": record.host if record else self.host,
            "log_path": str(self.log_path),
        }

    def status(self) -> dict:
        runtime = self.runtime()
        return {
            "schema_version": SCHEMA_VERSION,
            "runtime": runtime,
            "image": self.image_state(runtime["available"]),
            "agent": self.agent_state(),
            "bridge": self.bridge_state(runtime["available"]),
        }

    def other_agents(self) -> list[int]:
        """Pids of sim agents for this machine. Asked once the managed one is known not to run.

        tradeoff: a ``ps`` that cannot run or times out reads as none, so
        ``start`` goes ahead; the second agent then fails on its own, and the
        label check at its ``up`` still keeps two runs apart.
        """
        try:
            done = _capture(["ps", "-ax", "-ww", "-o", "pid=,command="], PS_TIMEOUT_S)
        except OSError:
            return []
        found = []
        for line in done.stdout.splitlines():
            pid, _, command = line.strip().partition(" ")
            if pid.isdigit() and int(pid) != os.getpid() and is_agent_for(command, self.host):
                found.append(int(pid))
        return found

    def runner(self) -> DockerRunner:
        """The runner the local label's containers belong to, for listing and removing them."""
        root = default_runs_dir().expanduser().resolve()
        return DockerRunner(self.image, root, self.host, host_network=False, docker=self.docker, label=LABEL)

    def live_runs(self) -> list[str]:
        """Run ids up under the local label.

        tradeoff: a listing that fails reads as none. ``start`` then goes
        ahead (and the agent's own check at ``up`` refuses beside a run it
        did not start), and ``stop`` removes nothing after a kill. The check
        and the start are not atomic either way: a run started between them
        is caught only by that ``up`` check.
        """
        try:
            return self.runner().active_runs()
        except RunnerError:
            return []

    # -- acting ----------------------------------------------------------

    def _prepare_state_dir(self) -> None:
        try:
            self.state_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
            info = os.lstat(self.state_dir)
            if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid():
                raise LocalError("state_unavailable", f"{self.state_dir} is not a directory this user owns")
            os.chmod(self.state_dir, 0o700)
        except OSError as exc:
            raise LocalError("state_unavailable", f"cannot prepare {self.state_dir}: {exc}") from None

    @contextmanager
    def _lock(self) -> Iterator[None]:
        """One pidfile change at a time on this machine.

        Held only to read the pidfile, decide, and spawn or signal: never
        across a Docker probe, the start-up watch or the stop grace, so a
        slow ``stop`` never stalls a ``start`` (which then finds the agent
        still running and does nothing).
        """
        self._prepare_state_dir()
        try:
            fd = os.open(self.state_dir / "lock", os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        except OSError as exc:
            raise LocalError("state_unavailable", f"cannot open the lock: {exc}") from None
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            yield
        finally:
            os.close(fd)

    def _write_pidfile(self, pid: int, argv: list[str]) -> None:
        record = {"pid": pid, "since": iso(datetime.now(timezone.utc)), "argv": argv, "host": self.host}
        scratch = self.state_dir / f".agent.pid.{os.getpid()}"
        fd = os.open(scratch, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as out:
            json.dump(record, out)
        os.replace(scratch, self.pid_path)

    def _remove_pidfile(self, pid: int | None = None) -> None:
        """Remove the pidfile; with ``pid``, only while it still names that pid."""
        if pid is not None:
            record = self._read_pidfile()
            if record is not None and record.pid != pid:
                return  # a later start wrote it
        try:
            self.pid_path.unlink()
        except FileNotFoundError:
            pass

    def _rotate_log(self) -> None:
        try:
            if self.log_path.stat().st_size > MAX_LOG_BYTES:
                os.replace(self.log_path, self.log_path.with_name("agent.log.1"))
        except FileNotFoundError:
            pass

    def _log_since(self, offset: int) -> str:
        try:
            with open(self.log_path, "rb") as log:
                log.seek(offset)
                return log.read(1 << 16).decode("utf-8", "replace")
        except OSError:
            return ""

    def start(self) -> dict:
        """Start the managed agent, or leave the running one be. Returns its LocalStatus.

        Everything slow (Docker, ``ps``) is asked before the lock is taken.
        Under it the pidfile is read again, so two starts in the same moment
        spawn one agent: the second finds the first's and does nothing.
        """
        if self._managed() is not None:
            return self.status()
        runtime = self.runtime()
        if not runtime["available"]:
            raise LocalError("runtime_unavailable", runtime["message"])
        if not self.image_state(True)["present"]:
            raise LocalError(
                "image_missing", f"{self.image} is not on this machine; `fm sim host pull local` gets it"
            )
        if self.bridges() and not self.bridge_image_present():
            raise LocalError(
                "image_missing", f"{bridge.IMAGE} is not on this machine; `fm sim host pull local` gets it"
            )
        others = self.other_agents()
        if others:
            raise LocalError(
                "already_running_elsewhere",
                f"a sim agent for this machine is already running as pid {others[0]}, not under fm sim host",
            )
        runs = self.live_runs()
        if runs:
            raise LocalError(
                "already_running_elsewhere",
                f"run {runs[0]} is active under the local label; `fm sim down --host local` ends it",
            )
        try:
            router_endpoint()
        except EndpointError as exc:
            raise LocalError("router_unavailable", str(exc)) from None
        with self._lock():
            if self._managed() is not None:
                return self.status()
            process, offset = self._spawn()
        self._settle(process, offset)
        return self.status()

    def _spawn(self) -> tuple[subprocess.Popen, int]:
        """Spawn the agent in a session and process group of its own, and file its pidfile.

        Its own group, so a caller that kills the command's whole group once it
        returns (the desktop app's runner does) leaves the agent running.
        """
        self._rotate_log()
        argv = agent_command(self.image, self.host)
        try:
            log_fd = os.open(self.log_path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        except OSError as exc:
            raise LocalError("state_unavailable", f"cannot open {self.log_path}: {exc}") from None
        offset = os.lseek(log_fd, 0, os.SEEK_END)
        try:
            process = subprocess.Popen(
                argv,
                stdin=subprocess.DEVNULL,
                stdout=log_fd,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                close_fds=True,
                env={**os.environ, "PYTHONUNBUFFERED": "1"},
            )
        except OSError as exc:
            raise LocalError("start_failed", f"cannot run the sim agent: {exc}") from None
        finally:
            os.close(log_fd)
        try:
            self._write_pidfile(process.pid, argv)
        except OSError as exc:
            # An agent with no pidfile is one nothing could stop: end it now.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
            process.wait()
            raise LocalError("start_failed", f"cannot write {self.pid_path}: {exc}") from None
        return process, offset

    def _settle(self, process: subprocess.Popen, offset: int) -> None:
        """Watch a new agent for ``START_SETTLE_S``: an early exit is ``start_failed``."""
        deadline = time.monotonic() + START_SETTLE_S
        while time.monotonic() < deadline:
            code = process.poll()
            if code is not None:
                with self._lock():
                    self._remove_pidfile(process.pid)
                lines = self._log_since(offset).strip().splitlines()
                last = f": {lines[-1][:300]}" if lines else ""
                raise LocalError(
                    "start_failed", f"the sim agent exited with {code}{last} (log {self.log_path})"
                )
            if SERVING_MARK in self._log_since(offset):
                return
            time.sleep(POLL_S)

    def _wait_gone(self, record: Pidfile, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self._is_managed(record):
            if time.monotonic() >= deadline:
                return False
            time.sleep(POLL_S)
        return True

    def _signal(self, record: Pidfile, kill_group: bool) -> None:
        """SIGTERM the agent, or SIGKILL its group; only while the pid is still that agent."""
        if not self._is_managed(record):
            return  # gone, or its pid already reused: never signal a stranger
        try:
            if kill_group:
                os.killpg(record.pid, signal.SIGKILL)  # its session: the agent and its docker calls
            else:
                os.kill(record.pid, signal.SIGTERM)  # the agent downs its run, then exits
        except (ProcessLookupError, PermissionError):
            pass

    def stop(self) -> dict:
        """Stop the managed agent, if one runs. Returns the LocalStatus after.

        The SIGTERM is sent under the lock; the grace is waited outside it.
        The runs up under the label are noted first: with the agent running,
        every one of them is its run (each refuses ``up`` beside the other's).
        Only those are removed if the agent has to be killed, so a run another
        process starts during the grace is left alone.
        """
        record = self._managed()
        if record is None:
            with self._lock():
                if self._managed() is None:
                    self._remove_pidfile()  # stale, or never there
            return self.status()
        its_runs = self.live_runs()
        with self._lock():
            self._signal(record, kill_group=False)
        killed = False
        if not self._wait_gone(record, STOP_GRACE_S):
            self._signal(record, kill_group=True)
            killed = True
            if not self._wait_gone(record, KILL_WAIT_S):
                raise LocalError("stop_failed", f"the sim agent (pid {record.pid}) did not exit")
        with self._lock():
            self._remove_pidfile(record.pid)
        if killed:
            runner = self.runner()
            for run_id in its_runs:  # the runs its down never finished
                try:
                    runner.stop(run_id)
                except RunnerError:
                    pass
        return self.status()

    def pull(self, on_progress: Callable[[dict], None]) -> dict:
        """Pull the image (by its pin when it has one), reporting each line. Returns LocalStatus."""
        runtime = self.runtime()
        if not runtime["available"]:
            raise LocalError("runtime_unavailable", runtime["message"])
        source = f"{repository_of(self.image)}@{self.pin}" if self.pin else self.image
        self._pull(source, self.image, on_progress)
        if self.pin:
            tagged = self._docker("tag", source, self.image)
            if tagged.returncode != 0:
                lines = (tagged.stderr or tagged.stdout).strip().splitlines()
                raise LocalError("pull_failed", lines[-1][:300] if lines else "docker tag failed")
            image = self.image_state(True)
            if image["local_digest"] != self.pin:
                raise LocalError(
                    "digest_mismatch", f"{self.image} is not at its pin {self.pin} after the pull"
                )
        # Pulled only when missing: a pull asks the registry even for a digest already here.
        if self.bridges() and not self.bridge_image_present():
            self._pull(bridge.IMAGE, bridge.IMAGE, on_progress)
        return self.status()

    def _pull(self, source: str, image: str, on_progress: Callable[[dict], None]) -> None:
        """``docker pull source``, reporting each line; ``image`` names it in a refusal."""
        try:
            process = subprocess.Popen(
                [self.docker, "pull", source],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
        except OSError as exc:
            raise LocalError("runtime_unavailable", f"cannot run docker: {exc}") from None
        last = ""
        try:
            for line in capped_lines(process.stdout):
                event = progress_event(line)
                if event is not None:
                    last = event["message"]
                    on_progress(event)
            code = process.wait()
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
        if code != 0:
            raise LocalError("pull_failed", pull_failure(last, image) or f"docker pull exited with {code}")


def this_machine(image: str, pin: str | None, host: str) -> LocalHost:
    return LocalHost(image, pin, default_state_dir(), host)


def managed_host(state_dir: Path | None = None) -> str | None:
    """The segment the managed agent serves, if one runs; ``None`` otherwise.

    Read from its pidfile, and only once ``ps`` confirms the pid is still that
    agent, so the CLI's ``--host local`` reaches it over the fabric only while
    it is up.
    """
    reader = LocalHost("", None, state_dir or default_state_dir(), "")
    record = reader._managed()
    return record.host if record is not None else None
