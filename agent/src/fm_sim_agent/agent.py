"""The sim agent's verbs and run lifecycle — with no Zenoh in sight.

:meth:`SimAgent.answer` turns one verb and its JSON body into one reply body,
``{"ok": true, ...}`` or ``{"ok": false, "error": {"code", "message"}}``
(contracts §3). :mod:`fm_sim_agent.service` only carries bytes between Zenoh and
this, the split fm-robot-agent's ``verbs.answer`` makes, for the same reason:
the suite drives every verb with a fake runner and no router.

A run moves through these states, one run per host:

    idle ──up──▶ starting ──broadcasters active──▶ running ──down──▶ stopping ──▶ idle
                    │                                 │
                    └── start fails / not ready / container exits ──▶ error (run finalised)

``starting`` lasts until the run's controller_manager lists every broadcaster
the robot's catalog entry names (``controller_roles.broadcasters``) as active.
Then, if the graph serves ``/sim/reset``, it is called with the spec's seed.
A graph without it cannot be seeded, and the summary says so: its metrics carry
``seed_applied``, true only when the seed took effect (seed 0 is the default a
scenario starts from, so it counts as applied either way).

``error`` holds no run: the failed run's summary is already written, and the
next ``up`` is accepted.

``description`` and ``asset`` let a viewer draw the active run's robot from its
own meshes. ``description`` is the run's ``/robot_description``, read once per
run. ``asset`` is one mesh's bytes, and only a mesh that description draws: the
URI must be one of its ``<visual>`` mesh filenames, so a caller can name no other
file in the image. Reading the description starts one background read of every
mesh it draws; ``asset`` answers from that, and the meshes are kept across runs
until the image changes.

What a caller is told is deliberately plain: a code and a short reason. What a
container call printed — Docker's stderr, a traceback — goes to the agent's own
log, never onto the fabric.

Locking: ``_lock`` guards the state fields and the catalog cache only. Every
container call — seconds to minutes — runs outside it, so ``status`` always
answers at once whatever ``up`` or ``down`` is waiting on. ``_catalog_flight``
is held across a catalog fetch, so concurrent callers share one fetch; nothing
takes it while holding ``_lock``.
"""

from __future__ import annotations

import json
import re
import shutil
import sys
import threading
import time
import traceback
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime
from xml.etree import ElementTree
from importlib.metadata import PackageNotFoundError, version

from fm_sim_agent import logline
from fm_sim_agent.runner import SIM_RESET, Runner, RunnerError
from fm_sim_agent.runs import RunStore
from fm_sim_agent.spec import (
    DIMENSIONS,
    SpecError,
    iso,
    parse_seed,
    parse_spec,
    run_id_of,
    utc_now,
    valid_name,
    valid_run_id,
)

VERBS = ("status", "catalog", "up", "down", "reset", "runs", "description", "asset")

#: Verbs whose success reply is raw bytes rather than a JSON body.
BINARY_VERBS = ("asset",)

#: tradeoff: a URDF is tens of kilobytes and the largest OpenArm mesh is about
#: a megabyte; these bound a description or image that sends far more.
MAX_DESCRIPTION_BYTES = 4 << 20
MAX_ASSET_BYTES = 64 << 20
#: tradeoff: every mesh one robot draws, read in one go; OpenArm's are about
#: 5 MB. The cache across runs holds at most this much.
MAX_MESHES_BYTES = 256 << 20
#: How long ``asset`` waits on the background read before giving up.
MESHES_WAIT_S = 60.0

#: ``package://<package>/<path>``, the only mesh URI form a sim robot uses.
PACKAGE_URI = re.compile(r"package://([A-Za-z][A-Za-z0-9_]*)/([A-Za-z0-9_./-]+)")

#: Ceiling on a request body. Every body the contract defines is a few hundred
#: bytes; anything past this is a caller spending the agent's memory.
MAX_BODY_BYTES = 1 << 16

#: How long a new run has to bring its broadcasters up. Gazebo and Isaac take
#: tens of seconds to load a world before controller_manager spawns anything; a
#: launch whose broadcasters are not active by then is a launch that failed.
STARTUP_TIMEOUT_S = 180.0
READY_POLL_S = 1.0
RESET_TIMEOUT_S = 30.0

#: A catalog that could not be fetched is not asked for again for this long, so
#: a broken image costs one container per window, not one per query.
CATALOG_FAILURE_TTL_S = 10.0

#: tradeoff: at most this many log records a second per run reach the fabric.
#: A healthy launch prints a few a second; a node logging in a tight loop can
#: print thousands. Lines past the ceiling are dropped and counted, and one warn
#: record per window says how many — the caller loses those lines' text, never
#: the fact that they existed.
LOG_MAX_PER_S = 200

#: tradeoff: a run is brought down on its own after this long. A trial on one
#: scenario takes minutes; a run left up overnight holds the GPU and, when
#: recording, keeps filling the disk. Operators raise it with --max-run-minutes.
MAX_RUN_S = 2 * 60 * 60

#: tradeoff: ``up`` is refused when the runs directory has less free than this.
#: A recorded run with cameras writes on the order of a gigabyte a minute; a
#: run started onto a nearly full disk fails mid-recording and can take the
#: host's other writers with it. Operators change it with --min-free-gb.
MIN_FREE_BYTES = 10 * 2**30

RUNS_DEFAULT_LIMIT = 20
RUNS_MAX_LIMIT = 1000

CATALOG_SCHEMA_VERSION = 1
CATALOG_ENTRY_KEYS = ("name", "package", "description", "available")

#: Where one log record goes: (run_id, record). The service publishes it.
LogSink = Callable[[str, dict], None]


def log(message: str) -> None:
    print(f"fm-sim-agent: {message}", file=sys.stderr, flush=True)


class VerbError(Exception):
    """A refusal, carried to the caller as ``{"code", "message"}``."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass
class Run:
    """One run the agent is driving."""

    run_id: str
    spec: dict
    started_at: datetime
    broadcasters: tuple[str, ...] = ()
    ended_at: datetime | None = None
    outcome: dict | None = None
    mcap: str | None = None
    metrics: dict = field(default_factory=dict)
    #: The seed of the last reset that took, or None while the spec's seed holds.
    #: Only an outcome under the seed in force is this run's. Set under ``write_lock``.
    reset_seed: int | None = None
    #: Set once the run is finalised, whoever finalised it.
    done: threading.Event = field(default_factory=threading.Event)
    #: True while ``docker run`` is in flight. Guarded by the agent's ``_lock``:
    #: a down that sees it leaves the stop to ``_start_container``.
    in_docker_run: bool = False
    #: Held while a summary is built and written, so a writer holding an older
    #: view of the run can never land after a newer one. Every change to
    #: ``metrics``, ``mcap``, ``outcome`` and ``ended_at`` after the run starts
    #: is made under it too. It is never taken while holding the agent's
    #: ``_lock``, and ``_lock`` is never taken while holding it.
    write_lock: threading.Lock = field(default_factory=threading.Lock)
    #: The run's URDF and the visual mesh URIs it names, read once on first ask.
    description: tuple[str, frozenset[str]] | None = None
    description_lock: threading.Lock = field(default_factory=threading.Lock)
    #: Set once the background read of the description's meshes has ended.
    meshes_read: threading.Event = field(default_factory=threading.Event)


def agent_version() -> str:
    try:
        return version("fm-sim-agent")
    except PackageNotFoundError:
        return "0.0.0"


def error_body(code: str, message: str) -> dict:
    return {"ok": False, "error": {"code": code, "message": message}}


def decode_body(payload: bytes | None) -> dict:
    """A request body as a dict. Absent is empty; anything else not an object is refused."""
    if not payload:
        return {}
    if len(payload) > MAX_BODY_BYTES:
        raise VerbError("invalid_request", f"body is larger than {MAX_BODY_BYTES} bytes")
    try:
        decoded = json.loads(payload)
    except (ValueError, UnicodeDecodeError, RecursionError):
        # RecursionError: JSON nested deeper than the parser's stack.
        raise VerbError("invalid_request", "body is not JSON") from None
    if not isinstance(decoded, dict):
        raise VerbError("invalid_request", "body must be a JSON object")
    return decoded


def visual_meshes(urdf: str) -> frozenset[str]:
    """Every ``<visual>`` mesh filename a URDF names. An unparsable URDF names none."""
    try:
        root = ElementTree.fromstring(urdf)
    except ElementTree.ParseError:
        return frozenset()
    return frozenset(
        mesh.get("filename", "")
        for mesh in root.iterfind("./link/visual/geometry/mesh")
        if mesh.get("filename")
    )


def check_catalog(doc: object) -> dict:
    """The catalog if it has the shape contracts §1 names, or raise ``ValueError``."""
    if not isinstance(doc, dict) or doc.get("schema_version") != CATALOG_SCHEMA_VERSION:
        raise ValueError(f"catalog is not schema_version {CATALOG_SCHEMA_VERSION}")
    for dimension in DIMENSIONS:
        entries = doc.get(f"{dimension}s")
        if not isinstance(entries, list) or not all(
            isinstance(e, dict) and all(k in e for k in CATALOG_ENTRY_KEYS) for e in entries
        ):
            raise ValueError(f"catalog {dimension}s is not a list of entries")
    return doc


def broadcasters_of(robot: dict) -> tuple[str, ...]:
    """The broadcasters a robot's run must have active to count as ready, or raise."""
    roles = robot.get("controller_roles")
    names = roles.get("broadcasters") if isinstance(roles, dict) else None
    # Empty is refused too: a robot with no broadcaster would be "ready" the
    # moment controller_manager answers, before anything publishes state.
    if not isinstance(names, list) or not names or not all(valid_name(n) for n in names):
        raise ValueError(f"robot {robot.get('name')!r} has no controller_roles.broadcasters list")
    return tuple(names)


class SimAgent:
    """One host's run control, behind the verb set."""

    def __init__(self, host: str, runner: Runner, store: RunStore, log_sink: LogSink | None = None) -> None:
        self.host = host
        self.runner = runner
        self.store = store
        self.log_sink = log_sink or (lambda _run_id, _record: None)
        self.version = agent_version()
        self.startup_timeout_s = STARTUP_TIMEOUT_S
        self.ready_poll_s = READY_POLL_S
        self.max_run_s = MAX_RUN_S
        self.min_free_bytes = MIN_FREE_BYTES
        #: Whether other processes may hold runs under this agent's label (see ``recover``).
        self.keep_foreign_runs = False
        self._lock = threading.Lock()
        self._catalog_flight = threading.Lock()
        self._state = "idle"
        self._since = utc_now()
        self._error: str | None = None
        self._run: Run | None = None
        self._catalog: tuple[str, dict] | None = None
        self._catalog_failed_at: float | None = None
        #: Mesh bytes by URI, for the image they were read from. Guarded by ``_meshes_lock``.
        self._meshes: dict[str, bytes] = {}
        self._meshes_image: str | None = None
        self._meshes_lock = threading.Lock()

    # -- the verb set -------------------------------------------------------

    def answer(self, verb: str, payload: bytes | None = None) -> dict:
        """One reply body for one verb. Never raises, whatever a caller sent."""
        handlers = {
            "status": self._status,
            "catalog": self._catalog_verb,
            "up": self._up,
            "down": self._down,
            "reset": self._reset,
            "runs": self._runs,
            "description": self._description,
            "asset": self._asset,
        }
        try:
            if verb not in handlers:
                raise VerbError("unknown_verb", "unknown verb")
            return {"ok": True, **handlers[verb](decode_body(payload))}
        except VerbError as exc:
            return error_body(exc.code, exc.message)
        except Exception:  # noqa: BLE001 - a bug answers the caller and is logged here
            log(f"{verb} failed:\n{traceback.format_exc()}")
            return error_body("internal_error", "the agent failed; see its log")

    def _status(self, _body: dict) -> dict:
        return self.status()

    def _catalog_verb(self, _body: dict) -> dict:
        return {"catalog": self.catalog()}

    def _up(self, body: dict) -> dict:
        return {"run_id": self.up(body)}

    def _down(self, body: dict) -> dict:
        run_id = body.get("run_id")
        if run_id is not None and not valid_run_id(run_id):
            raise VerbError("invalid_request", "run_id is not a run id")
        return {"summary": self.down(run_id)}

    def _reset(self, body: dict) -> dict:
        if "seed" not in body:
            raise VerbError("invalid_request", "seed is required")
        try:
            seed = parse_seed(body["seed"])
        except SpecError as exc:
            raise VerbError("invalid_request", str(exc)) from None
        self.reset(seed)
        return {}

    def _runs(self, body: dict) -> dict:
        limit = body.get("limit", RUNS_DEFAULT_LIMIT)
        if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= RUNS_MAX_LIMIT:
            raise VerbError("invalid_request", f"limit must be an integer from 1 to {RUNS_MAX_LIMIT}")
        return {"runs": self.store.recent(limit)}

    def _description(self, _body: dict) -> dict:
        run, (urdf, _meshes) = self.description()
        return {"run_id": run.run_id, "urdf": urdf}

    def _asset(self, body: dict) -> dict:
        uri = body.get("uri")
        if not isinstance(uri, str):
            raise VerbError("invalid_request", "uri is required")
        return {"data": self.asset(uri)}

    # -- operations ---------------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            status = {
                "host": self.host,
                "state": self._state,
                "run_id": self._run.run_id if self._run else None,
                "spec": dict(self._run.spec) if self._run else None,
                "since": iso(self._since),
                "version": self.version,
            }
            if self._error is not None:
                status["error"] = self._error
        return status

    def catalog(self) -> dict:
        """The image's catalog, fetched once per image id by one caller at a time."""
        unavailable = VerbError(
            "catalog_unavailable", "the image's catalog is unavailable; see the agent log"
        )
        with self._catalog_flight:
            failed_at = self._catalog_failed_at
            if failed_at is not None and time.monotonic() - failed_at < CATALOG_FAILURE_TTL_S:
                raise unavailable
            try:
                image_id = self.runner.image_id()
                with self._lock:
                    if self._catalog is not None and self._catalog[0] == image_id:
                        return self._catalog[1]
                doc = check_catalog(self.runner.catalog())
            except (RunnerError, ValueError) as exc:
                log(f"catalog: {exc}")
                self._catalog_failed_at = time.monotonic()
                raise unavailable from None
            self._catalog_failed_at = None
            with self._lock:
                self._catalog = (image_id, doc)
            return doc

    def _resolve(self, spec: dict) -> tuple[str, ...]:
        """Check each name against the catalog; return the robot's broadcasters."""
        doc = self.catalog()
        entries = {}
        for dimension in DIMENSIONS:
            entry = next((e for e in doc[f"{dimension}s"] if e["name"] == spec[dimension]), None)
            if entry is None:
                valid = ", ".join(sorted(str(e["name"]) for e in doc[f"{dimension}s"]))
                raise VerbError("unknown_entry", f"unknown {dimension}; valid: {valid}")
            if entry["available"] is not True:
                raise VerbError("unavailable_entry", f"{dimension} {spec[dimension]} is not available here")
            entries[dimension] = entry
        try:
            return broadcasters_of(entries["robot"])
        except ValueError as exc:
            log(f"catalog: {exc}")
            raise VerbError(
                "catalog_unavailable", "the robot's catalog entry names no broadcasters"
            ) from None

    def _foreign_run(self) -> str | None:
        """A run another process on this machine holds under this runner's label, if any.

        On a Mac the managed agent and an in-process ``fm sim up --host local``
        share one Docker and one label, and would otherwise both start a run.
        Asked outside the lock, so a slow ``docker ps`` never stalls ``status``.

        tradeoff: a failed listing lets ``up`` go ahead rather than refusing
        every run on a Docker hiccup; ``docker run`` then fails on its own if
        the simulator's ports are taken. The check and the start are not
        atomic: two processes that ask in the same moment can both start.
        """
        with self._lock:
            own = self._run.run_id if self._run is not None else None
        try:
            live = self.runner.active_runs()
        except RunnerError as exc:
            log(f"could not list this label's runs: {exc}")
            return None
        return next((run_id for run_id in live if run_id != own), None)

    def up(self, body: dict) -> str:
        try:
            spec = parse_spec(body)
        except SpecError as exc:
            raise VerbError("invalid_spec", str(exc)) from None
        broadcasters = self._resolve(spec)
        self._check_free_space()
        foreign = self._foreign_run()
        if foreign is not None:
            raise VerbError("busy", f"run {foreign} is active on {self.host}, started by another process")

        started = utc_now()
        run = Run(run_id=run_id_of(spec, started), spec=spec, started_at=started, broadcasters=broadcasters)
        with self._lock:
            if self._run is not None:
                raise VerbError("busy", f"run {self._run.run_id} is active on {self.host}")
            if self.store.exists(run.run_id):
                raise VerbError("run_exists", "a run with this id already exists; retry in a second")
            self._run = run
            self._enter("starting")

        # From here the run is claimed. Whatever fails hands it back, so a
        # broken step can never leave the host busy with a run nobody drives.
        # A failed or timed-out `docker run` may still have created the
        # container, so every failure here stops it as well.
        try:
            with run.write_lock:  # raised, not logged: a run with no record never starts
                self.store.write(self.summary(run))
            overtaken = self._start_container(run)
        except RunnerError as exc:
            log(f"{run.run_id}: start failed: {exc}")
            self._fail(run, "start failed", stop=True)
            raise VerbError("start_failed", "the run did not start; see the agent log") from None
        except Exception:
            self._fail(run, "internal error", stop=True)
            raise
        if overtaken:
            return run.run_id  # no thread is started for a run nobody drives
        try:
            for target, name in (
                (self._pump_logs, "logs"),
                (self._bring_up, "start"),
                (self._expire, "expire"),
            ):
                threading.Thread(
                    target=self._guarded, args=(target, run), name=f"{name}-{run.run_id}", daemon=True
                ).start()
        except Exception:
            self._fail(run, "internal error", stop=True)
            raise
        return run.run_id

    def _start_container(self, run: Run) -> bool:
        """``docker run`` the run; return whether a down overtook it meanwhile.

        A down accepted while ``docker run`` was still going ran its stop before
        the container existed, and ``docker run`` may have created it whether it
        then returns or raises. So on every exit, if the run is no longer
        starting, its container is stopped here, whatever state ``_fail`` would
        otherwise refuse to act in. A down that lands while ``in_docker_run``
        is set skips its own stop, so the container is stopped exactly once.
        """
        with self._lock:
            run.in_docker_run = True
        try:
            self.runner.start(run.run_id, run.spec)
        finally:
            with self._lock:
                run.in_docker_run = False
                overtaken = not (self._run is run and self._state == "starting")
            if overtaken:
                log(f"{run.run_id}: brought down while starting; stopping its container")
                try:
                    self.runner.stop(run.run_id)
                except Exception as exc:  # noqa: BLE001 - must not replace start's own exception
                    log(f"{run.run_id}: stopping failed: {exc}")
        return overtaken

    def _check_free_space(self) -> None:
        try:
            self.store.root.mkdir(parents=True, exist_ok=True)
            free = shutil.disk_usage(self.store.root).free
        except OSError as exc:
            log(f"cannot read free space under {self.store.root}: {exc}")
            raise VerbError("insufficient_space", "cannot read the runs directory's free space") from None
        if free < self.min_free_bytes:
            log(f"{free} bytes free under {self.store.root}; up needs {self.min_free_bytes}")
            raise VerbError("insufficient_space", "the sim host is low on disk space")

    def down(self, run_id: str | None = None, *, expired: bool = False) -> dict:
        """Bring the active run down. ``expired`` marks a down for the run's time limit."""
        with self._lock:
            run = self._run
            if run is None:
                raise VerbError("not_running", f"no run is active on {self.host}")
            if run_id is not None and run_id != run.run_id:
                raise VerbError("run_mismatch", f"the active run is {run.run_id}")
            if self._state == "stopping":
                raise VerbError("busy", f"run {run.run_id} is already stopping")
            self._enter("stopping")
            # Mid-`docker run` there is no container yet; _start_container stops it.
            stop = not run.in_docker_run
        if expired:
            with run.write_lock:
                run.metrics["max_duration_reached"] = True
        try:
            return self._finish(run, query_outcome=True, stop=stop)
        finally:
            run.done.set()
            with self._lock:
                if self._run is run:
                    self._run = None
                    self._enter("idle")

    def reset(self, seed: int) -> None:
        with self._lock:
            run = self._run
            if run is None or self._state != "running":
                raise VerbError("not_running", f"no running run on {self.host} to reset")
        try:
            if not self.runner.has_service(run.run_id, SIM_RESET):
                raise VerbError("reset_unavailable", f"this run serves no {SIM_RESET}")
            self.runner.reset(run.run_id, seed, RESET_TIMEOUT_S)
        except RunnerError as exc:
            log(f"{run.run_id}: reset failed: {exc}")
            raise VerbError("reset_failed", "the reset did not take; see the agent log") from None
        with run.write_lock:
            run.reset_seed = seed

    def description(self) -> tuple[Run, tuple[str, frozenset[str]]]:
        """The active run's URDF and the visual mesh URIs it names."""
        with self._lock:
            run = self._run
            if run is None or self._state not in ("starting", "running"):
                raise VerbError("not_running", f"no run on {self.host} to describe")
        with run.description_lock:
            if run.description is None:
                try:
                    urdf = self.runner.description(run.run_id)
                except RunnerError as exc:
                    log(f"{run.run_id}: description failed: {exc}")
                    raise VerbError("not_running", "the run has not published its description yet") from None
                if len(urdf.encode()) > MAX_DESCRIPTION_BYTES:
                    raise VerbError("internal_error", "the run's description is too large")
                run.description = (urdf, visual_meshes(urdf))
                threading.Thread(
                    target=self._read_meshes, args=(run,), name=f"meshes-{run.run_id}", daemon=True
                ).start()
            return run, run.description

    def _read_meshes(self, run: Run) -> None:
        """Reads every mesh the run's description draws that the cache lacks."""
        try:
            image = self.runner.image_id()
            with self._meshes_lock:
                if image != self._meshes_image:
                    self._meshes, self._meshes_image = {}, image
                wanted = {
                    uri: (match.group(1), match.group(2))
                    for uri in run.description[1]
                    if uri not in self._meshes
                    and (match := PACKAGE_URI.fullmatch(uri)) is not None
                    and ".." not in match.group(2).split("/")
                }
            if not wanted:
                return
            found = self.runner.assets(run.run_id, list(wanted.values()), MAX_MESHES_BYTES)
            missing = [uri for uri, key in wanted.items() if key not in found]
            if missing:
                log(f"{run.run_id}: {len(missing)} meshes are not in the image, e.g. {missing[0]}")
            with self._meshes_lock:
                if image != self._meshes_image:
                    return
                for uri, key in wanted.items():
                    data = found.get(key)
                    held = sum(map(len, self._meshes.values()))
                    if (
                        data is not None
                        and len(data) <= MAX_ASSET_BYTES
                        and held + len(data) <= MAX_MESHES_BYTES
                    ):
                        self._meshes[uri] = data
        except RunnerError as exc:
            log(f"{run.run_id}: reading meshes failed: {exc}")
        except Exception:  # noqa: BLE001 - a background thread's failure is logged, never lost
            log(f"{run.run_id}: reading meshes failed:\n{traceback.format_exc()}")
        finally:
            run.meshes_read.set()

    def asset(self, uri: str) -> bytes:
        """One mesh the active run's description draws."""
        run, (_urdf, meshes) = self.description()
        match = PACKAGE_URI.fullmatch(uri)
        if uri not in meshes or match is None or ".." in match.group(2).split("/"):
            raise VerbError("invalid_request", "not a mesh this run's description draws")
        with self._meshes_lock:
            data = self._meshes.get(uri)
        if data is None and run.meshes_read.wait(MESHES_WAIT_S):
            with self._meshes_lock:
                data = self._meshes.get(uri)
        if data is None:
            raise VerbError("internal_error", "the mesh could not be read; see the agent log")
        return data

    def recover(self) -> list[str]:
        """Clear what a previous agent left: its containers, and its open summaries.

        Run once at start, before serving. A container a dead agent started
        would otherwise hold the simulator while this agent reports idle, and a
        summary it never closed would read as a run still going.

        With ``keep_foreign_runs`` (an agent whose label other processes share,
        as this machine's managed agent shares ``local`` with the in-process
        verbs) nothing is removed while a run is up under the label, since
        that run may be another process's; a listing that fails removes
        nothing either. tradeoff: a run left by a dead agent under a shared
        label then stays until ``fm sim down --host local`` with no managed
        agent running clears it.
        """
        held: list[str] = []
        if self.keep_foreign_runs:
            try:
                held = self.runner.active_runs()
            except RunnerError as exc:
                log(f"could not list this label's runs, so none are cleared: {exc}")
                return []
        if held:
            # Another process on this machine holds a run under the label:
            # removing "stale" containers now would kill it mid-run.
            log(f"leaving {', '.join(held)}: held by another process under this label")
            removed = []
        else:
            try:
                removed = self.runner.stop_stale()
            except RunnerError as exc:
                removed = []
                log(f"could not clear stale runs: {exc}")
        now = iso(utc_now())
        for summary in self.store.unfinished():
            if summary.get("run_id") in held:
                continue  # its holder closes it
            summary["ended_at"] = now
            self.store.write(summary)
        return removed

    # -- lifecycle internals ------------------------------------------------

    def _enter(self, state: str, error: str | None = None) -> None:
        """Move to ``state``. Callers hold ``_lock``."""
        self._state = state
        self._since = utc_now()
        self._error = error

    def _starting(self, run: Run) -> bool:
        with self._lock:
            return self._run is run and self._state == "starting"

    def summary(self, run: Run) -> dict:
        return {
            "run_id": run.run_id,
            "host": self.host,
            "spec": dict(run.spec),
            "started_at": iso(run.started_at),
            "ended_at": iso(run.ended_at) if run.ended_at else None,
            "outcome": run.outcome,
            "mcap": run.mcap,
            "metrics": dict(run.metrics),
        }

    def _ready(self, run: Run) -> bool:
        """Whether controller_manager lists every broadcaster active. Unreachable is not yet."""
        try:
            states = self.runner.list_controllers(run.run_id)
        except RunnerError:
            return False
        return all(states.get(name) == "active" for name in run.broadcasters)

    def _bring_up(self, run: Run) -> None:
        """Wait for the broadcasters, apply the seed, start the recorder."""
        deadline = time.monotonic() + self.startup_timeout_s
        while not self._ready(run):
            if not self._starting(run):
                return  # stopped while starting; down owns the run now
            if time.monotonic() > deadline:
                log(
                    f"{run.run_id}: broadcasters {list(run.broadcasters)} not active after {self.startup_timeout_s:.0f}s"
                )
                self._fail(run, "not ready in time", stop=True)
                return
            time.sleep(self.ready_poll_s)
        # Each step below can take seconds, and a down may land during any of
        # them; whatever a down has already stopped is not started again.
        try:
            if not self._still_starting(run, "seeding"):
                return
            if self.runner.has_service(run.run_id, SIM_RESET):
                self.runner.reset(run.run_id, run.spec["seed"], RESET_TIMEOUT_S)
                applied = True
            else:
                applied = run.spec["seed"] == 0
            with run.write_lock:
                run.metrics["seed_applied"] = applied
            if run.spec["record"]:
                if not self._still_starting(run, "recording"):
                    return
                self.runner.start_recording(run.run_id)
        except RunnerError as exc:
            log(f"{run.run_id}: start failed: {exc}")
            self._fail(run, "start failed", stop=True)
            return
        # Filed before the state flips, so a caller who sees running also finds
        # seed_applied in the summary.
        if not self._still_starting(run, "running"):
            return
        self._write(run)
        with self._lock:
            if self._run is run and self._state == "starting":
                self._enter("running")

    def _still_starting(self, run: Run, step: str) -> bool:
        if self._starting(run):
            return True
        log(f"{run.run_id}: no longer starting; skipped {step}")
        return False

    def _guarded(self, target, run: Run) -> None:
        """A run thread's body. A bug in it finalises the run rather than orphaning it."""
        try:
            target(run)
        except Exception:  # noqa: BLE001 - the run is handed back whatever went wrong
            log(f"{run.run_id}: {target.__name__} failed:\n{traceback.format_exc()}")
            self._fail(run, "internal error", stop=True)

    def _expire(self, run: Run) -> None:
        """Bring the run down once it has been up ``max_run_s``, unless it ended first."""
        if run.done.wait(self.max_run_s):
            return
        log(f"{run.run_id}: up for {self.max_run_s:.0f}s; bringing it down")
        try:
            self.down(run.run_id, expired=True)
        except VerbError:
            pass  # it ended, or is ending, on its own

    def _pump_logs(self, run: Run) -> None:
        """Forward the run's log lines until the container exits, then notice if that was early."""
        window, sent, dropped = time.monotonic(), 0, 0

        def emit(record: dict) -> None:
            try:
                self.log_sink(run.run_id, record)
            except Exception as exc:  # noqa: BLE001 - a failed publish must not end the pump
                log(f"{run.run_id}: log publish failed: {exc}")

        def flush_dropped() -> None:
            if dropped:
                emit({
                    "t": time.time(), "level": "warn", "node": "fm_sim_agent",
                    "msg": f"{dropped} log lines dropped: more than {LOG_MAX_PER_S} a second",
                })  # fmt: skip

        try:
            for line in self.runner.logs(run.run_id):
                now = time.monotonic()
                if now - window >= 1.0:
                    flush_dropped()
                    window, sent, dropped = now, 0, 0
                if sent < LOG_MAX_PER_S:
                    emit(logline.parse(line))
                    sent += 1
                else:
                    dropped += 1
        except RunnerError as exc:
            # Without its log stream the agent cannot see the container exit,
            # so a run it can no longer watch is a run it brings down.
            log(f"{run.run_id}: log stream failed: {exc}")
            self._fail(run, "the run's log stream failed", stop=True)
            return
        flush_dropped()
        self._fail(run, "the run's container exited", stop=True)

    def _fail(self, run: Run, reason: str, *, stop: bool) -> None:
        """Finalise a run that failed on its own. A no-op if ``down`` got there first."""
        with self._lock:
            if self._run is not run or self._state == "stopping":
                return
            self._enter("stopping")
        try:
            self._finish(run, query_outcome=False, stop=stop)
        finally:
            run.done.set()
            with self._lock:
                if self._run is run:
                    self._run = None
                    self._enter("error", reason)

    def _write(self, run: Run) -> dict:
        with run.write_lock:
            summary = self.summary(run)
            try:
                self.store.write(summary)
            except OSError as exc:
                log(f"{run.run_id}: could not write its summary: {exc}")
        return summary

    @staticmethod
    def _own_outcome(run: Run, outcome: dict | None) -> dict | None:
        """The RunSummary outcome, or None when the latched one is another episode's.

        ``/sim/outcome`` keeps the latest verdict: after a reset, a run stopped
        before the new episode ends would otherwise report the previous one.
        Called under ``run.write_lock``.
        """
        if outcome is None:
            return None
        seed = run.spec["seed"] if run.reset_seed is None else run.reset_seed
        if (outcome["scenario"], outcome["seed"]) != (run.spec["scenario"], seed):
            log(
                f"{run.run_id}: latched outcome is for {outcome['scenario']} seed {outcome['seed']},"
                f" not {run.spec['scenario']} seed {seed}; reporting none"
            )
            return None
        return {key: outcome[key] for key in ("success", "reason", "elapsed_s")}

    def _finish(self, run: Run, *, query_outcome: bool, stop: bool = True) -> dict:
        """Collect the outcome, stop the recorder and the run, and write the summary.

        The summary is written whatever fails on the way: it is the one record a
        run always leaves. A failing step is noted in the log, not raised.
        """
        try:
            if query_outcome:
                outcome = self.runner.outcome(run.run_id)  # slow: outside the lock
                with run.write_lock:
                    run.outcome = self._own_outcome(run, outcome)
            if run.spec["record"]:
                try:
                    self.runner.stop_recording(run.run_id)
                except RunnerError as exc:
                    log(f"{run.run_id}: recorder did not stop cleanly: {exc}")
            if stop:
                self.runner.stop(run.run_id)
        except RunnerError as exc:
            log(f"{run.run_id}: stopping failed: {exc}")
        finally:
            with run.write_lock:
                run.ended_at = utc_now()
                run.metrics["wall_s"] = round((run.ended_at - run.started_at).total_seconds(), 3)
            # Written before anything else is looked at, so nothing below can
            # cost the run its record.
            summary = self._write(run)
        if run.spec["record"]:
            recordings = self.store.recordings(run.run_id)
            if recordings:
                with run.write_lock:
                    run.mcap = recordings[0][0]
                    run.metrics["mcap_files"] = len(recordings)
                    run.metrics["mcap_bytes"] = sum(size for _, size in recordings)
                summary = self._write(run)
        return summary
