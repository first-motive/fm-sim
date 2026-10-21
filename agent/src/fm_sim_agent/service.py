"""The Zenoh session, and nothing else.

Thin on purpose, as fm-robot-agent's service is: it builds the agent, opens a
client-mode session against the fleet router, declares one queryable per verb
under ``fm/sim/<host>/``, publishes status at 1 Hz and log lines as they come,
and hands every query to :meth:`fm_sim_agent.agent.SimAgent.answer`.

Client mode, not peer: every byte crosses the fleet through the router on 7447,
the one port the tailnet ACL opens.

A refusal goes out as a Zenoh error reply carrying the ``{"ok": false, "error"}``
body, as fm-robot-agent sends its refusals: a caller waiting on a verb gets the
reason rather than a timeout, and a reply's kind already says whether it worked.

Only ``status`` is answered on Zenoh's own thread. Every other verb may wait on
Docker for seconds to minutes, and a Zenoh callback must not block the runtime
thread that calls it, so those are answered from two bounded lanes of our own:
one for ``down``, one for everything else (see ``WORK_LANE``).
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import sys
import threading
import traceback
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from fm_sim_agent import bridge
from fm_sim_agent.agent import (
    BINARY_VERBS,
    MAX_BODY_BYTES,
    MAX_RUN_S,
    MIN_FREE_BYTES,
    VERBS,
    SimAgent,
    error_body,
    log,
)
from fm_sim_agent.card import CardError, host_segment, read_host
from fm_sim_agent.env import EndpointError, router_endpoint
from fm_sim_agent.runner import (
    DEFAULT_BAG_SPLIT_BYTES,
    DEFAULT_BAG_SPLIT_S,
    DEFAULT_DDS_RANGE,
    DEFAULT_MEMORY,
    DEFAULT_PIDS_LIMIT,
    DDS_RANGES,
    DockerRunner,
)
from fm_sim_agent.runs import RunStore, default_runs_dir
from fm_sim_agent.spec import NAME_PATTERN, valid_name

KEY_PREFIX = "fm/sim"
STATUS_PERIOD_S = 1.0

#: The image every run starts from, and the one place its pin lives: the
#: service, ``fm sim … --host local`` and ``fm sim host … local`` all read
#: these through :func:`image_ref` and :func:`image_pin`.
DEFAULT_IMAGE = "ghcr.io/first-motive/fm-sim:jazzy"
#: The registry digest ``DEFAULT_IMAGE`` is pinned to (the multi-arch index,
#: amd64 and arm64), or None to follow the tag. CI pushes the tag on every main
#: push; this pin moves only when it is bumped here. Read the tag's current one
#: with ``docker buildx imagetools inspect <DEFAULT_IMAGE>``.
DEFAULT_IMAGE_DIGEST: str | None = "sha256:4b25cf45b890ed1783d3126681c79cddd87a9fd74606996652cb4f37c47a3f97"
#: The private image with First Motive's controllers, which ``fm sim host …
#: local --controllers`` (or ``FM_SIM_CONTROLLERS=1``) runs instead, and its
#: pin, moved as ``DEFAULT_IMAGE_DIGEST`` is. Pulling it needs ``docker login
#: ghcr.io`` with access to it.
CONTROLLERS_IMAGE = "ghcr.io/first-motive/fm-sim-controllers:jazzy"
CONTROLLERS_IMAGE_DIGEST: str | None = (
    "sha256:d62c863eabe790b56798829e93f858838d3dc5df1e1c14b0b84ad73c6c2db5d2"
)
CONTROLLERS_ENV = "FM_SIM_CONTROLLERS"
IMAGE_ENV = "FM_SIM_IMAGE"
DIGEST_PATTERN = re.compile(r"^sha256:[0-9a-f]{64}$")
#: ``[registry[:port]/]path[:tag][@sha256:<64 hex>]``, lower case, never starting
#: with ``-``. The digest form is what fm-setup's unit passes as ``--image``: a
#: host pinned to one image digest runs exactly that image.
IMAGE_PATTERN = re.compile(
    r"^[a-z0-9][a-z0-9._-]*(?::[0-9]+)?(?:/[a-z0-9][a-z0-9._-]*)*(?::[A-Za-z0-9_][A-Za-z0-9_.-]{0,127})?"
    r"(?:@sha256:[0-9a-f]{64})?$"
)
#: The same override fm-setup's sim step reads, to try a digest before pinning it.
IMAGE_DIGEST_ENV = "FM_SIM_IMAGE_DIGEST"

#: Verbs answered on Zenoh's thread. Everything here returns without a container call.
INLINE_VERBS = ("status",)

#: tradeoff: each lane bounds how many queries may be waiting or running at
#: once. Past that the caller is told ``busy`` at once rather than queued
#: behind work it cannot see. ``down`` has a lane of its own, so stopping a run
#: never waits behind a catalog fetch or a slow ``up``; one down at a time is
#: all the agent can act on anyway.
WORK_LANE = ("work", 3, 8)  # name, workers, waiting-or-running ceiling
STOP_LANE = ("stop", 1, 2)
STOP_VERBS = ("down",)
BUSY = error_body("busy", "the agent has too many queries in hand; retry")
TOO_LARGE = error_body("invalid_request", f"body is larger than {MAX_BODY_BYTES} bytes")


class Lane:
    """A pool with a ceiling on what it holds; ``submit`` refuses past it."""

    def __init__(self, name: str, workers: int, ceiling: int) -> None:
        self.pool = ThreadPoolExecutor(max_workers=workers, thread_name_prefix=name)
        self.slots = threading.BoundedSemaphore(ceiling)

    def submit(self, work) -> bool:
        if not self.slots.acquire(blocking=False):
            return False

        def run() -> None:
            try:
                work()
            except Exception:  # noqa: BLE001 - a pool swallows it otherwise, unseen
                log(f"lane worker failed:\n{traceback.format_exc()}")
            finally:
                self.slots.release()

        try:
            self.pool.submit(run)
        except Exception:
            self.slots.release()  # never ran, so it never releases its own slot
            raise
        return True

    def shutdown(self) -> None:
        self.pool.shutdown(wait=False, cancel_futures=True)


class ImageError(ValueError):
    """The image ref or its pin cannot be used; the message says which and why."""


def image_ref(flag: str = "") -> str:
    """The image runs use: ``flag``, else ``$FM_SIM_IMAGE``, else ``DEFAULT_IMAGE``.

    Checked here, so every caller (the service, ``--host local``, ``fm sim
    host … local``) refuses a ref that is not one: it becomes docker argv,
    where a leading ``-`` would read as a flag.
    """
    ref = flag or os.environ.get(IMAGE_ENV, "").strip() or DEFAULT_IMAGE
    if not IMAGE_PATTERN.fullmatch(ref):
        raise ImageError(f"image {ref!r} is not an image reference (--image or ${IMAGE_ENV})")
    return ref


def image_pin(ref: str) -> str | None:
    """The digest ``ref`` is pinned to: ``$FM_SIM_IMAGE_DIGEST``, else its shipped pin.

    ``DEFAULT_IMAGE`` and ``CONTROLLERS_IMAGE`` have one; any other ref (a
    local dev build such as ``fm-sim:jazzy``) has no pin unless the
    environment names one. A ref that already carries a digest is refused:
    here the pin is added to the ref, so the digest belongs in
    ``$FM_SIM_IMAGE_DIGEST``.
    """
    if "@" in ref:
        raise ImageError(f"name the digest in ${IMAGE_DIGEST_ENV}, not in the image reference")
    shipped = {DEFAULT_IMAGE: DEFAULT_IMAGE_DIGEST, CONTROLLERS_IMAGE: CONTROLLERS_IMAGE_DIGEST}
    pin = os.environ.get(IMAGE_DIGEST_ENV, "").strip() or shipped.get(ref)
    if pin is not None and not DIGEST_PATTERN.fullmatch(pin):
        raise ImageError(f"the image pin must be sha256:<64 hex> (${IMAGE_DIGEST_ENV})")
    return pin


def controllers_chosen(flag: bool = False) -> bool:
    """Whether ``--controllers`` or ``$FM_SIM_CONTROLLERS=1`` chose the controllers image."""
    value = os.environ.get(CONTROLLERS_ENV, "").strip()
    if value not in ("", "0", "1"):
        raise ImageError(f"${CONTROLLERS_ENV} must be 1 or 0, not {value!r}")
    return flag or value == "1"


def image_choice(controllers: bool) -> tuple[str, str | None]:
    """The image and pin ``fm sim host … local`` runs: ``CONTROLLERS_IMAGE`` at its
    pin when chosen, else :func:`image_ref` at :func:`image_pin`.

    The choice is the operator's, made for this command, so it outranks
    ``$FM_SIM_IMAGE`` and ``$FM_SIM_IMAGE_DIGEST``, which the CLI warns of.
    """
    if controllers:
        return CONTROLLERS_IMAGE, CONTROLLERS_IMAGE_DIGEST
    ref = image_ref()
    return ref, image_pin(ref)


def repository_of(ref: str) -> str:
    """``ref`` without its tag: ``ghcr.io/first-motive/fm-sim:jazzy`` → ``ghcr.io/first-motive/fm-sim``."""
    head, sep, tail = ref.rpartition(":")
    return head if sep and "/" not in tail else ref


def image_kind(ref: str) -> str:
    """``controllers`` for the controllers image's repository, whatever its tag; else ``public``."""
    return "controllers" if repository_of(ref) == repository_of(CONTROLLERS_IMAGE) else "public"


def key_of(host: str, verb: str) -> str:
    return f"{KEY_PREFIX}/{host}/{verb}"


def encode(body: dict) -> bytes:
    return json.dumps(body, sort_keys=True).encode()


def _reply(query, key: str, body: dict, binary: bool = False) -> None:
    import zenoh

    encoding = zenoh.Encoding.APPLICATION_JSON
    if body.get("ok") and binary:
        query.reply(key, body["data"], encoding=zenoh.Encoding.APPLICATION_OCTET_STREAM)
    elif body.get("ok"):
        query.reply(key, encode(body), encoding=encoding)
    else:
        query.reply_err(encode(body), encoding=encoding)


def _handler(agent: SimAgent, host: str, verb: str, lane: Lane | None):
    """The queryable callback for one verb. ``lane`` is None for an inline verb."""
    key = key_of(host, verb)

    # query is a zenoh.Query; the type is not imported at module scope on purpose.
    def handle(query) -> None:
        # Measured before it is copied out of Zenoh: an oversized body is
        # refused without the agent ever holding it.
        if query.payload is not None and len(query.payload) > MAX_BODY_BYTES:
            _reply(query, key, TOO_LARGE)
            return
        payload = query.payload.to_bytes() if query.payload is not None else None

        def answer() -> None:
            _reply(query, key, agent.answer(verb, payload), binary=verb in BINARY_VERBS)

        if lane is None:
            answer()
        elif not lane.submit(answer):
            _reply(query, key, BUSY)

    return handle


class Fabric:
    """Everything this agent declares on one session, kept alive together."""

    def __init__(self, session, agent: SimAgent, host: str, status_period_s: float = STATUS_PERIOD_S) -> None:
        self.session = session
        self.agent = agent
        self.host = host
        self.stop = threading.Event()
        self.lanes = {"work": Lane(*WORK_LANE), "stop": Lane(*STOP_LANE)}
        self.queryables = [
            session.declare_queryable(key_of(host, verb), _handler(agent, host, verb, self._lane_of(verb)))
            for verb in VERBS
        ]
        self.status_publisher = session.declare_publisher(key_of(host, "status"))
        agent.log_sink = self.publish_log
        self.status_thread = threading.Thread(
            target=self._publish_status, args=(status_period_s,), name="status", daemon=True
        )
        self.status_thread.start()

    def _lane_of(self, verb: str) -> Lane | None:
        if verb in INLINE_VERBS:
            return None
        return self.lanes["stop" if verb in STOP_VERBS else "work"]

    def publish_log(self, run_id: str, record: dict) -> None:
        self.session.put(f"{key_of(self.host, 'logs')}/{run_id}", encode(record))

    def _publish_status(self, period_s: float) -> None:
        while not self.stop.is_set():
            self.status_publisher.put(encode(self.agent.status()))
            self.stop.wait(period_s)

    def close(self) -> None:
        self.stop.set()
        self.status_thread.join(timeout=2)
        for lane in self.lanes.values():
            lane.shutdown()
        for queryable in self.queryables:
            queryable.undeclare()
        self.status_publisher.undeclare()


def _session_config(endpoint: str):
    """A client-mode config pointed at the fleet router.

    Imported here rather than at module scope so ``--help`` works on a host
    without the zenoh wheel.
    """
    import zenoh

    config = zenoh.Config()
    config.insert_json5("mode", json.dumps("client"))
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    return config


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve this sim host's run control verbs over Zenoh.")
    parser.add_argument(
        "--host", default="", help="Serve under this host name instead of the card's (e.g. local)."
    )
    parser.add_argument(
        "--image", default="", help=f"fm-sim image to run (env FM_SIM_IMAGE, default {DEFAULT_IMAGE})."
    )
    parser.add_argument(
        "--runs-dir", type=Path, default=None, help="Where run summaries and bags are written."
    )
    parser.add_argument(
        "--label",
        default="",
        help="fm.sim.agent label on this agent's containers (default: the host name). This machine's "
        "managed agent uses local, the label the in-process `fm sim --host local` verbs use.",
    )
    parser.add_argument("--gpus", action="store_true", help="Give runs every GPU (Isaac).")
    parser.add_argument(
        "--no-host-network", action="store_true", help="Do not share the host's network (macOS)."
    )
    parser.add_argument(
        "--bridge-config",
        type=Path,
        default=None,
        help=f"With --no-host-network: the bridge config each run's bridge reads, its {bridge.CYCLONE_NAME} "
        f"beside it (env {bridge.CONFIG_ENV}, default {bridge.DEFAULT_CONFIG}).",
    )
    parser.add_argument(
        "--docker-arg",
        action="append",
        default=[],
        help="Extra `docker run` argument, repeatable (host specifics).",
    )
    parser.add_argument(
        "--dds-range",
        choices=DDS_RANGES,
        default=DEFAULT_DDS_RANGE,
        help="How far a run's DDS discovery reaches (ROS_AUTOMATIC_DISCOVERY_RANGE). Keep LOCALHOST "
        "on a host that shares a LAN with real robots.",
    )
    parser.add_argument("--pids-limit", type=int, default=DEFAULT_PIDS_LIMIT, help="Process ceiling per run.")
    parser.add_argument(
        "--memory", default=DEFAULT_MEMORY, help="Memory ceiling per run, as docker takes it."
    )
    parser.add_argument(
        "--max-run-minutes", type=float, default=MAX_RUN_S / 60, help="Bring a run down after this long."
    )
    parser.add_argument(
        "--min-free-gb",
        type=float,
        default=MIN_FREE_BYTES / 2**30,
        help="Refuse up below this much free disk.",
    )
    parser.add_argument(
        "--bag-split-seconds",
        type=int,
        default=DEFAULT_BAG_SPLIT_S,
        help="Start a new MCAP file after this long.",
    )
    parser.add_argument(
        "--bag-split-mb",
        type=int,
        default=DEFAULT_BAG_SPLIT_BYTES // 2**20,
        help="Start a new MCAP file past this size.",
    )
    return parser


def bridge_of(args: argparse.Namespace) -> Path | None:
    """The bridge config each run's bridge reads, or None for no bridge; prints why not.

    Only a run off the host network needs one: on the host network the host's
    own sim bridge sees it.
    """
    if not args.no_host_network:
        return None
    config = bridge.active_config(args.bridge_config)
    if config is None:
        absent = bridge.missing(bridge.config_path(args.bridge_config))
        names = ", ".join(str(path) for path in absent)
        print(f"fm-sim-agent: live topics are off: no {names} (fm-comms renders them)", flush=True)
        return None
    print(f"fm-sim-agent: each run's topics are bridged with {config}", flush=True)
    return config


def serve(agent: SimAgent, host: str, endpoint: str, wait) -> None:
    """Serve ``agent`` on a client session to ``endpoint`` until ``wait()`` returns.

    On the way out the queryables and lanes go first, so no new ``up`` can
    land, and then the active run, if any, is brought down: a run outlives no
    agent, so its summary is closed and its container removed.
    """
    import zenoh

    with zenoh.open(_session_config(endpoint)) as session:
        fabric = Fabric(session, agent, host)
        print(f"fm-sim-agent: serving {KEY_PREFIX}/{host}/* via {endpoint}", flush=True)
        try:
            wait()
        finally:
            fabric.close()
            if agent.status()["run_id"] is not None:
                reply = agent.answer("down")
                print(f"fm-sim-agent: down on stop: ok={reply['ok']}", flush=True)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        host = host_segment(args.host) if args.host else read_host()
        endpoint = router_endpoint()
    except (CardError, EndpointError) as exc:
        print(f"fm-sim-agent: {exc}", file=sys.stderr)
        return 1

    label = args.label or host
    if not valid_name(label):
        print(f"fm-sim-agent: --label must match {NAME_PATTERN.pattern}", file=sys.stderr)
        return 1
    try:
        image = image_ref(args.image)
    except ImageError as exc:
        print(f"fm-sim-agent: {exc}", file=sys.stderr)
        return 1
    # Absolute, so the bind mount names the same directory whatever the cwd.
    runs_dir = (args.runs_dir or default_runs_dir()).expanduser().resolve()
    runner = DockerRunner(
        image,
        runs_dir,
        host,
        host_network=not args.no_host_network,
        gpus=args.gpus,
        docker_args=tuple(args.docker_arg),
        pids_limit=args.pids_limit,
        memory=args.memory,
        bag_split_s=args.bag_split_seconds,
        bag_split_bytes=args.bag_split_mb * 2**20,
        dds_range=args.dds_range,
        label=label,
        bridge_config=bridge_of(args),
    )
    agent = SimAgent(host, runner, RunStore(runs_dir))
    agent.keep_foreign_runs = label != host  # a label shared with other processes
    agent.max_run_s = args.max_run_minutes * 60
    agent.min_free_bytes = int(args.min_free_gb * 2**30)
    for removed in agent.recover():
        print(f"fm-sim-agent: removed stale run container {removed}", flush=True)
    print(f"fm-sim-agent: runs use {image}, filed under {runs_dir}", flush=True)

    # Blocked before the session starts its threads, so they inherit the mask
    # and the signal systemd sends on stop reaches sigwait, not a default handler.
    stop_signals = {signal.SIGINT, signal.SIGTERM}
    signal.pthread_sigmask(signal.SIG_BLOCK, stop_signals)
    serve(agent, host, endpoint, wait=lambda: signal.sigwait(stop_signals))
    print("fm-sim-agent: stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
