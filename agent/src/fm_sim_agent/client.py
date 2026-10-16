"""``fm sim`` — the CLI half of the sim agent's verb set.

    fm sim catalog --host fm-ws-01
    fm sim status  --host fm-ws-01
    fm sim up      --host fm-ws-01 --scenario can_on_table --controller fm_skills_gt --seed 3
    fm sim logs    --host fm-ws-01
    fm sim down    --host fm-ws-01
    fm sim runs    --host fm-ws-01 --limit 5
    fm sim trial   --host fm-ws-01 --scenario can_on_table --controller fm_skills_gt --seeds 0..19

A remote host is reached the way fm-robot-agent's client reaches a robot: one
client-mode Zenoh query per verb under ``fm/sim/<host>/``, through the fleet
router. Nothing is parsed or decided here that the agent decides; the client
turns flags into a body and a reply into a line.

``--host local`` runs the agent itself in this process, against this machine's
Docker (OrbStack on a Mac), with no router and no service. The same
:class:`~fm_sim_agent.agent.SimAgent` and
:class:`~fm_sim_agent.runner.DockerRunner` a sim host runs answer every verb,
so a run here is the run a host would start. Because the agent lives only as
long as this process:

- ``up`` stays in the foreground, streams the run's logs, and brings the run
  down on Ctrl-C, on its outcome, or after ``--duration``. A remote ``up``
  returns once the run is started; the host's agent keeps it.
- ``down`` with no run in this process clears what another ``fm sim`` process
  on this machine left: it removes every local run container and closes its
  summary, as an agent does when it starts.
- ``status`` and ``logs`` find a run another local process holds through
  Docker's labels.

Exit codes: 0 success, 1 the host refused, 2 usage, 3 precondition (no router
endpoint, or the host did not answer), 130 interrupted.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import threading
from datetime import datetime, timezone
from pathlib import Path

from fm_sim_agent import logline
from fm_sim_agent.agent import SimAgent, error_body
from fm_sim_agent.card import CardError, host_segment
from fm_sim_agent.env import EndpointError, router_endpoint
from fm_sim_agent.runner import DockerRunner, RunnerError
from fm_sim_agent.runs import RunStore, default_runs_dir
from fm_sim_agent.service import DEFAULT_IMAGE, key_of
from fm_sim_agent.spec import DIMENSIONS, SpecError, valid_run_id
from fm_sim_agent.trial import (
    DEFAULT_RUN_TIMEOUT_S,
    Refused,
    drive,
    parse_seeds,
    run_trial,
    seed_run,
)

LOCAL = "local"

#: How long a remote verb waits for its answer. ``up`` waits on a first
#: catalog fetch (up to 120 s) and ``docker run`` (up to 90 s); ``down`` on the
#: outcome, the recorder and ``docker stop``. Everything else answers at once.
QUERY_TIMEOUT_S = 15.0
VERB_TIMEOUT_S = {"catalog": 150.0, "up": 240.0, "down": 120.0}

#: Every RunSpec field, with the default a flag left out takes. The robot and
#: backend defaults are the ones fm-ros2's old ``sim`` verb used.
SPEC_DEFAULTS = {"backend": "mujoco", "robot": "openarm", "scenario": "empty", "controller": "idle"}

EX_REFUSED = 1
EX_USAGE = 2
EX_PRECONDITION = 3
EX_INTERRUPTED = 130

#: Control characters a reply must not carry into a terminal (as fm-robot-agent's
#: client strips them): the text comes off the fabric, and could otherwise
#: repaint the line an operator is reading.
CONTROL_CHARACTERS = re.compile(r"[\x00-\x08\x0b-\x1f\x7f-\x9f]")


def safe(text: object) -> str:
    return CONTROL_CHARACTERS.sub("", str(text))


class NoAnswer(Exception):
    """The host did not answer a verb."""


class Remote:
    """A sim host's agent, reached through the fleet router."""

    def __init__(self, session, host: str) -> None:
        self.session = session
        self.host = host

    def call(self, verb: str, body: dict | None = None) -> dict:
        payload = json.dumps(body).encode() if body is not None else None
        timeout_s = VERB_TIMEOUT_S.get(verb, QUERY_TIMEOUT_S)
        for reply in self.session.get(key_of(self.host, verb), payload=payload, timeout=timeout_s):
            # A refusal is an error reply carrying the agent's body; read it the
            # same as a success. Zenoh's own error (a timeout) is not JSON.
            sample = reply.ok if reply.ok is not None else reply.err
            try:
                decoded = json.loads(sample.payload.to_bytes())
            except (ValueError, UnicodeDecodeError):
                continue
            if isinstance(decoded, dict):
                return decoded
        raise NoAnswer(f"{self.host} did not answer {verb}")

    def outcome(self, _run_id: str) -> dict | None:
        return None  # the agent shows the outcome only at down

    def follow_logs(self, run_id: str, on_record, stop: threading.Event) -> None:
        """Print the run's log records as the agent publishes them, until ``stop``."""

        def received(sample) -> None:
            try:
                record = json.loads(sample.payload.to_bytes())
            except (ValueError, UnicodeDecodeError):
                return
            if isinstance(record, dict):
                on_record(record)

        subscriber = self.session.declare_subscriber(f"{key_of(self.host, 'logs')}/{run_id}", received)
        try:
            stop.wait()
        finally:
            subscriber.undeclare()

    def close(self) -> None:
        self.session.close()


class Local:
    """The agent itself, in this process, against this machine's Docker.

    ``runner`` is a :class:`DockerRunner` in use, and anything with its
    ``active_runs``, ``outcome`` and ``logs`` in the suite.
    """

    def __init__(self, agent: SimAgent) -> None:
        self.agent = agent
        self.runner = agent.runner
        self.host = agent.host

    def _other_run(self) -> str | None:
        """A run another process on this machine holds, if this process holds none."""
        if self.agent.status()["run_id"] is not None:
            return None
        active = self.runner.active_runs()
        return active[0] if active else None

    def call(self, verb: str, body: dict | None = None) -> dict:
        payload = json.dumps(body).encode() if body is not None else None
        try:
            other = self._other_run() if verb in ("status", "up", "down") else None
        except RunnerError as exc:
            return error_body("internal_error", f"cannot list local runs: {exc}")
        if other is None:
            return self.agent.answer(verb, payload)
        if verb == "status":
            return {**self.agent.answer("status"), "state": "running", "run_id": other}
        if verb == "up":
            return error_body(
                "busy", f"run {other} is active on {self.host}; `fm sim down --host local` ends it"
            )
        removed = self.agent.recover()  # down: the agent's own clean-up of a run nobody drives
        return {"ok": True, "summary": self.agent.store.read(other), "removed": removed}

    def outcome(self, run_id: str) -> dict | None:
        return self.runner.outcome(run_id)

    def follow_logs(self, run_id: str, on_record, _stop: threading.Event) -> None:
        """Print the run's log lines straight from its container, until it exits."""
        for line in self.runner.logs(run_id):
            on_record(logline.parse(line))

    def close(self) -> None:
        """Bring down a run this process still holds: none outlives its agent.

        Reached on every way out of ``main``, an interrupt mid-``up`` included,
        where :func:`~fm_sim_agent.trial.drive` has no run id to down yet.
        """
        if self.agent.status()["run_id"] is not None:
            self.agent.answer("down")


def local_host(image: str, runs_dir: Path | None) -> Local:
    """The in-process agent ``--host local`` drives. Never shares the host's network."""
    root = (runs_dir or default_runs_dir()).expanduser().resolve()
    runner = DockerRunner(image, root, LOCAL, host_network=False)
    return Local(SimAgent(LOCAL, runner, RunStore(root)))


def remote_host(host: str) -> Remote:
    """A client-mode session against this machine's router, aimed at ``host``."""
    import zenoh

    config = zenoh.Config()
    config.insert_json5("mode", json.dumps("client"))
    config.insert_json5("connect/endpoints", json.dumps([router_endpoint()]))
    return Remote(zenoh.open(config), host)


def connect(args: argparse.Namespace) -> Local | Remote:
    if args.host == LOCAL:
        image = args.image or os.environ.get("FM_SIM_IMAGE", "").strip() or DEFAULT_IMAGE
        return local_host(image, args.runs_dir)
    return remote_host(args.host)


# -- the command line --------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--host", required=True, help="the sim host's machine name (fm-ws-01), or `local` for this machine"
    )
    common.add_argument("--json", action="store_true", dest="as_json", help="print the raw reply")
    common.add_argument(
        "--image",
        default="",
        help=f"local only: the image runs use (env FM_SIM_IMAGE, default {DEFAULT_IMAGE})",
    )
    common.add_argument("--runs-dir", type=Path, default=None, help="local only: where runs are filed")

    run = argparse.ArgumentParser(add_help=False)
    for dimension in DIMENSIONS:
        run.add_argument(
            f"--{dimension}",
            default=SPEC_DEFAULTS[dimension],
            help=f"catalog {dimension} (default %(default)s)",
        )
    run.add_argument("--record", action="store_true", help="record an MCAP of the run")

    parser = argparse.ArgumentParser(prog="fm sim", description="Drive a First Motive sim host.")
    verbs = parser.add_subparsers(dest="verb", required=True, metavar="verb")
    verbs.add_parser("catalog", parents=[common], help="what the host can run")
    verbs.add_parser("status", parents=[common], help="the host's state and active run")
    up = verbs.add_parser("up", parents=[common, run], help="start one run")
    up.add_argument("--seed", type=int, default=0, help="scenario seed (default 0)")
    up.add_argument(
        "--duration",
        type=float,
        default=None,
        help="local only: bring the run down after this many seconds (default: on outcome or Ctrl-C)",
    )
    down = verbs.add_parser("down", parents=[common], help="stop the active run")
    down.add_argument("--run-id", default=None, help="refuse unless this is the active run")
    runs = verbs.add_parser("runs", parents=[common], help="recent run summaries, newest first")
    runs.add_argument("--limit", type=int, default=20)
    logs = verbs.add_parser("logs", parents=[common], help="follow a run's log lines")
    logs.add_argument("--run-id", default=None, help="the run to follow (default: the active run)")
    trial = verbs.add_parser("trial", parents=[common, run], help="one run per seed, scored")
    seeds = trial.add_mutually_exclusive_group(required=True)
    seeds.add_argument("--seeds", help="seeds to run: 0..19, 0,3,7, or a mix")
    seeds.add_argument("--count", type=int, help="run this many consecutive seeds from --seed-start")
    trial.add_argument("--seed-start", type=int, default=0, help="first seed for --count (default 0)")
    trial.add_argument(
        "--run-timeout",
        type=float,
        default=DEFAULT_RUN_TIMEOUT_S,
        help="seconds a run gets, start-up included, before it is downed unscored (default %(default)s)",
    )
    trial.add_argument("--report", type=Path, default=None, help="where the JSON report goes")
    return parser


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse and check every flag, so nothing below reaches a host on a bad one."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.host != LOCAL:
        try:
            args.host = host_segment(args.host)
        except CardError as exc:
            parser.error(str(exc))
        if args.image or args.runs_dir or getattr(args, "duration", None) is not None:
            parser.error("--image, --runs-dir and --duration apply only to --host local")
    if getattr(args, "run_id", None) is not None and not valid_run_id(args.run_id):
        parser.error(f"{args.run_id!r} is not a run id")
    if args.verb == "trial":
        try:
            args.seed_list = parse_seeds(args.seeds) if args.seeds else seed_run(args.count, args.seed_start)
        except SpecError as exc:
            parser.error(str(exc))
        if args.run_timeout <= 0:
            parser.error("--run-timeout must be positive")
    if args.verb == "up" and args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    return args


def spec_of(args: argparse.Namespace, seed: int) -> dict:
    return {**{d: getattr(args, d) for d in DIMENSIONS}, "seed": seed, "record": args.record}


def default_report_path(spec: dict) -> Path:
    """``$XDG_STATE_HOME/fm-sim/trials/<UTC stamp>-<backend>-<robot>-<scenario>-<controller>.json``."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    name = "-".join([stamp, *(spec[d] for d in DIMENSIONS)])
    return default_runs_dir().parent / "trials" / f"{name}.json"


# -- rendering ---------------------------------------------------------------


def emit(reply: dict, as_json: bool, line) -> int:
    """Print a reply as JSON or as ``line(reply)``; a refusal goes to stderr."""
    if as_json:
        print(json.dumps(reply, indent=2, sort_keys=True))
    elif reply.get("ok") is not True:
        error = reply.get("error") if isinstance(reply.get("error"), dict) else {}
        print(
            f"refused ({safe(error.get('code', '?'))}): {safe(error.get('message', 'no reason given'))}",
            file=sys.stderr,
        )
    else:
        print(safe(line(reply)))
    return 0 if reply.get("ok") is True else EX_REFUSED


def status_line(status: dict) -> str:
    run = f" {status['run_id']}" if status.get("run_id") else ""
    error = f" ({status['error']})" if status.get("error") else ""
    return f"{status.get('host')}: {status.get('state')}{run}{error} since {status.get('since')}"


def catalog_lines(reply: dict) -> str:
    catalog = reply.get("catalog", {})
    lines = []
    for dimension in DIMENSIONS:
        names = [
            f"{e.get('name')}{'' if e.get('available') is True else ' (unavailable)'}"
            for e in catalog.get(f"{dimension}s", [])
        ]
        lines.append(f"{dimension}s: {', '.join(names) or '-'}")
    return "\n".join(lines)


def outcome_text(summary: dict | None) -> str:
    outcome = (summary or {}).get("outcome")
    if not isinstance(outcome, dict):
        return "no outcome"
    verdict = "success" if outcome.get("success") is True else "failure"
    return f"{verdict}: {outcome.get('reason', '')} ({outcome.get('elapsed_s')} s)"


def summary_line(summary: dict | None) -> str:
    if not summary:
        return "no summary filed"
    return f"{summary.get('run_id')}: {outcome_text(summary)}"


def runs_lines(reply: dict) -> str:
    return "\n".join(summary_line(s) for s in reply.get("runs", [])) or "no runs"


def down_line(reply: dict) -> str:
    removed = reply.get("removed")
    cleared = f"\nremoved {', '.join(removed)}" if removed else ""
    return summary_line(reply.get("summary")) + cleared


def print_record(record: dict) -> None:
    try:
        stamp = datetime.fromtimestamp(float(record.get("t", 0)), timezone.utc).strftime("%H:%M:%S")
    except (TypeError, ValueError, OverflowError, OSError):
        stamp = "--:--:--"
    print(safe(f"{stamp} [{record.get('level')}] {record.get('node')}: {record.get('msg')}"), flush=True)


TABLE_COLUMNS = ("seed", "result", "reason", "elapsed_s", "run_id")


def table_row(row: dict) -> tuple[str, ...]:
    result = {True: "success", False: "failure", None: "unscored"}[row["success"]]
    elapsed = "" if row["elapsed_s"] is None else f"{row['elapsed_s']:.1f}"
    return (str(row["seed"]), result, safe(row["reason"])[:40], elapsed, str(row["run_id"]))


def trial_table(report: dict) -> str:
    rows = [TABLE_COLUMNS, *(table_row(row) for row in report["runs"])]
    widths = [max(len(row[i]) for row in rows) for i in range(len(TABLE_COLUMNS))]
    lines = ["  ".join(cell.ljust(widths[i]) for i, cell in enumerate(row)).rstrip() for row in rows]
    rate = report["success_rate"] * 100
    lines.append(
        f"success {report['successes']}/{report['attempted']} ({rate:.1f}%), {report['unscored']} unscored"
        + (
            ""
            if report["complete"]
            else f"; stopped after {report['attempted']} of {len(report['seeds'])} seeds"
        )
    )
    return "\n".join(lines)


# -- verbs -------------------------------------------------------------------


def cmd_up(host, args) -> int:
    spec = spec_of(args, args.seed)
    if not isinstance(host, Local):
        return emit(host.call("up", spec), args.as_json, lambda r: f"started {r['run_id']} on {args.host}")
    # Local: this process is the agent, so the run lives in the foreground.
    host.agent.log_sink = lambda _run_id, record: print_record(record)
    try:
        summary = drive(
            host, spec, timeout_s=args.duration, on_up=lambda r: print(f"started {r}", flush=True)
        )
    except Refused as exc:
        print(f"refused ({exc.code}): {safe(exc.message)}", file=sys.stderr)
        return EX_REFUSED
    except KeyboardInterrupt:
        print("interrupted; the run was brought down", file=sys.stderr)
        return EX_INTERRUPTED
    return emit({"ok": True, "summary": summary}, args.as_json, lambda r: summary_line(r["summary"]))


def cmd_logs(host, args) -> int:
    run_id = args.run_id
    if run_id is None:
        status = host.call("status")
        run_id = status.get("run_id")
        if not run_id:
            return emit(error_body("not_running", f"no run is active on {args.host}"), args.as_json, str)
    print(f"following {run_id}; Ctrl-C stops", file=sys.stderr, flush=True)
    stop = threading.Event()
    try:
        host.follow_logs(run_id, print_record, stop)
    except RunnerError as exc:
        print(f"fm sim: cannot follow {run_id}: {exc}", file=sys.stderr)
        return EX_PRECONDITION
    except KeyboardInterrupt:
        stop.set()
    return 0


def cmd_trial(host, args) -> int:
    spec = spec_of(args, 0)
    seeds = args.seed_list
    print(f"trial: {len(seeds)} runs of {'/'.join(spec[d] for d in DIMENSIONS)} on {args.host}", flush=True)

    def progress(row: dict) -> None:
        print(safe("  ".join(table_row(row))), flush=True)

    report, stopped = run_trial(host, args.host, spec, seeds, timeout_s=args.run_timeout, on_row=progress)
    path = (args.report or default_report_path(spec)).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.as_json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(trial_table(report))
    print(f"report: {path}")
    if isinstance(stopped, KeyboardInterrupt):
        print("interrupted; the active run was brought down", file=sys.stderr)
        return EX_INTERRUPTED
    if stopped is not None:
        print(f"fm sim: {safe(stopped)}", file=sys.stderr)
        return EX_PRECONDITION if isinstance(stopped, NoAnswer) else EX_REFUSED
    return 0


def run_verb(host, args) -> int:
    if args.verb == "catalog":
        return emit(host.call("catalog"), args.as_json, catalog_lines)
    if args.verb == "status":
        return emit(host.call("status"), args.as_json, status_line)
    if args.verb == "runs":
        return emit(host.call("runs", {"limit": args.limit}), args.as_json, runs_lines)
    if args.verb == "down":
        body = {"run_id": args.run_id} if args.run_id else {}
        return emit(host.call("down", body), args.as_json, down_line)
    if args.verb == "up":
        return cmd_up(host, args)
    if args.verb == "logs":
        return cmd_logs(host, args)
    if args.verb == "trial":
        return cmd_trial(host, args)
    raise ValueError(f"unknown verb {args.verb}")  # argparse's choices make this unreachable


def main(argv: list[str] | None = None, connect=connect) -> int:
    """``connect`` builds the host from the parsed flags; the suite passes its own."""
    args = parse_args(argv)
    try:
        host = connect(args)
    except EndpointError as exc:
        print(f"fm sim: {exc}", file=sys.stderr)
        return EX_PRECONDITION
    try:
        return run_verb(host, args)
    except NoAnswer as exc:
        print(f"fm sim: {exc}", file=sys.stderr)
        return EX_PRECONDITION
    finally:
        host.close()


if __name__ == "__main__":
    raise SystemExit(main())
