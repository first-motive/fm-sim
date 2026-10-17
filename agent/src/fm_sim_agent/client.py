"""``fm sim`` — the CLI half of the sim agent's verb set.

    fm sim catalog --host fm-ws-01
    fm sim status  --host fm-ws-01
    fm sim up      --host fm-ws-01 --scenario can_on_table --controller fm_skills_gt --seed 3
    fm sim logs    --host fm-ws-01
    fm sim down    --host fm-ws-01
    fm sim runs    --host fm-ws-01 --limit 5
    fm sim trial   --host fm-ws-01 --scenario can_on_table --controller fm_skills_gt --seeds 0..19
    fm sim host start aws --stack fm-sim-cloud-host --region eu-west-1

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
- ``up`` and ``trial`` first ask Docker whether another ``fm sim`` process on
  this machine holds a run under the ``local`` label, and refuse with ``busy``
  if one does. The check and the start are not atomic: two processes started
  in the same moment can both pass it, and the second run then fails on the
  simulator's ports rather than with ``busy``.
- ``down`` with no run in this process clears what another ``fm sim`` process
  on this machine left: it removes every local run container and closes its
  summary, as an agent does when it starts. With ``--run-id`` it does so only
  if that is the run it finds.
- ``status`` and ``logs`` find a run another local process holds through
  Docker's labels.
- ``up --duration`` is checked between polls, and a poll can wait on the
  container's ``/sim/outcome`` for up to its 5 s timeout, so a run can
  outlast its duration by that much.

``host start|stop|status aws`` starts, stops or reads the AWS sim host stack
through the ``aws`` CLI (see :mod:`fm_sim_agent.cloud`). ``running`` there means
the instance is in service, not that its agent answers: ``fm sim status --host
<HostName>`` says that.

Exit codes: 0 success, 1 the host refused, 2 usage, 3 precondition (no router
endpoint, or the host did not answer; for ``host``, AWS could not be asked or
answered outside the stack's contract), 130 interrupted.
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

from fm_sim_agent import cloud, logline
from fm_sim_agent.agent import SimAgent, error_body
from fm_sim_agent.card import CardError, host_segment
from fm_sim_agent.env import EndpointError, router_endpoint
from fm_sim_agent.runner import DockerRunner, RunnerError
from fm_sim_agent.runs import RunStore, default_runs_dir
from fm_sim_agent.service import DEFAULT_IMAGE, key_of
from fm_sim_agent.spec import DIMENSIONS, NAME_PATTERN, SpecError, valid_name, valid_run_id
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

#: What a field off the fabric must not carry into a terminal: control
#: characters, newlines and tabs included (every field prints on one line), and
#: the bidi overrides and isolates. The text comes from a host, and could
#: otherwise repaint or reorder the line an operator is reading.
UNSAFE_CHARACTERS = re.compile("[\x00-\x1f\x7f-\x9f\u202a-\u202e\u2066-\u2069]")

#: A reply or log record is a few kilobytes at most. One past this is not
#: decoded: a host, or anything speaking as one, must not spend this machine's
#: memory.
MAX_PAYLOAD_BYTES = 1 << 20


def safe(text: object) -> str:
    """One field as a single safe line."""
    return UNSAFE_CHARACTERS.sub("", str(text))


def decode(payload) -> dict | None:
    """A Zenoh payload as a JSON object, or ``None`` if it is oversized or not one."""
    if payload is None or len(payload) > MAX_PAYLOAD_BYTES:
        return None
    try:
        decoded = json.loads(payload.to_bytes())
    except (ValueError, UnicodeDecodeError, RecursionError):
        return None
    return decoded if isinstance(decoded, dict) else None


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
            decoded = decode(sample.payload)
            if decoded is not None:
                return decoded
        raise NoAnswer(f"{self.host} did not answer {verb}")

    def outcome(self, _run_id: str) -> dict | None:
        return None  # the agent shows the outcome only at down

    def follow_logs(self, run_id: str, on_record, stop: threading.Event) -> None:
        """Print the run's log records as the agent publishes them, until ``stop``."""

        def received(sample) -> None:
            record = decode(sample.payload)
            if record is not None:
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
        wanted = (body or {}).get("run_id")
        if wanted is not None and wanted != other:
            return error_body("run_mismatch", f"the active run is {other}")
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
    host = verbs.add_parser("host", help="start, stop or read a cloud sim host")
    host.add_argument("action", choices=cloud.ACTIONS)
    host.add_argument("provider", choices=cloud.PROVIDERS)
    host.add_argument("--stack", default=None, help=f"CloudFormation stack (env {cloud.STACK_ENV})")
    host.add_argument(
        "--region", default=None, help=f"AWS region (env {cloud.REGION_ENV}, else the aws default)"
    )
    host.add_argument("--profile", default=None, help="AWS CLI profile (default: the aws default)")
    host.add_argument("--json", action="store_true", dest="as_json", help="print the checked reply")
    return parser


def cloud_target(args: argparse.Namespace) -> cloud.Target:
    """The stack ``host`` drives, from its flags, else the environment."""
    region = args.region or os.environ.get(cloud.REGION_ENV, "").strip() or None
    stack = args.stack or os.environ.get(cloud.STACK_ENV, "").strip()
    return cloud.Target(stack=stack, region=region, profile=args.profile or None)


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse and check every flag, so nothing below reaches a host on a bad one."""
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.verb == "host":
        args.target = cloud_target(args)
        if not args.target.stack:
            parser.error(f"host needs --stack or ${cloud.STACK_ENV}")
        problem = cloud.target_problem(args.target)
        if problem:
            parser.error(problem)
        return args
    if args.host != LOCAL:
        try:
            args.host = host_segment(args.host)
        except CardError as exc:
            parser.error(str(exc))
        if args.image or args.runs_dir or getattr(args, "duration", None) is not None:
            parser.error("--image, --runs-dir and --duration apply only to --host local")
    if getattr(args, "run_id", None) is not None and not valid_run_id(args.run_id):
        parser.error(f"{args.run_id!r} is not a run id")
    if args.verb in ("up", "trial"):
        for dimension in DIMENSIONS:
            # Checked here as well as by the agent: a trial's report is named after them.
            if not valid_name(getattr(args, dimension)):
                parser.error(f"--{dimension} must match {NAME_PATTERN.pattern}")
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
        print(line(reply))  # each renderer makes its own fields safe
    return 0 if reply.get("ok") is True else EX_REFUSED


def status_line(status: dict) -> str:
    run = f" {safe(status['run_id'])}" if status.get("run_id") else ""
    error = f" ({safe(status['error'])})" if status.get("error") else ""
    return f"{safe(status.get('host'))}: {safe(status.get('state'))}{run}{error} since {safe(status.get('since'))}"


def catalog_lines(reply: dict) -> str:
    catalog = reply.get("catalog", {})
    lines = []
    for dimension in DIMENSIONS:
        names = [
            f"{safe(e.get('name'))}{'' if e.get('available') is True else ' (unavailable)'}"
            for e in catalog.get(f"{dimension}s", [])
        ]
        lines.append(f"{dimension}s: {', '.join(names) or '-'}")
    return "\n".join(lines)


def outcome_text(summary: dict | None) -> str:
    outcome = (summary or {}).get("outcome")
    if not isinstance(outcome, dict):
        return "no outcome"
    verdict = "success" if outcome.get("success") is True else "failure"
    return f"{verdict}: {safe(outcome.get('reason', ''))} ({safe(outcome.get('elapsed_s'))} s)"


def summary_line(summary: dict | None) -> str:
    if not summary:
        return "no summary filed"
    return f"{safe(summary.get('run_id'))}: {outcome_text(summary)}"


def runs_lines(reply: dict) -> str:
    return "\n".join(summary_line(s) for s in reply.get("runs", [])) or "no runs"


def down_line(reply: dict) -> str:
    removed = reply.get("removed")
    cleared = f"\nremoved {', '.join(safe(name) for name in removed)}" if removed else ""
    return summary_line(reply.get("summary")) + cleared


def print_record(record: dict) -> None:
    try:
        stamp = datetime.fromtimestamp(float(record.get("t", 0)), timezone.utc).strftime("%H:%M:%S")
    except (TypeError, ValueError, OverflowError, OSError):
        stamp = "--:--:--"
    print(
        f"{stamp} [{safe(record.get('level'))}] {safe(record.get('node'))}: {safe(record.get('msg'))}",
        flush=True,
    )


TABLE_COLUMNS = ("seed", "result", "reason", "elapsed_s", "run_id")


def table_row(row: dict) -> tuple[str, ...]:
    result = {True: "success", False: "failure", None: "unscored"}[row["success"]]
    elapsed_s = row["elapsed_s"]
    elapsed = f"{elapsed_s:.1f}" if isinstance(elapsed_s, (int, float)) else safe(elapsed_s or "")
    return (str(row["seed"]), result, safe(row["reason"])[:40], elapsed, safe(row["run_id"]))


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


def money(amount: object) -> str:
    return f"${amount:.2f}" if isinstance(amount, (int, float)) else "?"


def host_lines(reply: dict, outputs: cloud.Outputs | None, target: cloud.Target) -> str:
    """A checked Status as lines; ``reply_problem`` has already held it to the schema."""
    status = reply["status"]
    region = f" ({target.region})" if target.region else ""
    head = f"stack {target.stack}{region}: {status['state']}"
    if status["state"] == "unknown":
        return head + " (the controller has not reconciled yet)"
    lines = [f"{head}, desired {status['desired_capacity']}, updated {safe(status.get('updated_at'))}"]
    for instance in status["instances"]:
        lines.append(
            f"  {safe(instance['instance_id'])} {safe(instance.get('lifecycle_state'))},"
            f" launched {safe(instance.get('launched_at'))}, terminate after {safe(instance.get('terminate_after'))}"
        )
    budget = status["budget"]
    gate = f"blocked ({budget['reason']})" if budget["blocked"] else budget["reason"]
    lines.append(
        f"  budget {safe(budget.get('name'))}: {money(budget.get('actual_usd'))} of"
        f" {money(budget.get('limit_usd'))}, blocks at {safe(budget.get('block_percent'))}%, {gate}"
    )
    if status["state"] == "running":
        host = safe(outputs.host_name) if outputs and outputs.host_name else "<HostName>"
        lines.append(
            f"  running means in service; `fm sim status --host {host}` says when the agent is ready"
        )
    return "\n".join(lines)


def cmd_host(args: argparse.Namespace) -> int:
    target = args.target
    stack = {"name": target.stack, "region": target.region, "host_name": None}
    try:
        reply, outputs = cloud.run_action(cloud.Aws(target), args.action)
    except cloud.AwsError as exc:
        if args.as_json:
            error = {"code": safe(exc.code), "message": safe(exc.message)}
            print(json.dumps({"ok": False, "error": error, "stack": stack}, indent=2, sort_keys=True))
        print(f"fm sim: {safe(exc.code)}: {safe(exc.message)}", file=sys.stderr)
        return EX_PRECONDITION
    stack["host_name"] = safe(outputs.host_name) if outputs.host_name else None
    if args.as_json:
        print(json.dumps({**reply, "stack": stack}, indent=2, sort_keys=True))
        return 0 if reply["ok"] else EX_REFUSED
    if "status" in reply:
        print(host_lines(reply, outputs, target))
    return 0 if reply["ok"] else emit(reply, False, str)  # a refusal prints its code to stderr


# -- verbs -------------------------------------------------------------------


def cmd_up(host, args) -> int:
    spec = spec_of(args, args.seed)
    if not isinstance(host, Local):
        return emit(
            host.call("up", spec), args.as_json, lambda r: f"started {safe(r['run_id'])} on {args.host}"
        )
    # Local: this process is the agent, so the run lives in the foreground.
    host.agent.log_sink = lambda _run_id, record: print_record(record)
    try:
        summary = drive(
            host, spec, timeout_s=args.duration, on_up=lambda r: print(f"started {safe(r)}", flush=True)
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
        if not valid_run_id(run_id):
            # It becomes a key expression, where `*` would widen it over every run.
            return emit(
                error_body("invalid_reply", "the host named a run id that is not one"), args.as_json, str
            )
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
        print("  ".join(table_row(row)), flush=True)

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
    if args.verb == "host":
        return cmd_host(args)
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
