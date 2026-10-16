"""Seeded trials: one RunSpec run once per seed, one after another, and scored.

A sim host runs one run at a time (contracts §3), so a trial is sequential by
necessity: each run is brought up, given until its outcome is in or its time is
up, and brought down before the next seed starts. ``down`` is what reads the
latched ``/sim/outcome``, so the RunSummary it answers with is the score.

:func:`drive` is that one run, and the local ``fm sim up`` uses it too. It talks
to a host through two methods only (see :class:`Host`), so the suite drives it
against the in-process agent and the fabric alike.

How a run's end is noticed depends on what the host can see. In local mode the
CLI reaches the run's container and peeks at ``/sim/outcome`` while the run is
up, so a run ends as soon as it is scored. A remote host's agent shows the
outcome only at ``down``, so a remote run is given its whole time limit, unless
it ends on its own first.
"""

from __future__ import annotations

import re
import time
from collections.abc import Callable
from typing import Protocol

from fm_sim_agent.spec import SEED_MAX, SpecError, iso, parse_seed, utc_now

REPORT_SCHEMA_VERSION = 1

#: tradeoff: a run gets this long, start-up included, before it is brought down
#: unscored. The agent itself allows 180 s for start-up; a pick on can_on_table
#: takes well under a minute once the arm is up.
DEFAULT_RUN_TIMEOUT_S = 300.0
POLL_S = 1.0

#: More seeds than this in one trial is a typo, not a plan: at minutes a run, a
#: thousand runs is already most of a day on one host.
MAX_SEEDS = 1000

#: How far back ``runs`` is searched for the summary of a run that ended on its own.
SUMMARY_SEARCH_LIMIT = 50

RANGE = re.compile(r"([0-9]+)\.\.([0-9]+)")


class Host(Protocol):
    """What a trial needs of a sim host."""

    def call(self, verb: str, body: dict | None = None) -> dict:
        """One verb's reply body, ``{"ok": true, ...}`` or ``{"ok": false, "error"}``."""

    def outcome(self, run_id: str) -> dict | None:
        """The run's outcome if this host can see it before ``down``, else ``None``."""


class Refused(Exception):
    """The host refused a verb. ``code`` is the agent's error code."""

    def __init__(self, verb: str, reply: dict) -> None:
        error = reply.get("error") if isinstance(reply.get("error"), dict) else {}
        self.code = str(error.get("code", "unknown"))
        self.message = str(error.get("message", "no reason given"))
        super().__init__(f"{verb} refused ({self.code}): {self.message}")


def parse_seeds(text: str) -> list[int]:
    """``0..19``, ``0,3,7`` or a mix (``0..4,10``) as a list of seeds, or raise.

    A range is inclusive at both ends. A seed named twice is refused: it would
    run the same episode twice and weigh it double in the success rate.
    """
    seeds: list[int] = []
    for part in text.split(","):
        part = part.strip()
        bounds = RANGE.fullmatch(part)
        if bounds:
            low, high = int(bounds[1]), int(bounds[2])
            if low > high:
                raise SpecError(f"seed range {part} runs backwards")
            if high - low >= MAX_SEEDS:
                raise SpecError(f"a trial runs at most {MAX_SEEDS} seeds")
            seeds.extend(range(low, high + 1))
        elif part.isdigit():
            seeds.append(int(part))
        else:
            raise SpecError(f"{part!r} is not a seed or a range like 0..19")
    return check_seeds(seeds)


def seed_run(count: int, start: int) -> list[int]:
    """``count`` consecutive seeds from ``start``."""
    if not 1 <= count <= MAX_SEEDS:
        raise SpecError(f"count must be from 1 to {MAX_SEEDS}")
    if start < 0 or start + count - 1 > SEED_MAX:
        raise SpecError(f"seeds must be between 0 and {SEED_MAX}")
    return check_seeds(list(range(start, start + count)))


def check_seeds(seeds: list[int]) -> list[int]:
    if len(seeds) > MAX_SEEDS:
        raise SpecError(f"a trial runs at most {MAX_SEEDS} seeds")
    if len(set(seeds)) != len(seeds):
        raise SpecError("a seed is named more than once")
    return [parse_seed(seed) for seed in seeds]


def _checked(verb: str, reply: dict) -> dict:
    if reply.get("ok") is not True:
        raise Refused(verb, reply)
    return reply


def find_summary(host: Host, run_id: str) -> dict | None:
    """A finished run's summary from the host's ``runs``, or ``None``."""
    reply = host.call("runs", {"limit": SUMMARY_SEARCH_LIMIT})
    if reply.get("ok") is not True:
        return None
    return next((s for s in reply.get("runs", []) if isinstance(s, dict) and s.get("run_id") == run_id), None)


def drive(
    host: Host,
    spec: dict,
    *,
    timeout_s: float | None,
    poll_s: float | None = None,
    on_up: Callable[[str], None] = lambda _run_id: None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict:
    """Run ``spec`` once and return its RunSummary.

    The run ends when its outcome is in (where the host can show it), when it
    ends on its own, or when ``timeout_s`` has passed since ``up``; ``None``
    waits until interrupted. ``poll_s`` is how often the host is asked,
    :data:`POLL_S` unless given. Whatever ends the wait, an interrupt included, the
    run is brought down before this returns or raises, so it never outlives the
    caller. Raises :class:`Refused` if ``up`` is refused.
    """
    poll_s = POLL_S if poll_s is None else poll_s
    run_id = _checked("up", host.call("up", spec))["run_id"]
    on_up(run_id)
    deadline = None if timeout_s is None else clock() + timeout_s
    summary = None
    try:
        while deadline is None or clock() < deadline:
            status = host.call("status")
            if status.get("ok") is True and status.get("run_id") != run_id:
                break  # finalised on its own: failed to start, or its container exited
            if status.get("state") == "running" and host.outcome(run_id) is not None:
                break
            sleep(poll_s)
    finally:
        reply = host.call("down", {"run_id": run_id})
        if reply.get("ok") is True:
            summary = reply["summary"]
    # Not ours to down any more: the run was finalised before the down landed.
    return summary or find_summary(host, run_id) or {"run_id": run_id, "outcome": None, "metrics": {}}


def row_of(seed: int, summary: dict) -> dict:
    """One trial row: a seed, its run, and how it scored."""
    outcome = summary.get("outcome") if isinstance(summary.get("outcome"), dict) else None
    metrics = summary.get("metrics") if isinstance(summary.get("metrics"), dict) else {}
    return {
        "seed": seed,
        "run_id": summary.get("run_id"),
        "success": outcome.get("success") is True if outcome else None,
        "reason": str(outcome.get("reason", "")) if outcome else "no outcome",
        "elapsed_s": outcome.get("elapsed_s") if outcome else None,
        "wall_s": metrics.get("wall_s"),
        "seed_applied": metrics.get("seed_applied"),
    }


def aggregate(host: str, spec: dict, seeds: list[int], rows: list[dict], started_at: str) -> dict:
    """The trial report.

    ``success_rate`` is over every run attempted: a run with no outcome counts
    as a failure, so a host that crashes every run cannot score well by
    scoring nothing. ``unscored`` says how many those were.
    """
    successes = sum(1 for row in rows if row["success"] is True)
    unscored = sum(1 for row in rows if row["success"] is None)
    base = {k: v for k, v in spec.items() if k != "seed"}
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "host": host,
        "spec": base,
        "seeds": seeds,
        "started_at": started_at,
        "ended_at": iso(utc_now()),
        "complete": len(rows) == len(seeds),
        "runs": rows,
        "attempted": len(rows),
        "successes": successes,
        "unscored": unscored,
        "success_rate": round(successes / len(rows), 4) if rows else 0.0,
    }


def run_trial(
    host: Host,
    host_name: str,
    spec: dict,
    seeds: list[int],
    *,
    timeout_s: float = DEFAULT_RUN_TIMEOUT_S,
    poll_s: float | None = None,
    on_row: Callable[[dict], None] = lambda _row: None,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[dict, Exception | None]:
    """Run ``spec`` once per seed, in order; return the report and what stopped it early.

    A refused ``up`` ends the trial: ``busy`` means another run holds the host,
    and a trial does not take a host from someone else. A host that stops
    answering ends it, and so does an interrupt, after the current run is down. Either way the report covers every run
    that finished, and the exception is handed back for the caller to report.
    """
    started_at = iso(utc_now())
    rows: list[dict] = []
    stopped: Exception | None = None
    for seed in seeds:
        try:
            summary = drive(host, {**spec, "seed": seed}, timeout_s=timeout_s, poll_s=poll_s, sleep=sleep)
        except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001 - reported, with the report kept
            stopped = exc
            break
        row = row_of(seed, summary)
        rows.append(row)
        on_row(row)
    return aggregate(host_name, spec, seeds, rows, started_at), stopped
