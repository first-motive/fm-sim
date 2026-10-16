"""What a run is: the RunSpec a caller sends, and the id a run is filed under.

Every name in a RunSpec ends up on a ``docker run`` command line, in a launch
argument and in a directory name on the host. Validation is therefore strict and
total: an unknown key, a wrong type or a name outside ``NAME_PATTERN`` is
refused, never coerced or dropped. Nothing here reaches a shell — the runner
passes argv lists — but a name that cannot be a path component is still refused
here, before it gets anywhere near one.

Caller ``params`` are not accepted in v1: nothing consumes them yet, and every
free-form string on a launch line is surface the agent does not need.
"""

from __future__ import annotations

import re
from datetime import datetime, timezone

#: A catalog entry name and a host segment. Lowercase snake_case: what the ament
#: resource index marker files are called (contracts §1). Matched whole with
#: ``fullmatch``, so a trailing newline is not a name.
NAME_PATTERN = re.compile(r"[a-z][a-z0-9_]*")
NAME_MAX_LEN = 64

#: The four dimensions of a run, in the order the launch and the run id use.
DIMENSIONS = ("backend", "robot", "scenario", "controller")

#: `/sim/reset` takes a uint32 seed (fm_sim_msgs/srv/Reset).
SEED_MAX = 2**32 - 1

SPEC_KEYS = frozenset((*DIMENSIONS, "seed", "record"))

#: What a run id looks like. A caller's `down` names one, and it becomes a
#: directory name, so it is matched whole rather than trusted.
RUN_ID_PATTERN = re.compile(r"[0-9]{8}T[0-9]{6}(?:-[a-z][a-z0-9_]*){4}-[0-9]+")


class SpecError(ValueError):
    """A RunSpec, seed or name that the agent refuses."""


def valid_name(value: object) -> bool:
    """Whether ``value`` is a name that may become a path, argument or key segment."""
    return isinstance(value, str) and len(value) <= NAME_MAX_LEN and NAME_PATTERN.fullmatch(value) is not None


def valid_run_id(value: object) -> bool:
    return (
        isinstance(value, str)
        and len(value) <= 6 * NAME_MAX_LEN
        and RUN_ID_PATTERN.fullmatch(value) is not None
    )


def parse_seed(value: object) -> int:
    """A seed as `/sim/reset` takes it, or raise. ``True`` is not a seed."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise SpecError("seed must be an integer")
    if not 0 <= value <= SEED_MAX:
        raise SpecError(f"seed must be between 0 and {SEED_MAX}")
    return value


def parse_spec(body: object) -> dict:
    """Validate a RunSpec and return a normalised copy, or raise :class:`SpecError`.

    Messages name the field, never echo the caller's value back.
    """
    if not isinstance(body, dict):
        raise SpecError("a RunSpec is a JSON object")
    if set(body) - SPEC_KEYS:
        raise SpecError(f"a RunSpec holds only {', '.join(sorted(SPEC_KEYS))}")

    spec: dict = {}
    for dimension in DIMENSIONS:
        if dimension not in body:
            raise SpecError(f"{dimension} is required")
        if not valid_name(body[dimension]):
            raise SpecError(f"{dimension} must match {NAME_PATTERN.pattern}")
        spec[dimension] = body[dimension]

    if "seed" not in body:
        raise SpecError("seed is required")
    spec["seed"] = parse_seed(body["seed"])

    if "record" not in body:
        raise SpecError("record is required")
    if not isinstance(body["record"], bool):
        raise SpecError("record must be true or false")
    spec["record"] = body["record"]
    return spec


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def iso(moment: datetime) -> str:
    """An ISO 8601 UTC timestamp, the form every time on the wire takes."""
    return moment.astimezone(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def run_id_of(spec: dict, started: datetime) -> str:
    """``<UTC yyyymmddThhmmss>-<backend>-<robot>-<scenario>-<controller>-<seed>``."""
    stamp = started.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S")
    return "-".join([stamp, *(spec[d] for d in DIMENSIONS), str(spec["seed"])])
