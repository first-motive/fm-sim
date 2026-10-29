"""What a run is: the RunSpec a caller sends, and the id a run is filed under.

Every name in a RunSpec ends up on a ``docker run`` command line, in a launch
argument and in a directory name on the host. Validation is therefore strict and
total: an unknown key, a wrong type or a name outside ``NAME_PATTERN`` is
refused, never coerced or dropped. Nothing here reaches a shell — the runner
passes argv lists — but a name that cannot be a path component is still refused
here, before it gets anywhere near one.

A RunSpec may carry ``params``: overrides keyed ``<dimension>.<name>``, each a
JSON number, bool, string or array of numbers. Here only their shape is
checked; :mod:`fm_sim_agent.params` checks each against the tunable its catalog
entry declares. No value reaches a command line: the agent writes them to the
run's params files, which the container reads through a read-only mount.
"""

from __future__ import annotations

import math
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

SPEC_KEYS = frozenset((*DIMENSIONS, "seed", "record", "params"))

#: A ``params`` key: ``<dimension>.<tunable name>``.
PARAM_KEY_PATTERN = re.compile(r"(?:backend|robot|scenario|controller)\.[a-z][a-z0-9_]*")
#: tradeoff: bounds on what a caller's params can hold. A run's entries declare
#: tens of tunables; these stop a body that tries to send far more.
MAX_PARAMS = 256
MAX_PARAM_ARRAY = 64
MAX_PARAM_STRING = 64

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

    params = parse_params(body.get("params", {}))
    if params:
        spec["params"] = params
    return spec


def _param_value(value: object) -> bool:
    """Whether ``value`` has a tunable value's shape: a finite number, a bool, a short string,
    or a short list of finite numbers."""
    if isinstance(value, (bool, int)):
        return True  # an int of any size is finite; the catalog's bounds refuse a huge one
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, str):
        return len(value) <= MAX_PARAM_STRING
    if isinstance(value, list):
        return len(value) <= MAX_PARAM_ARRAY and all(
            not isinstance(v, bool) and (isinstance(v, int) or (isinstance(v, float) and math.isfinite(v)))
            for v in value
        )
    return False


def parse_params(params: object) -> dict:
    """A RunSpec's ``params`` by shape alone, sorted by key; ``{}`` when there are none."""
    if not isinstance(params, dict):
        raise SpecError("params must be an object")
    if len(params) > MAX_PARAMS:
        raise SpecError(f"params holds at most {MAX_PARAMS} entries")
    for key, value in params.items():
        if len(key) > 2 * NAME_MAX_LEN or PARAM_KEY_PATTERN.fullmatch(key) is None:
            raise SpecError(f"params keys must match {PARAM_KEY_PATTERN.pattern}")
        if not _param_value(value):
            raise SpecError(f"params.{key} must be a number, a bool, a string or a list of numbers")
    return dict(sorted(params.items()))


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def iso(moment: datetime) -> str:
    """An ISO 8601 UTC timestamp, the form every time on the wire takes."""
    return moment.astimezone(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def run_id_of(spec: dict, started: datetime) -> str:
    """``<UTC yyyymmddThhmmss>-<backend>-<robot>-<scenario>-<controller>-<seed>``."""
    stamp = started.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S")
    return "-".join([stamp, *(spec[d] for d in DIMENSIONS), str(spec["seed"])])
