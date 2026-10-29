"""A RunSpec's ``params``: overrides of the tunables the run's catalog entries declare.

A caller names a tunable ``<dimension>.<name>``; the entry of that dimension
declares it under ``params.<name>`` with its type, bounds and target. Only a
declared tunable is accepted, so every node and param name that reaches the
run's params file comes from the image's catalog, never from the caller.

:func:`check_value` mirrors ``fm_sim_bringup.params.check_value``, which the
launch runs again on the same values: the agent runs on the host and cannot
import it. Both suites check their copy against
``fm_sim_bringup/test/param_cases.json``.

Messages name the param, never echo the caller's value back.
"""

from __future__ import annotations

import math
import re

from fm_sim_agent.spec import SpecError

#: What the catalog schema allows a ros_param target to name. Checked here as
#: well, so a malformed catalog cannot put any other key into the params file.
NODE_PATTERN = re.compile(r"(/[A-Za-z][A-Za-z0-9_]*)+")
PARAM_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*")
TARGET_KINDS = ("ros_param", "entry", "description")


def _number(value: object, kind: str) -> bool:
    """Whether ``value`` is a JSON number of ``kind``: ``True`` is neither; an int is a float.

    An int of any size is finite; ``math.isfinite`` would overflow on one past
    the float range, and the bounds refuse it instead.
    """
    if isinstance(value, bool):
        return False
    if isinstance(value, int):
        return True
    return kind == "float" and isinstance(value, float) and math.isfinite(value)


def check_value(param: dict, value: object) -> object:
    """``value`` normalised for ``param``, or raise :class:`SpecError` saying what it must be.

    A float tunable's value comes back a float even when sent as an int: ROS
    refuses an integer for a double parameter.
    """
    kind = param.get("type")
    if kind == "bool":
        if not isinstance(value, bool):
            raise SpecError("must be true or false")
        return value
    if kind == "enum":
        members = param.get("enum") or []
        if not isinstance(value, str) or value not in members:
            raise SpecError(f"must be one of {', '.join(map(str, members))}")
        return value
    low, high = param.get("min"), param.get("max")
    if not (_number(low, "float") and _number(high, "float")):
        raise SpecError("has no bounds in the catalog")
    if kind == "array":
        items, length = param.get("items"), param.get("length")
        if not isinstance(value, list) or len(value) != length or not all(_number(v, items) for v in value):
            raise SpecError(f"must be a list of {length} {items} values")
        if not all(low <= v <= high for v in value):
            raise SpecError(f"each element must be between {low} and {high}")
        if param.get("ordered") is True and any(a > b for a, b in zip(value, value[1:])):
            raise SpecError("must be in ascending order")
        return [float(v) if items == "float" else v for v in value]
    if kind not in ("float", "int"):
        raise SpecError("has no known type in the catalog")
    if not _number(value, kind):
        raise SpecError("must be an integer" if kind == "int" else "must be a number")
    if not low <= value <= high:
        raise SpecError(f"must be between {low} and {high}")
    return float(value) if kind == "float" else value


def declared(entries: dict, key: str) -> dict:
    """The tunable ``key`` (``<dimension>.<name>``) names in ``entries``, or raise."""
    dimension, name = key.split(".", 1)
    tunables = entries[dimension].get("params")
    param = tunables.get(name) if isinstance(tunables, dict) else None
    target = param.get("target") if isinstance(param, dict) else None
    if (
        not isinstance(target, dict)
        or target.get("kind") not in TARGET_KINDS
        or (
            target["kind"] == "ros_param"
            and not (
                isinstance(target.get("node"), str)
                and NODE_PATTERN.fullmatch(target["node"])
                and isinstance(target.get("param"), str)
                and PARAM_PATTERN.fullmatch(target["param"])
            )
        )
    ):
        raise SpecError(f"params.{key} is not a tunable of {dimension} {entries[dimension]['name']}")
    return param


def check(params: dict, entries: dict) -> dict:
    """``params`` checked against the four resolved catalog entries, values normalised.

    Refuses a key no entry declares, a value its tunable does not take, and a
    tunable whose ``backends`` leave out the run's backend.
    """
    checked = {}
    backend = entries["backend"]["name"]
    for key, value in params.items():
        param = declared(entries, key)
        backends = param.get("backends")
        if isinstance(backends, list) and backend not in backends:
            raise SpecError(f"params.{key} applies only on {', '.join(map(str, backends))}")
        try:
            checked[key] = check_value(param, value)
        except SpecError as exc:
            raise SpecError(f"params.{key} {exc}") from None
    return checked


def ros_params(params: dict, entries: dict) -> dict:
    """The ROS params file for the ``ros_param`` targets among checked ``params``.

    Keyed by the fully qualified node name, then ``ros__parameters``, with a
    dotted param nested one map per segment. Node and param names come from
    the catalog's declarations only. Empty when no override targets a node.
    Refuses two overrides that set one ROS param, or one inside another, as
    two entries' tunables can.
    """
    document: dict = {}
    written: dict[str, tuple[str, list[str]]] = {}
    for key, value in sorted(params.items()):
        target = declared(entries, key)["target"]
        if target["kind"] != "ros_param":
            continue
        segments = target["param"].split(".")
        for other, (other_node, other_segments) in written.items():
            shortest = min(len(segments), len(other_segments))
            if other_node == target["node"] and segments[:shortest] == other_segments[:shortest]:
                raise SpecError(f"params.{key} and params.{other} set the same ROS param")
        written[key] = (target["node"], segments)
        node = document.setdefault(target["node"], {}).setdefault("ros__parameters", {})
        *parents, leaf = target["param"].split(".")
        for segment in parents:
            node = node.setdefault(segment, {})
        node[leaf] = value
    return document
