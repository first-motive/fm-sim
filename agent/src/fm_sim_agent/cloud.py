"""The AWS sim host stack (``infra/aws/sim``), driven through the ``aws`` CLI.

``fm sim host start|stop|status aws`` comes here. Start and stop go through the
stack's controller Lambda, which owns the budget gate; nothing here touches the
Auto Scaling group. Status is read from the ``/<stack>/status`` parameter the
controller rewrites every minute, so a viewer that may not start the host can
still read it.

Every call is one ``aws`` process, run as argv with no shell. The ``aws`` CLI
is the operator's own tool and brings their profile and credentials; boto3
would be a heavy new dependency for three calls.

Everything AWS returns is checked against the stack's contract before it is
used: an unknown state, budget reason or error code is an invalid reply, not
something to print as if it were known.
"""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

PROVIDERS = ("aws",)
ACTIONS = ("status", "start", "stop")
STACK_ENV = "FM_SIM_CLOUD_STACK"
REGION_ENV = "FM_SIM_CLOUD_REGION"

STATES = ("unknown", "stopped", "starting", "running", "stopping")
BUDGET_REASONS = ("under_cap", "over_cap", "budget_unavailable")
CONTROLLER_CODES = (
    "budget_blocked",
    "budget_unavailable",
    "invalid_request",
    "unknown_action",
    "controller_error",
)
BUDGET_AMOUNTS = ("limit_usd", "actual_usd", "forecast_usd", "block_percent")

STACK_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9-]{0,127}$")
REGION_PATTERN = re.compile(r"^[a-z]{2}(-[a-z]+)+-[0-9]+$")
PROFILE_PATTERN = re.compile(r"^[A-Za-z0-9_.+=,@][A-Za-z0-9_.+=,@-]{0,63}$")
FUNCTION_PATTERN = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
PARAMETER_PATTERN = re.compile(r"^(/[A-Za-z0-9_.-]+){1,15}$")
AWS_ERROR_CODE = re.compile(r"An error occurred \(([A-Za-z0-9]+)\)")

#: The group holds zero or one instance; a handful is already a bad reply.
MAX_INSTANCES = 8
#: The status parameter is one String (4 KiB at most); a reply is a few times that.
#: tradeoff: checked after ``subprocess.run`` has buffered the output, so it
#: stops a runaway reply from being parsed, not from being read; the ``aws``
#: CLI is the operator's own, so streaming to a capped file is not worth it.
MAX_OUTPUT_BYTES = 1 << 20
AWS_TIMEOUT_S = 60.0
#: The controller allows two concurrent invocations, one often the minute
#: reconcile. A throttled invoke waits 1, 2, then 4 s before giving up.
INVOKE_ATTEMPTS = 4
INVOKE_BACKOFF_S = 1.0
THROTTLED = "TooManyRequestsException"
ACCESS_DENIED = ("AccessDenied", "AccessDeniedException")


class AwsError(Exception):
    """AWS could not be asked, or did not answer as the stack's contract says."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class Target:
    """Which stack, and how the ``aws`` CLI reaches it."""

    stack: str
    region: str | None = None
    profile: str | None = None


@dataclass(frozen=True)
class Outputs:
    function_name: str
    status_parameter: str
    host_name: str | None


def target_problem(target: Target) -> str | None:
    """Why the flags cannot name a stack, or ``None``. Every one becomes argv."""
    if not STACK_PATTERN.fullmatch(target.stack):
        return f"stack name must match {STACK_PATTERN.pattern} (--stack or ${STACK_ENV})"
    if target.region is not None and not REGION_PATTERN.fullmatch(target.region):
        return f"region must match {REGION_PATTERN.pattern} (--region or ${REGION_ENV})"
    if target.profile is not None and not PROFILE_PATTERN.fullmatch(target.profile):
        return f"profile must match {PROFILE_PATTERN.pattern}"
    return None


def _finite_or_none(value: object) -> bool:
    if value is None:
        return True
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def status_problem(status: object) -> str | None:
    """Why a Status does not hold to the stack's schema, or ``None``."""
    if not isinstance(status, dict) or status.get("schema_version") != 1:
        return "status is not a schema_version 1 object"
    state = status.get("state")
    if state not in STATES:
        return "status has an unknown state"
    if state == "unknown":
        return None  # what the parameter holds before the first reconcile
    if status.get("desired_capacity") not in (0, 1) or isinstance(status.get("desired_capacity"), bool):
        return "desired_capacity is not 0 or 1"
    instances = status.get("instances")
    if not isinstance(instances, list) or len(instances) > MAX_INSTANCES:
        return "instances is not a short list"
    for instance in instances:
        if not isinstance(instance, dict) or not isinstance(instance.get("instance_id"), str):
            return "an instance has no instance_id"
    budget = status.get("budget")
    if not isinstance(budget, dict):
        return "status has no budget"
    if budget.get("reason") not in BUDGET_REASONS:
        return "budget has an unknown reason"
    if not isinstance(budget.get("blocked"), bool):
        return "budget.blocked is not a boolean"
    if not all(_finite_or_none(budget.get(key)) for key in BUDGET_AMOUNTS):
        return "a budget amount is not a finite number"
    return None


def reply_problem(reply: object) -> str | None:
    """Why a controller reply is not ``{"ok": true, "status"}`` or a known refusal."""
    if not isinstance(reply, dict) or not isinstance(reply.get("ok"), bool):
        return "the controller reply is not an envelope"
    if reply["ok"]:
        return status_problem(reply.get("status"))
    error = reply.get("error")
    if not isinstance(error, dict) or error.get("code") not in CONTROLLER_CODES:
        return "the controller refused with an unknown code"
    if "status" in reply:
        return status_problem(reply["status"])
    return None


def _json_object(text: str, what: str) -> dict:
    try:
        decoded = json.loads(text)
    except (ValueError, RecursionError):
        raise AwsError("invalid_reply", f"{what} is not JSON") from None
    if not isinstance(decoded, dict):
        raise AwsError("invalid_reply", f"{what} is not a JSON object")
    return decoded


class Aws:
    """One stack, through the ``aws`` CLI found on ``PATH``."""

    def __init__(self, target: Target, *, aws: str = "aws") -> None:
        self.target = target
        self.aws = aws

    def _cli(self, *args: str) -> str:
        argv = [self.aws]
        if self.target.profile:
            argv += ["--profile", self.target.profile]
        if self.target.region:
            argv += ["--region", self.target.region]
        argv += ["--output", "json", *args]
        try:
            done = subprocess.run(
                argv,
                capture_output=True,
                timeout=AWS_TIMEOUT_S,
                env={**os.environ, "AWS_PAGER": ""},
                check=False,
            )
        except FileNotFoundError:
            raise AwsError("aws_unavailable", "the aws CLI is not on PATH") from None
        except subprocess.TimeoutExpired:
            raise AwsError("aws_timeout", f"aws {args[0]} did not finish in {AWS_TIMEOUT_S:.0f} s") from None
        if done.returncode != 0:
            stderr = done.stderr[:4096].decode("utf-8", "replace")
            match = AWS_ERROR_CODE.search(stderr)
            lines = stderr.strip().splitlines()
            raise AwsError(
                match.group(1) if match else "aws_error", lines[-1][:300] if lines else "aws failed"
            )
        if len(done.stdout) > MAX_OUTPUT_BYTES:
            raise AwsError("invalid_reply", f"aws {args[0]} printed more than {MAX_OUTPUT_BYTES} bytes")
        return done.stdout.decode("utf-8", "replace")

    def outputs(self) -> Outputs:
        """The stack's outputs, or the contract's names when the caller may not describe stacks.

        The stack's operator and viewer policies grant
        ``cloudformation:DescribeStacks`` on it. A role without that grant
        still works: the names the stack gives its controller and parameter
        are fixed by the contract, so they stand in.

        tradeoff: with that fallback a mistyped stack name surfaces as the
        next call's ``ParameterNotFound`` or ``ResourceNotFoundException``,
        not ``stack_not_found``; the policies' DescribeStacks grant avoids it.
        """
        stack = self.target.stack
        try:
            described = _json_object(
                self._cli("cloudformation", "describe-stacks", "--stack-name", stack), "describe-stacks"
            )
        except AwsError as exc:
            if exc.code in ACCESS_DENIED:
                return Outputs(f"{stack}-controller", f"/{stack}/status", None)
            if exc.code == "ValidationError":
                raise AwsError("stack_not_found", f"stack {stack} was not found") from None
            raise
        stacks = described.get("Stacks")
        if not isinstance(stacks, list) or len(stacks) != 1 or not isinstance(stacks[0], dict):
            raise AwsError("stack_not_found", f"stack {stack} was not found")
        values = {
            o.get("OutputKey"): o.get("OutputValue")
            for o in stacks[0].get("Outputs") or []
            if isinstance(o, dict)
        }
        function, parameter = values.get("ControllerFunctionName"), values.get("StatusParameterName")
        if not isinstance(function, str) or not FUNCTION_PATTERN.fullmatch(function):
            raise AwsError("invalid_reply", "the stack has no usable ControllerFunctionName output")
        if not isinstance(parameter, str) or not PARAMETER_PATTERN.fullmatch(parameter):
            raise AwsError("invalid_reply", "the stack has no usable StatusParameterName output")
        host = values.get("HostName")
        return Outputs(function, parameter, host if isinstance(host, str) else None)

    def status(self, outputs: Outputs) -> dict:
        """``{"ok": true, "status"}`` from the status parameter."""
        got = _json_object(
            self._cli("ssm", "get-parameter", "--name", outputs.status_parameter), "get-parameter"
        )
        parameter = got.get("Parameter")
        value = parameter.get("Value") if isinstance(parameter, dict) else None
        if not isinstance(value, str):
            raise AwsError("invalid_reply", "the status parameter has no value")
        status = _json_object(value, "the status parameter")
        problem = status_problem(status)
        if problem:
            raise AwsError("invalid_reply", problem)
        return {"ok": True, "status": status}

    def invoke(self, outputs: Outputs, action: str) -> dict:
        """The controller's reply to exactly ``{"action": action}``, for start or stop."""
        if action not in ("start", "stop"):
            raise ValueError(f"unknown action {action}")
        payload = json.dumps({"action": action})
        with tempfile.TemporaryDirectory(prefix="fm-sim-host-") as scratch:
            out = Path(scratch) / "reply.json"
            for attempt in range(INVOKE_ATTEMPTS):
                try:
                    meta = self._cli(
                        "lambda", "invoke",
                        "--function-name", outputs.function_name,
                        "--cli-binary-format", "raw-in-base64-out",
                        "--payload", payload,
                        str(out),
                    )  # fmt: skip
                    break
                except AwsError as exc:
                    if exc.code != THROTTLED or attempt == INVOKE_ATTEMPTS - 1:
                        raise
                    time.sleep(INVOKE_BACKOFF_S * 2**attempt)
            if _json_object(meta, "lambda invoke").get("FunctionError"):
                # The function raised instead of replying: the controller's own failure.
                return {
                    "ok": False,
                    "error": {"code": "controller_error", "message": "the controller failed"},
                }
            if not out.is_file() or out.stat().st_size > MAX_OUTPUT_BYTES:
                raise AwsError("invalid_reply", "the controller reply is missing or oversized")
            reply = _json_object(out.read_text(encoding="utf-8", errors="replace"), "the controller reply")
        problem = reply_problem(reply)
        if problem:
            raise AwsError("invalid_reply", problem)
        return reply


def run_action(host: Aws, action: str) -> tuple[dict, Outputs]:
    """Resolve the stack, then read (``status``) or ask the controller (``start``, ``stop``)."""
    outputs = host.outputs()
    reply = host.status(outputs) if action == "status" else host.invoke(outputs, action)
    return reply, outputs
