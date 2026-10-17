"""Lifecycle controller for the AWS sim host's Auto Scaling group.

One Lambda function owns every capacity change except the host's own idle exit.
It answers four actions:

- ``start`` sets desired capacity to 1, unless the monthly budget gate blocks it.
- ``stop`` sets desired capacity to 0.
- ``status`` publishes and returns the current status.
- ``reconcile`` runs every minute. It terminates an instance that has outlived
  the deadline and publishes the status.

Every reply is ``{"ok": true, "status": {...}}`` or
``{"ok": false, "error": {"code", "message"}, "status"?: {...}}``, the envelope
the sim agent uses. The status is also written to one SSM String parameter, so a
reader that may not start the host can still see its state and budget.

The pattern follows fm-data's annotation lifecycle controller: the budget is
read from AWS Budgets, an unreadable budget blocks a start, and the deadline is
enforced by the controller rather than trusted to the instance.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Any, Callable, Mapping

SCHEMA_VERSION = 1
ACTIONS = ("start", "stop", "status", "reconcile")
# An instance in one of these states is already on its way out of the group.
LEAVING_STATES = (
    "Terminating",
    "Terminating:Wait",
    "Terminating:Proceed",
    "Terminated",
    "Detaching",
    "Detached",
)


class BudgetUnavailable(Exception):
    """The budget could not be read, so spend cannot be proven below the cap."""


@dataclass(frozen=True)
class Config:
    auto_scaling_group: str
    budget_name: str
    budget_block_percent: int
    max_lifetime_minutes: int
    idle_minutes: int
    status_parameter: str
    account_id: str

    def __post_init__(self) -> None:
        for name in ("auto_scaling_group", "budget_name", "status_parameter", "account_id"):
            if not getattr(self, name):
                raise ValueError(f"{name} is required")
        if not 1 <= self.budget_block_percent <= 100:
            raise ValueError("budget_block_percent must be 1 to 100")
        if self.max_lifetime_minutes < 1 or self.idle_minutes < 1:
            raise ValueError("minutes must be positive")

    @classmethod
    def from_environ(cls, environ: Mapping[str, str]) -> "Config":
        return cls(
            auto_scaling_group=environ.get("AUTO_SCALING_GROUP", ""),
            budget_name=environ.get("BUDGET_NAME", ""),
            budget_block_percent=int(environ.get("BUDGET_BLOCK_PERCENT", "100")),
            max_lifetime_minutes=int(environ.get("MAX_LIFETIME_MINUTES", "0")),
            idle_minutes=int(environ.get("IDLE_MINUTES", "0")),
            status_parameter=environ.get("STATUS_PARAMETER", ""),
            account_id=environ.get("AWS_ACCOUNT_ID", ""),
        )


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso(value: datetime) -> str:
    return value.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _error(code: str, message: str, status: Mapping[str, Any] | None = None) -> dict[str, Any]:
    reply: dict[str, Any] = {"ok": False, "error": {"code": code, "message": message}}
    if status is not None:
        reply["status"] = dict(status)
    return reply


class Controller:
    """Drives one Auto Scaling group. The AWS clients are boto3 clients or fakes."""

    def __init__(
        self,
        config: Config,
        *,
        autoscaling: Any,
        ec2: Any,
        budgets: Any,
        ssm: Any,
        now: Callable[[], datetime] = _utc_now,
        log: Callable[[str], None] = print,
    ) -> None:
        self.config = config
        self.autoscaling = autoscaling
        self.ec2 = ec2
        self.budgets = budgets
        self.ssm = ssm
        self.now = now
        self.log = log

    def handle(self, event: object) -> dict[str, Any]:
        if not isinstance(event, Mapping) or set(event) != {"action"}:
            return _error("invalid_request", 'the event must be exactly {"action": <name>}')
        action = event["action"]
        if action not in ACTIONS:
            return _error("unknown_action", f"action must be one of {', '.join(ACTIONS)}")
        try:
            return getattr(self, f"_{action}")()
        except Exception as exc:  # noqa: BLE001 - the reply carries a code, the log the detail
            self.log(f"{action} failed: {exc!r}")
            return _error("controller_error", f"{action} failed; see the controller log")

    def _start(self) -> dict[str, Any]:
        budget = self._budget()
        if budget["blocked"]:
            status = self._publish(budget=budget)
            code = "budget_unavailable" if budget["reason"] == "budget_unavailable" else "budget_blocked"
            return _error(code, "the monthly budget gate refuses a start", status)
        if self._describe()["DesiredCapacity"] < 1:
            self._set_desired(1)
        return {"ok": True, "status": self._publish(budget=budget)}

    def _stop(self) -> dict[str, Any]:
        if self._describe()["DesiredCapacity"] > 0:
            self._set_desired(0)
        return {"ok": True, "status": self._publish()}

    def _status(self) -> dict[str, Any]:
        return {"ok": True, "status": self._publish()}

    def _reconcile(self) -> dict[str, Any]:
        for instance in self._instances(self._describe()):
            if instance["lifecycle_state"] in LEAVING_STATES:
                continue
            if self._past_deadline(instance):
                self.log(f"deadline passed for {instance['instance_id']}; terminating")
                self.autoscaling.terminate_instance_in_auto_scaling_group(
                    InstanceId=instance["instance_id"], ShouldDecrementDesiredCapacity=True
                )
        return {"ok": True, "status": self._publish()}

    def _describe(self) -> Mapping[str, Any]:
        groups = self.autoscaling.describe_auto_scaling_groups(
            AutoScalingGroupNames=[self.config.auto_scaling_group]
        ).get("AutoScalingGroups", [])
        if len(groups) != 1:
            raise RuntimeError(f"auto scaling group {self.config.auto_scaling_group} not found")
        return groups[0]

    def _set_desired(self, capacity: int) -> None:
        self.autoscaling.set_desired_capacity(
            AutoScalingGroupName=self.config.auto_scaling_group,
            DesiredCapacity=capacity,
            HonorCooldown=False,
        )

    def _instances(self, group: Mapping[str, Any]) -> list[dict[str, Any]]:
        members = [m for m in group.get("Instances", []) if m.get("InstanceId")]
        if not members:
            return []
        launched: dict[str, datetime] = {}
        response = self.ec2.describe_instances(InstanceIds=[m["InstanceId"] for m in members])
        for reservation in response.get("Reservations", []):
            for instance in reservation.get("Instances", []):
                if isinstance(instance.get("LaunchTime"), datetime):
                    launched[instance["InstanceId"]] = instance["LaunchTime"]
        lifetime = timedelta(minutes=self.config.max_lifetime_minutes)
        instances = []
        for member in members:
            launch = launched.get(member["InstanceId"])
            instances.append(
                {
                    "instance_id": member["InstanceId"],
                    "lifecycle_state": member.get("LifecycleState", "unknown"),
                    "launched_at": _iso(launch) if launch else None,
                    "terminate_after": _iso(launch + lifetime) if launch else None,
                    "_launch": launch,
                }
            )
        return instances

    def _past_deadline(self, instance: Mapping[str, Any]) -> bool:
        launch = instance["_launch"]
        if launch is None:
            # No launch time means EC2 could not describe it; the next minute retries.
            return False
        return self.now() >= launch + timedelta(minutes=self.config.max_lifetime_minutes)

    # tradeoff: one describe_budget per invocation, about 45k calls a month at
    # one a minute; cache it in the status parameter if that call is ever billed.
    def _budget(self) -> dict[str, Any]:
        percent = self.config.budget_block_percent
        summary: dict[str, Any] = {
            "name": self.config.budget_name,
            "block_percent": percent,
            "limit_usd": None,
            "actual_usd": None,
            "forecast_usd": None,
        }
        try:
            actual, limit, forecast = self._read_budget()
        except BudgetUnavailable as exc:
            self.log(f"budget unavailable: {exc}")
            return {**summary, "blocked": True, "reason": "budget_unavailable"}
        blocked = actual >= limit * percent / 100
        return {
            **summary,
            "limit_usd": limit,
            "actual_usd": actual,
            "forecast_usd": forecast,
            "blocked": blocked,
            "reason": "over_cap" if blocked else "under_cap",
        }

    def _read_budget(self) -> tuple[float, float, float | None]:
        try:
            budget = self.budgets.describe_budget(
                AccountId=self.config.account_id, BudgetName=self.config.budget_name
            )["Budget"]
            spend = budget.get("CalculatedSpend", {})
            actual = float(spend["ActualSpend"]["Amount"])
            limit = float(budget["BudgetLimit"]["Amount"])
        except Exception as exc:  # noqa: BLE001 - any failure leaves spend unproven
            raise BudgetUnavailable(repr(exc)) from exc
        if not (math.isfinite(actual) and math.isfinite(limit)) or actual < 0 or limit <= 0:
            raise BudgetUnavailable("budget amounts are not usable")
        forecast_amount = spend.get("ForecastedSpend", {}).get("Amount")
        try:
            forecast = float(forecast_amount) if forecast_amount is not None else None
        except ValueError:
            forecast = None
        return actual, limit, forecast

    def _publish(self, budget: Mapping[str, Any] | None = None) -> dict[str, Any]:
        group = self._describe()
        instances = self._instances(group)
        status = {
            "schema_version": SCHEMA_VERSION,
            "updated_at": _iso(self.now()),
            "auto_scaling_group": self.config.auto_scaling_group,
            "state": _state(group["DesiredCapacity"], instances),
            "desired_capacity": group["DesiredCapacity"],
            "instances": [{k: v for k, v in i.items() if not k.startswith("_")} for i in instances],
            "max_lifetime_minutes": self.config.max_lifetime_minutes,
            "idle_minutes": self.config.idle_minutes,
            "budget": dict(budget) if budget is not None else self._budget(),
        }
        try:
            self.ssm.put_parameter(
                Name=self.config.status_parameter,
                Value=json.dumps(status, sort_keys=True, separators=(",", ":")),
                Type="String",
                Overwrite=True,
            )
        except Exception as exc:  # noqa: BLE001 - a stale status must not undo a capacity change
            self.log(f"status publish failed: {exc!r}")
        return status


def _state(desired: int, instances: list[Mapping[str, Any]]) -> str:
    """EC2-level state. Whether the sim agent is ready is the agent's own status."""
    live = [i for i in instances if i["lifecycle_state"] not in LEAVING_STATES]
    if desired == 0:
        return "stopping" if instances else "stopped"
    if any(i["lifecycle_state"] == "InService" for i in live):
        return "running"
    return "starting"


def handler(event: object, context: object = None) -> dict[str, Any]:
    import boto3  # the Lambda runtime ships it; tests use fakes instead

    try:
        config = Config.from_environ(os.environ)
    except ValueError as exc:
        print(f"bad controller configuration: {exc}")
        return _error("controller_error", "the controller is misconfigured; see the controller log")
    controller = Controller(
        config,
        autoscaling=boto3.client("autoscaling"),
        ec2=boto3.client("ec2"),
        budgets=boto3.client("budgets"),
        ssm=boto3.client("ssm"),
    )
    return controller.handle(event)
