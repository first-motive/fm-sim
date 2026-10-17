"""Lifecycle controller behaviour against fake AWS clients."""

from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone

import pytest

from controller import Config, Controller

NOW = datetime(2026, 10, 17, 12, 0, tzinfo=timezone.utc)


class FakeAutoScaling:
    def __init__(self, desired: int = 0, instances: list[dict] | None = None, exists: bool = True) -> None:
        self.desired = desired
        self.instances = instances or []
        self.exists = exists
        self.calls: list[tuple[str, dict]] = []

    def describe_auto_scaling_groups(self, **kwargs):
        if not self.exists:
            return {"AutoScalingGroups": []}
        return {"AutoScalingGroups": [{"DesiredCapacity": self.desired, "Instances": list(self.instances)}]}

    def set_desired_capacity(self, **kwargs):
        self.calls.append(("set_desired_capacity", kwargs))
        self.desired = kwargs["DesiredCapacity"]

    def terminate_instance_in_auto_scaling_group(self, **kwargs):
        self.calls.append(("terminate", kwargs))


class FakeEc2:
    def __init__(self, launched: dict[str, datetime] | None = None) -> None:
        self.launched = launched or {}

    def describe_instances(self, InstanceIds):
        return {
            "Reservations": [
                {
                    "Instances": [
                        {"InstanceId": i, "LaunchTime": self.launched[i]}
                        for i in InstanceIds
                        if i in self.launched
                    ]
                }
            ]
        }


class FakeBudgets:
    def __init__(self, actual="10.0", limit="100.0", forecast="40.0", error: Exception | None = None) -> None:
        self.actual, self.limit, self.forecast, self.error = actual, limit, forecast, error
        self.calls: list[dict] = []

    def describe_budget(self, **kwargs):
        self.calls.append(kwargs)
        if self.error:
            raise self.error
        spend = {"ActualSpend": {"Amount": self.actual, "Unit": "USD"}}
        if self.forecast is not None:
            spend["ForecastedSpend"] = {"Amount": self.forecast, "Unit": "USD"}
        return {"Budget": {"BudgetLimit": {"Amount": self.limit, "Unit": "USD"}, "CalculatedSpend": spend}}


class FakeSsm:
    def __init__(self, fail: bool = False) -> None:
        self.fail = fail
        self.values: dict[str, str] = {}

    def put_parameter(self, **kwargs):
        if self.fail:
            raise RuntimeError("throttled")
        assert kwargs["Type"] == "String" and kwargs["Overwrite"] is True
        self.values[kwargs["Name"]] = kwargs["Value"]


def config(**overrides) -> Config:
    values = dict(
        auto_scaling_group="stack-host",
        budget_name="stack-monthly",
        budget_block_percent=100,
        max_lifetime_minutes=480,
        idle_minutes=15,
        status_parameter="/stack/status",
        account_id="000000000000",
    )
    values.update(overrides)
    return Config(**values)


def make(asg=None, ec2=None, budgets=None, ssm=None, **overrides):
    asg = asg or FakeAutoScaling()
    ec2 = ec2 or FakeEc2()
    budgets = budgets or FakeBudgets()
    ssm = ssm or FakeSsm()
    logs: list[str] = []
    ctl = Controller(
        config(**overrides),
        autoscaling=asg,
        ec2=ec2,
        budgets=budgets,
        ssm=ssm,
        now=lambda: NOW,
        log=logs.append,
    )
    return ctl, asg, ssm, logs


def capacity_calls(asg: FakeAutoScaling) -> list[tuple[str, dict]]:
    return [c for c in asg.calls if c[0] == "set_desired_capacity"]


def test_start_under_cap_raises_capacity_and_publishes_status():
    ctl, asg, ssm, _ = make()
    reply = ctl.handle({"action": "start"})
    assert reply["ok"] is True
    assert capacity_calls(asg) == [
        (
            "set_desired_capacity",
            {"AutoScalingGroupName": "stack-host", "DesiredCapacity": 1, "HonorCooldown": False},
        )
    ]
    published = json.loads(ssm.values["/stack/status"])
    assert published == reply["status"]
    assert published["state"] == "starting"
    assert published["budget"]["blocked"] is False
    assert published["budget"]["reason"] == "under_cap"
    assert published["budget"]["actual_usd"] == 10.0
    assert published["budget"]["forecast_usd"] == 40.0


@pytest.mark.parametrize("actual", ["100.0", "150.0"])
def test_start_at_or_over_cap_is_refused_without_capacity(actual):
    ctl, asg, ssm, _ = make(budgets=FakeBudgets(actual=actual))
    reply = ctl.handle({"action": "start"})
    assert reply["ok"] is False
    assert reply["error"]["code"] == "budget_blocked"
    assert capacity_calls(asg) == []
    assert json.loads(ssm.values["/stack/status"])["budget"]["blocked"] is True
    assert reply["status"]["state"] == "stopped"


def test_block_percent_lowers_the_cap():
    ctl, asg, _, _ = make(budgets=FakeBudgets(actual="90.0"), budget_block_percent=90)
    assert ctl.handle({"action": "start"})["error"]["code"] == "budget_blocked"
    assert capacity_calls(asg) == []


@pytest.mark.parametrize(
    "budgets",
    [
        FakeBudgets(error=RuntimeError("AccessDenied")),
        FakeBudgets(limit="nan"),
        FakeBudgets(limit="0"),
        FakeBudgets(actual="x"),
    ],
)
def test_unreadable_budget_fails_closed(budgets):
    ctl, asg, _, _ = make(budgets=budgets)
    reply = ctl.handle({"action": "start"})
    assert reply["error"]["code"] == "budget_unavailable"
    assert reply["status"]["budget"]["reason"] == "budget_unavailable"
    assert capacity_calls(asg) == []


def test_budget_is_read_for_the_configured_account_and_name():
    budgets = FakeBudgets()
    ctl, _, _, _ = make(budgets=budgets)
    ctl.handle({"action": "status"})
    assert budgets.calls == [{"AccountId": "000000000000", "BudgetName": "stack-monthly"}]


def test_start_when_already_started_changes_nothing():
    ctl, asg, _, _ = make(asg=FakeAutoScaling(desired=1))
    assert ctl.handle({"action": "start"})["ok"] is True
    assert capacity_calls(asg) == []


def test_stop_lowers_capacity_once():
    ctl, asg, _, _ = make(asg=FakeAutoScaling(desired=1))
    assert ctl.handle({"action": "stop"})["ok"] is True
    assert capacity_calls(asg)[0][1]["DesiredCapacity"] == 0
    ctl.handle({"action": "stop"})
    assert len(capacity_calls(asg)) == 1


def test_stop_is_allowed_while_budget_blocked():
    ctl, asg, _, _ = make(asg=FakeAutoScaling(desired=1), budgets=FakeBudgets(actual="500"))
    assert ctl.handle({"action": "stop"})["ok"] is True
    assert capacity_calls(asg)[0][1]["DesiredCapacity"] == 0


def running(launch: datetime, state: str = "InService"):
    asg = FakeAutoScaling(desired=1, instances=[{"InstanceId": "i-0abc", "LifecycleState": state}])
    return asg, FakeEc2({"i-0abc": launch})


def test_reconcile_terminates_past_deadline_with_decrement():
    asg, ec2 = running(NOW - timedelta(minutes=480))
    ctl, _, _, _ = make(asg=asg, ec2=ec2)
    reply = ctl.handle({"action": "reconcile"})
    assert reply["ok"] is True
    assert ("terminate", {"InstanceId": "i-0abc", "ShouldDecrementDesiredCapacity": True}) in asg.calls


def test_reconcile_leaves_a_host_inside_its_deadline():
    asg, ec2 = running(NOW - timedelta(minutes=479))
    ctl, _, _, _ = make(asg=asg, ec2=ec2)
    status = ctl.handle({"action": "reconcile"})["status"]
    assert asg.calls == []
    assert status["state"] == "running"
    assert status["instances"] == [
        {
            "instance_id": "i-0abc",
            "lifecycle_state": "InService",
            "launched_at": "2026-10-17T04:01:00Z",
            "terminate_after": "2026-10-17T12:01:00Z",
        }
    ]


def test_reconcile_skips_an_instance_already_leaving():
    asg, ec2 = running(NOW - timedelta(hours=10), state="Terminating")
    ctl, _, _, _ = make(asg=asg, ec2=ec2)
    ctl.handle({"action": "reconcile"})
    assert asg.calls == []


def test_reconcile_skips_an_instance_without_launch_time():
    asg = FakeAutoScaling(desired=1, instances=[{"InstanceId": "i-0abc", "LifecycleState": "Pending"}])
    ctl, _, _, _ = make(asg=asg)
    status = ctl.handle({"action": "reconcile"})["status"]
    assert asg.calls == []
    assert status["instances"][0]["terminate_after"] is None


@pytest.mark.parametrize(
    "desired, instances, expected",
    [
        (0, [], "stopped"),
        (1, [], "starting"),
        (1, [{"InstanceId": "i-0abc", "LifecycleState": "Pending"}], "starting"),
        (1, [{"InstanceId": "i-0abc", "LifecycleState": "InService"}], "running"),
        (0, [{"InstanceId": "i-0abc", "LifecycleState": "Terminating"}], "stopping"),
    ],
)
def test_status_state(desired, instances, expected):
    ctl, _, _, _ = make(
        asg=FakeAutoScaling(desired=desired, instances=instances), ec2=FakeEc2({"i-0abc": NOW})
    )
    assert ctl.handle({"action": "status"})["status"]["state"] == expected


@pytest.mark.parametrize(
    "event, code",
    [
        (None, "invalid_request"),
        ("start", "invalid_request"),
        ({}, "invalid_request"),
        ({"action": "start", "capacity": 5}, "invalid_request"),
        ({"action": "scale"}, "unknown_action"),
    ],
)
def test_bad_events_are_refused_without_touching_aws(event, code):
    ctl, asg, ssm, _ = make()
    reply = ctl.handle(event)
    assert reply == {"ok": False, "error": {"code": code, "message": reply["error"]["message"]}}
    assert asg.calls == [] and ssm.values == {}


def test_status_publish_failure_does_not_undo_a_start():
    ctl, asg, _, logs = make(ssm=FakeSsm(fail=True))
    assert ctl.handle({"action": "start"})["ok"] is True
    assert capacity_calls(asg)
    assert any("status publish failed" in line for line in logs)


def test_missing_group_is_a_controller_error():
    ctl, _, _, logs = make(asg=FakeAutoScaling(exists=False))
    reply = ctl.handle({"action": "status"})
    assert reply["error"]["code"] == "controller_error"
    assert logs


def test_config_from_environ_requires_every_name():
    environ = {
        "AUTO_SCALING_GROUP": "stack-host",
        "BUDGET_NAME": "stack-monthly",
        "BUDGET_BLOCK_PERCENT": "100",
        "MAX_LIFETIME_MINUTES": "480",
        "IDLE_MINUTES": "15",
        "STATUS_PARAMETER": "/stack/status",
        "AWS_ACCOUNT_ID": "000000000000",
    }
    assert Config.from_environ(environ) == config()
    for key in ("AUTO_SCALING_GROUP", "BUDGET_NAME", "STATUS_PARAMETER", "AWS_ACCOUNT_ID"):
        with pytest.raises(ValueError):
            Config.from_environ({**environ, key: ""})
    with pytest.raises(ValueError):
        Config.from_environ({**environ, "BUDGET_BLOCK_PERCENT": "0"})
