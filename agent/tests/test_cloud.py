"""``fm sim host start|stop|status aws``, run against a stand-in ``aws`` on ``PATH``.

The stand-in is a real executable that records its argv and answers from a
scenario in the environment, so every call goes through the same
``subprocess`` argv the CLI sends to the real ``aws``. Nothing reaches AWS.
"""

from __future__ import annotations

import json
import os
import stat
import sys
import textwrap

import pytest

from fm_sim_agent import cloud
from fm_sim_agent.client import main, parse_args

STACK = "fm-sim-cloud-host"

FAKE_AWS = textwrap.dedent(
    """\
    import json, os, sys
    args = sys.argv[1:]
    with open(os.environ["FAKE_AWS_LOG"], "a") as log:
        log.write(json.dumps(args) + "\\n")
    scenario = json.loads(os.environ["FAKE_AWS"])
    while args and args[0] in ("--profile", "--region", "--output"):
        args = args[2:]
    def fail(code, message="boom"):
        print(f"\\nAn error occurred ({code}) when calling the X operation: {message}", file=sys.stderr)
        sys.exit(254)
    if args[:2] == ["cloudformation", "describe-stacks"]:
        if "describe_error" in scenario:
            fail(scenario["describe_error"])
        print(json.dumps({"Stacks": [{"Outputs": scenario["outputs"]}]}))
    elif args[:2] == ["ssm", "get-parameter"]:
        print(json.dumps({"Parameter": {"Value": scenario["parameter"]}}))
    elif args[:2] == ["lambda", "invoke"]:
        counter = os.environ["FAKE_AWS_LOG"] + ".invokes"
        n = int(open(counter).read()) if os.path.exists(counter) else 0
        open(counter, "w").write(str(n + 1))
        answer = scenario["invokes"][min(n, len(scenario["invokes"]) - 1)]
        if "error" in answer:
            fail(answer["error"])
        with open(args[-1], "w") as out:
            out.write(json.dumps(answer.get("reply", {"errorMessage": "x"})))
        meta = {"StatusCode": 200}
        if answer.get("function_error"):
            meta["FunctionError"] = "Unhandled"
        print(json.dumps(meta))
    else:
        fail("UnknownCommand")
    """
)

OUTPUTS = [
    {"OutputKey": "ControllerFunctionName", "OutputValue": f"{STACK}-controller"},
    {"OutputKey": "StatusParameterName", "OutputValue": f"/{STACK}/status"},
    {"OutputKey": "HostName", "OutputValue": "sim-cloud"},
    {"OutputKey": "AutoScalingGroupName", "OutputValue": "asg-1"},
]


def status(state="running", **budget) -> dict:
    return {
        "schema_version": 1,
        "updated_at": "2026-10-17T10:00:00Z",
        "auto_scaling_group": "asg-1",
        "state": state,
        "desired_capacity": 0 if state in ("stopped", "stopping") else 1,
        "instances": [
            {
                "instance_id": "i-0abc",
                "lifecycle_state": "InService",
                "launched_at": "2026-10-17T09:00:00Z",
                "terminate_after": "2026-10-17T17:00:00Z",
            }
        ]
        if state == "running"
        else [],
        "max_lifetime_minutes": 480,
        "idle_minutes": 15,
        "budget": {
            "name": "fm-sim-cloud",
            "limit_usd": 100.0,
            "actual_usd": 12.3,
            "forecast_usd": 40.0,
            "block_percent": 100,
            "blocked": False,
            "reason": "under_cap",
            **budget,
        },
    }


@pytest.fixture
def aws(tmp_path, monkeypatch):
    """Put the stand-in first on PATH; return a setter for its scenario and its call log."""
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    script = bin_dir / "aws"
    script.write_text(f"#!{sys.executable}\n{FAKE_AWS}")
    script.chmod(script.stat().st_mode | stat.S_IEXEC)
    log = tmp_path / "aws.log"
    monkeypatch.setenv("PATH", f"{bin_dir}{os.pathsep}{os.environ['PATH']}")
    monkeypatch.setenv("FAKE_AWS_LOG", str(log))
    monkeypatch.setenv(cloud.STACK_ENV, STACK)
    monkeypatch.delenv(cloud.REGION_ENV, raising=False)
    monkeypatch.setattr(cloud, "INVOKE_BACKOFF_S", 0.0)

    class Fake:
        def scenario(self, **values) -> None:
            monkeypatch.setenv("FAKE_AWS", json.dumps({"outputs": OUTPUTS, **values}))

        def calls(self) -> list[list[str]]:
            return [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []

    return Fake()


# -- flags -------------------------------------------------------------------


def test_stack_and_region_come_from_flags_else_the_environment(monkeypatch):
    monkeypatch.setenv(cloud.STACK_ENV, "from-env")
    monkeypatch.setenv(cloud.REGION_ENV, "eu-west-1")
    args = parse_args(["host", "status", "aws"])
    assert args.target == cloud.Target("from-env", "eu-west-1", None)
    args = parse_args(
        ["host", "start", "aws", "--stack", "flag", "--region", "us-east-2", "--profile", "ops"]
    )
    assert args.target == cloud.Target("flag", "us-east-2", "ops")


@pytest.mark.parametrize(
    "argv",
    [
        ["host", "status", "gcp"],
        ["host", "reboot", "aws"],
        ["host", "status", "aws", "--stack", "../etc"],
        ["host", "status", "aws", "--stack", "-x"],
        ["host", "status", "aws", "--region", "Mars"],
        ["host", "status", "aws", "--profile", "--debug"],
        ["host", "status", "aws", "--stack", "fm-sim\n"],
        ["host", "status", "aws", "--region", "eu-west-1\n"],
    ],
)
def test_bad_host_flags_are_usage_errors(argv, monkeypatch):
    monkeypatch.setenv(cloud.STACK_ENV, STACK)
    with pytest.raises(SystemExit) as caught:
        parse_args(argv)
    assert caught.value.code == 2


def test_no_stack_anywhere_is_a_usage_error(monkeypatch):
    monkeypatch.delenv(cloud.STACK_ENV, raising=False)
    with pytest.raises(SystemExit) as caught:
        parse_args(["host", "status", "aws"])
    assert caught.value.code == 2


# -- status ------------------------------------------------------------------


def test_status_reads_the_parameter_the_stack_names_and_never_invokes(aws, capsys):
    aws.scenario(parameter=json.dumps(status()))
    assert main(["host", "status", "aws", "--region", "eu-west-1", "--profile", "ops"]) == 0
    describe, get = aws.calls()
    assert describe[:8] == [
        "--profile", "ops", "--region", "eu-west-1", "--output", "json", "cloudformation", "describe-stacks",
    ]  # fmt: skip
    assert get[6:] == ["ssm", "get-parameter", "--name", f"/{STACK}/status"]
    out = capsys.readouterr().out
    assert f"stack {STACK} (eu-west-1): running, desired 1" in out
    assert "i-0abc InService" in out and "terminate after 2026-10-17T17:00:00Z" in out
    assert "budget fm-sim-cloud: $12.30 of $100.00, blocks at 100%, under_cap" in out
    assert "fm sim status --host sim-cloud" in out


def test_status_json_is_the_checked_status_and_the_stack(aws, capsys):
    aws.scenario(parameter=json.dumps(status()))
    assert main(["host", "status", "aws", "--json"]) == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed["ok"] is True and printed["status"] == status()
    assert printed["stack"] == {"name": STACK, "region": None, "host_name": "sim-cloud"}


def test_status_before_the_first_reconcile_is_unknown(aws, capsys):
    aws.scenario(parameter='{"schema_version":1,"state":"unknown"}')
    assert main(["host", "status", "aws"]) == 0
    assert "unknown (the controller has not reconciled yet)" in capsys.readouterr().out


@pytest.mark.parametrize(
    "parameter",
    [
        json.dumps({**status(), "state": "rebooting"}),
        json.dumps(status(reason="maybe")),
        json.dumps(status(actual_usd=float("inf"))),
        json.dumps(status(blocked="no")),
        json.dumps({**status(), "schema_version": 2}),
        json.dumps(status(block_percent=float("nan"))),
        json.dumps({**status(), "desired_capacity": 2}),
        json.dumps({**status(), "desired_capacity": True}),
        json.dumps({**status(), "instances": [{"lifecycle_state": "InService"}] * 9}),
        json.dumps({**status(), "instances": [{"lifecycle_state": "InService"}]}),
        "not json",
    ],
)
def test_a_status_outside_the_schema_is_an_invalid_reply(aws, capsys, parameter):
    aws.scenario(parameter=parameter)
    assert main(["host", "status", "aws", "--json"]) == 3
    assert json.loads(capsys.readouterr().out)["error"]["code"] == "invalid_reply"


def test_a_viewer_without_describe_stacks_uses_the_contracts_names(aws, capsys):
    aws.scenario(describe_error="AccessDenied", parameter=json.dumps(status()))
    assert main(["host", "status", "aws"]) == 0
    assert aws.calls()[1][2:] == ["ssm", "get-parameter", "--name", f"/{STACK}/status"]
    assert "--host <HostName>" in capsys.readouterr().out


def test_a_missing_stack_or_aws_cli_is_a_precondition_failure(aws, capsys, monkeypatch):
    aws.scenario(describe_error="ValidationError")
    assert main(["host", "status", "aws"]) == 3
    assert "stack_not_found" in capsys.readouterr().err
    monkeypatch.setenv("PATH", "/nonexistent")
    assert main(["host", "status", "aws"]) == 3
    assert "aws_unavailable" in capsys.readouterr().err


def test_control_characters_from_aws_are_cleaned(aws, capsys):
    hostile = status()
    hostile["instances"][0]["instance_id"] = "i-0abc\x1b[2J‮"
    aws.scenario(parameter=json.dumps(hostile))
    assert main(["host", "status", "aws"]) == 0
    out = capsys.readouterr().out
    assert "\x1b" not in out and "‮" not in out and "i-0abc[2J" in out


# -- start and stop ----------------------------------------------------------


def test_start_invokes_the_controller_with_exactly_the_action(aws, capsys):
    aws.scenario(invokes=[{"reply": {"ok": True, "status": status("starting")}}])
    assert main(["host", "start", "aws"]) == 0
    _, invoke = aws.calls()
    assert invoke[2:6] == ["lambda", "invoke", "--function-name", f"{STACK}-controller"]
    assert json.loads(invoke[invoke.index("--payload") + 1]) == {"action": "start"}
    assert invoke[invoke.index("--cli-binary-format") + 1] == "raw-in-base64-out"
    assert not any("autoscaling" in call for call in aws.calls())
    assert f"stack {STACK}: starting, desired 1" in capsys.readouterr().out


def test_stop_invokes_stop(aws, capsys):
    aws.scenario(invokes=[{"reply": {"ok": True, "status": status("stopping")}}])
    assert main(["host", "stop", "aws"]) == 0
    invoke = aws.calls()[1]
    assert json.loads(invoke[invoke.index("--payload") + 1]) == {"action": "stop"}


def test_a_budget_block_is_a_refusal_with_its_status(aws, capsys):
    blocked = status("stopped", blocked=True, reason="over_cap", actual_usd=101.0)
    refusal = {"ok": False, "error": {"code": "budget_blocked", "message": "gate"}, "status": blocked}
    aws.scenario(invokes=[{"reply": refusal}])
    assert main(["host", "start", "aws"]) == 1
    captured = capsys.readouterr()
    assert "$101.00 of $100.00, blocks at 100%, blocked (over_cap)" in captured.out
    assert "refused (budget_blocked): gate" in captured.err


def test_a_throttled_invoke_is_retried(aws, capsys):
    ok = {"reply": {"ok": True, "status": status("starting")}}
    aws.scenario(invokes=[{"error": "TooManyRequestsException"}, {"error": "TooManyRequestsException"}, ok])
    assert main(["host", "start", "aws"]) == 0
    assert [c[2] for c in aws.calls()] == ["cloudformation", "lambda", "lambda", "lambda"]


def test_throttled_invokes_back_off_one_two_four_seconds(aws, capsys, monkeypatch):
    waits = []
    monkeypatch.setattr(cloud, "INVOKE_BACKOFF_S", 1.0)
    monkeypatch.setattr(cloud.time, "sleep", waits.append)
    aws.scenario(invokes=[{"error": "TooManyRequestsException"}])
    assert main(["host", "start", "aws"]) == 3
    assert waits == [1.0, 2.0, 4.0]


@pytest.mark.parametrize(
    "outputs",
    [
        [o for o in OUTPUTS if o["OutputKey"] != "ControllerFunctionName"],
        [o for o in OUTPUTS if o["OutputKey"] != "StatusParameterName"],
        [{**o, "OutputValue": "a b"} if o["OutputKey"] == "ControllerFunctionName" else o for o in OUTPUTS],
        [
            {**o, "OutputValue": "../status"} if o["OutputKey"] == "StatusParameterName" else o
            for o in OUTPUTS
        ],
    ],
)
def test_unusable_stack_outputs_are_an_invalid_reply(aws, capsys, monkeypatch, outputs):
    aws.scenario(parameter=json.dumps(status()), invokes=[{"reply": {"ok": True, "status": status()}}])
    monkeypatch.setenv("FAKE_AWS", json.dumps({**json.loads(os.environ["FAKE_AWS"]), "outputs": outputs}))
    assert main(["host", "start", "aws", "--json"]) == 3
    assert json.loads(capsys.readouterr().out)["error"]["code"] == "invalid_reply"
    assert len(aws.calls()) == 1


def test_a_throttle_that_never_clears_gives_up(aws, capsys):
    aws.scenario(invokes=[{"error": "TooManyRequestsException"}])
    assert main(["host", "start", "aws"]) == 3
    assert len(aws.calls()) == 1 + cloud.INVOKE_ATTEMPTS


def test_other_aws_errors_are_not_retried(aws, capsys):
    aws.scenario(invokes=[{"error": "AccessDeniedException"}])
    assert main(["host", "start", "aws", "--json"]) == 3
    assert len(aws.calls()) == 2
    assert json.loads(capsys.readouterr().out)["error"]["code"] == "AccessDeniedException"


def test_a_function_error_is_the_controllers_failure(aws, capsys):
    aws.scenario(invokes=[{"function_error": True}])
    assert main(["host", "start", "aws", "--json"]) == 1
    assert json.loads(capsys.readouterr().out)["error"]["code"] == "controller_error"


def test_an_unknown_refusal_code_is_an_invalid_reply(aws, capsys):
    aws.scenario(invokes=[{"reply": {"ok": False, "error": {"code": "whatever", "message": "x"}}}])
    assert main(["host", "stop", "aws"]) == 3
    assert "invalid_reply" in capsys.readouterr().err
