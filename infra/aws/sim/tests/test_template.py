"""host.yaml's controls, read from the template itself, and deploy.sh's plan mode."""

from __future__ import annotations

import base64
import json
import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

import controller

SIM_DIR = Path(__file__).resolve().parents[1]


class CfnLoader(yaml.SafeLoader):
    """Reads CloudFormation short-form tags as {"Fn": value} mappings."""


def _tag(loader, suffix, node):
    if isinstance(node, yaml.ScalarNode):
        value = loader.construct_scalar(node)
    elif isinstance(node, yaml.SequenceNode):
        value = loader.construct_sequence(node, deep=True)
    else:
        value = loader.construct_mapping(node, deep=True)
    return {suffix: value}


CfnLoader.add_multi_constructor("!", _tag)
TEMPLATE = yaml.load((SIM_DIR / "host.yaml").read_text(), Loader=CfnLoader)
RESOURCES = TEMPLATE["Resources"]
PARAMETERS = TEMPLATE["Parameters"]

SAMPLE = {
    "AWS::Region": "us-test-1",
    "AWS::StackName": "fm-sim-cloud-host",
    "IdleMinutes": "15",
    "MaxLifetimeMinutes": "480",
    "HostName": "sim-cloud",
    "FmSetupCommit": "a" * 40,
    "FmSimCommit": "b" * 40,
    "FmSetupRole": "sim-cloud",
    "FmSetupRepoUrl": "https://github.com/first-motive/fm-setup.git",
    "FmSimRepoUrl": "https://github.com/first-motive/fm-sim.git",
    "SimImageDigest": "sha256:" + "c" * 64,
    "RouterEndpoint": "tcp/router.example:7447",
    "TailscaleAuthKeyParameterName": "/fm-sim/cloud-host/tailscale-auth-key",
    "GhcrTokenParameterName": "",
    "GhcrUsername": "",
    "ControllersImageDigest": "",
}
CONTROLLERS = {
    "GhcrTokenParameterName": "/fm-sim/cloud-host/ghcr-token",
    "GhcrUsername": "fm-operator",
    "ControllersImageDigest": "sha256:" + "d" * 64,
}


def user_data(**overrides: str) -> str:
    """The bootstrap as the instance receives it, with sample parameter values.

    Variables in the Sub map resolve their Fn::If against UseControllersImage,
    which the sample decides the way the template does: a non-empty token name.
    """
    template, variables = RESOURCES["HostLaunchTemplate"]["Properties"]["LaunchTemplateData"]["UserData"][
        "Fn::Base64"
    ]["Sub"]
    values = {**SAMPLE, **overrides}
    conditions = {"UseControllersImage": values["GhcrTokenParameterName"] != ""}
    for name, value in variables.items():
        condition, when_true, when_false = value["If"]
        values[name] = when_true if conditions[condition] else when_false

    def substitute(match: re.Match) -> str:
        name = match.group(1)
        return "${" + name[1:] + "}" if name.startswith("!") else values[name]

    return re.sub(r"\$\{([^}]+)\}", substitute, template)


def statements(resource: str, conditions: frozenset[str] = frozenset({"UseControllersImage"})) -> list[dict]:
    """The role's statements with each Fn::If resolved; by default every condition holds."""
    policies = RESOURCES[resource]["Properties"]["Policies"]
    resolved = []
    for statement in (s for p in policies for s in p["PolicyDocument"]["Statement"]):
        if "If" in statement:
            condition, when_true, when_false = statement["If"]
            statement = when_true if condition in conditions else when_false
            if statement == {"Ref": "AWS::NoValue"}:
                continue
        resolved.append(statement)
    return resolved


def actions(statement: dict) -> list[str]:
    value = statement["Action"]
    return value if isinstance(value, list) else [value]


def test_group_is_zero_to_one_and_starts_at_zero():
    group = RESOURCES["HostAutoScalingGroup"]["Properties"]
    assert (group["MinSize"], group["MaxSize"], group["DesiredCapacity"]) == ("0", "1", "0")


def test_defaults_match_the_plan():
    assert PARAMETERS["InstanceType"]["Default"] == "g6e.xlarge"
    assert PARAMETERS["IdleMinutes"]["Default"] == 15
    assert PARAMETERS["MaxLifetimeMinutes"]["Default"] == 480
    assert PARAMETERS["FmSetupRole"]["Default"] == "sim-cloud"


@pytest.mark.parametrize("name", ["FmSetupCommit", "FmSimCommit"])
def test_fetched_code_is_pinned_to_a_full_commit(name):
    assert PARAMETERS[name]["AllowedPattern"] == "^[0-9a-f]{40}$"
    assert "Default" not in PARAMETERS[name]


def test_sim_image_is_pinned_by_digest():
    pattern = PARAMETERS["SimImageDigest"]["AllowedPattern"]
    assert re.fullmatch(pattern, SAMPLE["SimImageDigest"])
    assert not re.fullmatch(pattern, "ghcr.io/first-motive/fm-sim:jazzy")
    assert "FM_SIM_IMAGE_DIGEST=sha256:" + "c" * 64 in user_data()


def test_host_has_no_ingress_and_requires_imdsv2():
    group = RESOURCES["HostSecurityGroup"]["Properties"]
    assert "SecurityGroupIngress" not in group
    data = RESOURCES["HostLaunchTemplate"]["Properties"]["LaunchTemplateData"]
    assert data["MetadataOptions"]["HttpTokens"] == "required"
    assert data["BlockDeviceMappings"][0]["Ebs"]["Encrypted"] is True


def test_host_role_can_only_lower_capacity_of_its_own_group():
    host = statements("HostRole")
    granted = {a for s in host for a in actions(s)}
    assert granted == {"ssm:GetParameter", "kms:Decrypt", "autoscaling:TerminateInstanceInAutoScalingGroup"}
    leave = next(s for s in host if s["Sid"] == "LeaveOwnGroup")
    assert leave["Resource"]["Sub"].endswith("autoScalingGroupName/${AWS::StackName}-host")
    read_key = next(s for s in host if s["Sid"] == "ReadTailscaleAuthKey")
    assert read_key["Resource"]["Sub"].endswith("parameter${TailscaleAuthKeyParameterName}")


def test_controller_mutations_are_scoped_to_the_group_budget_and_status():
    for statement in statements("ControllerRole"):
        if statement["Resource"] == "*":
            assert set(actions(statement)) == {
                "autoscaling:DescribeAutoScalingGroups",
                "ec2:DescribeInstances",
            }
    group_arn = RESOURCES["HostAutoScalingGroup"]["Properties"]["AutoScalingGroupName"]["Sub"]
    capacity = next(s for s in statements("ControllerRole") if s["Sid"] == "CapacityOfThisGroupOnly")
    assert capacity["Resource"]["Sub"].endswith(f"autoScalingGroupName/{group_arn}")


def test_budget_filters_on_the_cost_tag_and_the_controller_reads_it():
    budget = RESOURCES["MonthlyBudget"]["Properties"]["Budget"]
    assert budget["CostFilters"]["TagKeyValue"] == [{"Sub": "user:${CostTagKey}$${CostTagValue}"}]
    env = RESOURCES["ControllerFunction"]["Properties"]["Environment"]["Variables"]
    assert env["BUDGET_NAME"] == budget["BudgetName"]


def test_lambda_runs_the_shipped_handler_with_the_config_it_reads():
    function = RESOURCES["ControllerFunction"]["Properties"]
    module, attribute = function["Handler"].split(".")
    assert module == "controller" and callable(getattr(controller, attribute))
    environ = {key: "1" for key in function["Environment"]["Variables"]}
    config = controller.Config.from_environ(environ)  # raises if one it needs is missing
    assert (config.idle_minutes, config.max_lifetime_minutes, config.budget_block_percent) == (1, 1, 1)


def test_schedule_sends_a_known_action():
    target = RESOURCES["ReconcileSchedule"]["Properties"]["Targets"][0]
    assert json.loads(target["Input"])["action"] in controller.ACTIONS


def test_bootstrap_never_carries_the_auth_key():
    script = user_data()
    assert "FM_TS_AUTHKEY_SSM_PARAMETER=/fm-sim/cloud-host/tailscale-auth-key" in script
    assert "tskey" not in script
    assert "--sim-cloud" in script


def test_bootstrap_installs_the_idle_units_this_repo_ships():
    script = user_data()
    unit = (SIM_DIR / "fm-sim-idle.service").read_text()
    assert "ExecStart=/usr/bin/python3 -I /usr/local/lib/fm-sim-cloud/idle_to_zero.py check" in unit
    assert "src=/opt/fm-sim-cloud/fm-sim/infra/aws/sim\n" in script
    assert "lib=/usr/local/lib/fm-sim-cloud\n" in script
    assert 'install -d -o root -g root -m 0755 "$lib"' in script
    assert 'install -o root -g root -m 0644 "$src/idle_to_zero.py" "$lib/idle_to_zero.py"' in script
    assert 'owned 755 "$lib"' in script and 'owned 644 "$lib/idle_to_zero.py"' in script
    assert "EnvironmentFile=/etc/fm-sim-cloud/host.env" in unit
    for name in ("FM_SIM_CLOUD_REGION", "FM_SIM_IDLE_MINUTES", "FM_SIM_MAX_LIFETIME_MINUTES"):
        assert f"\n{name}=" in script
    assert "systemctl enable --now fm-sim-idle.timer" in script


def test_bootstrap_parses_and_passes_shellcheck(tmp_path):
    script = tmp_path / "user-data.sh"
    script.write_text(user_data())
    subprocess.run(["bash", "-n", str(script)], check=True)
    if shutil.which("shellcheck") is None:
        pytest.skip("shellcheck not installed")
    subprocess.run(["shellcheck", "--shell=bash", str(script)], check=True)


def test_public_repo_carries_no_account_identifiers():
    for path in SIM_DIR.rglob("*"):
        parts = path.relative_to(SIM_DIR).parts
        if not path.is_file() or any(p.startswith(".") or p == "__pycache__" for p in parts):
            continue
        text = path.read_text()
        if path.name.startswith("test_"):
            continue  # fakes use the all-zero account id
        assert not re.search(r"(?<![0-9a-f])\d{12}(?![0-9a-f])", text), path
        assert not re.search(r"arn:aws:[a-z0-9-]+:[a-z0-9-]*:\d{12}", text), path
        assert not re.search(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[a-z]{2,}", text), path
        assert "tskey-" not in text, path


def test_deploy_plan_makes_no_aws_call(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    called = tmp_path / "aws-called"
    (fake_bin / "aws").write_text(f"#!/bin/sh\ntouch {called}\nexit 1\n")
    (fake_bin / "aws").chmod(0o755)
    params = tmp_path / "params"
    params.write_text("# comment\n\nFmSetupCommit=" + "a" * 40 + "\nMonthlyBudgetUsd=100\n")
    env = {
        **os.environ,
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "FM_SIM_PARAMS": str(params),
        "FM_SIM_CODE_BUCKET": "operator-bucket",
        "FM_SIM_BUILD_DIR": str(tmp_path / "build"),
    }
    for verb in (["plan"], ["deploy"]):
        result = subprocess.run(
            [str(SIM_DIR / "deploy.sh"), *verb], env=env, capture_output=True, text=True, check=True
        )
        assert "Plan only" in result.stdout
        assert "MonthlyBudgetUsd=100" in result.stdout
        assert re.search(r"ControllerCodeKey=fm-sim-cloud-host/controller/[0-9a-f]{64}\.zip", result.stdout)
    assert not called.exists()


def test_controller_zip_is_reproducible(tmp_path):
    env = {**os.environ, "FM_SIM_BUILD_DIR": str(tmp_path)}
    first = subprocess.run(
        [str(SIM_DIR / "deploy.sh"), "package"], env=env, capture_output=True, text=True, check=True
    )
    second = subprocess.run(
        [str(SIM_DIR / "deploy.sh"), "package"], env=env, capture_output=True, text=True, check=True
    )
    assert first.stdout.split()[0] == second.stdout.split()[0]


def test_deploy_refuses_a_malformed_params_line(tmp_path):
    params = tmp_path / "params"
    params.write_text("rm -rf /\n")
    env = {
        **os.environ,
        "FM_SIM_PARAMS": str(params),
        "FM_SIM_CODE_BUCKET": "b",
        "FM_SIM_BUILD_DIR": str(tmp_path),
    }
    result = subprocess.run([str(SIM_DIR / "deploy.sh"), "plan"], env=env, capture_output=True, text=True)
    assert result.returncode != 0
    assert "bad params line" in result.stderr


def test_bootstrap_leaves_the_group_on_any_failure():
    script = user_data()
    trap = script.index("trap 'leave \"$LINENO\"' ERR")
    assert trap < script.index("for tool in")  # armed before the first thing that can fail
    leave = script[script.index("leave() {") : trap]
    assert "terminate-instance-in-auto-scaling-group" in leave
    assert "--should-decrement-desired-capacity" in leave
    assert "X-aws-ec2-metadata-token" in leave


def test_handler_wires_boto3_clients_and_refuses_bad_config(monkeypatch):
    import sys
    import types

    made = []
    fake = types.ModuleType("boto3")
    fake.client = lambda name: made.append(name) or object()
    monkeypatch.setitem(sys.modules, "boto3", fake)
    for key in ("AUTO_SCALING_GROUP", "BUDGET_NAME", "STATUS_PARAMETER", "AWS_ACCOUNT_ID"):
        monkeypatch.delenv(key, raising=False)
    reply = controller.handler({"action": "status"})
    assert reply["error"]["code"] == "controller_error"
    environ = RESOURCES["ControllerFunction"]["Properties"]["Environment"]["Variables"]
    for key in environ:
        monkeypatch.setenv(key, "1")
    assert controller.handler({"action": "scale"})["error"]["code"] == "unknown_action"
    assert sorted(made) == ["autoscaling", "budgets", "ec2", "ssm"]


IMDS_RULE = "OUTPUT -d 169.254.169.254/32 -m owner --uid-owner fm-sim -j DROP"


def test_bootstrap_blocks_imds_for_the_agent_account_before_the_role_runs():
    script = user_data()
    role = script.index("/opt/fm/fm-setup/install.sh")
    enable = script.index("systemctl enable --now fm-sim-imds-block.service")
    assert script.index("useradd --system --user-group") < enable < role  # the rule names the account
    assert script.index(f"iptables -C {IMDS_RULE}") < role
    unit = (SIM_DIR / "fm-sim-imds-block.service").read_text()
    assert (
        f"iptables -C {IMDS_RULE}" in unit
        and f"iptables -I {IMDS_RULE.replace('OUTPUT', 'OUTPUT 1', 1)}" in unit
    )
    assert "Before=docker.service fm-sim-agent.service" in unit
    assert "WantedBy=multi-user.target" in unit


def test_bootstrap_bounds_the_role_with_a_timeout():
    script = user_data()
    assert "NONINTERACTIVE=1 timeout 60m /opt/fm/fm-setup/install.sh '--sim-cloud'" in script
    # timeout exits nonzero on expiry, so the ERR trap, armed earlier, ends the host.
    assert script.index("trap 'leave") < script.index("timeout 60m")


def fake_aws(tmp_path, *, versioning="Enabled", checksum=None, status=None):
    """An aws stand-in: answers the calls deploy.sh makes and logs every argv."""
    log = tmp_path / "aws.log"
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    script = bin_dir / "aws"
    script.write_text(
        "#!/bin/sh\n"
        f'echo "$*" >> {log}\n'
        'case "$*" in\n'
        f'  *get-bucket-versioning*) echo "{versioning}" ;;\n'
        f'  *head-object*) echo "{checksum or ""}" ;;\n'
        f"  *get-parameter*) echo '{status or ''}' ;;\n"
        "esac\n"
    )
    script.chmod(0o755)
    return bin_dir, log


def deploy_env(tmp_path, bin_dir):
    params = tmp_path / "params"
    params.write_text("MonthlyBudgetUsd=100\n")
    return {
        **os.environ,
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "FM_AWS_PROFILE": "p",
        "FM_AWS_REGION": "us-test-1",
        "FM_SIM_PARAMS": str(params),
        "FM_SIM_CODE_BUCKET": "operator-bucket",
        "FM_SIM_BUILD_DIR": str(tmp_path / "build"),
    }


def run_deploy(env, *args):
    return subprocess.run([str(SIM_DIR / "deploy.sh"), *args], env=env, capture_output=True, text=True)


def zip_checksum(env) -> str:
    digest = run_deploy(env, "package").stdout.split()[0]
    return base64.b64encode(bytes.fromhex(digest)).decode()


def test_deploy_refuses_an_unversioned_bucket(tmp_path):
    bin_dir, log = fake_aws(tmp_path, versioning="None")
    result = run_deploy(deploy_env(tmp_path, bin_dir), "deploy", "--execute")
    assert result.returncode != 0 and "enable versioning" in result.stderr
    assert "cloudformation" not in log.read_text() and "s3 cp" not in log.read_text()


def test_deploy_refuses_a_checksum_that_does_not_match_the_key(tmp_path):
    bin_dir, log = fake_aws(tmp_path, checksum="AAAA")
    result = run_deploy(deploy_env(tmp_path, bin_dir), "deploy", "--execute")
    assert result.returncode != 0 and "not the SHA-256 its key names" in result.stderr
    assert "cloudformation" not in log.read_text()


def test_deploy_uploads_with_a_checksum_and_deploys_when_it_matches(tmp_path):
    checksum = zip_checksum({**os.environ, "FM_SIM_BUILD_DIR": str(tmp_path / "probe")})
    bin_dir, log = fake_aws(tmp_path, checksum=checksum)
    result = run_deploy(deploy_env(tmp_path, bin_dir), "deploy", "--execute")
    assert result.returncode == 0, result.stderr
    calls = log.read_text()
    assert "s3 cp" in calls and "--checksum-algorithm SHA256" in calls
    assert "--checksum-mode ENABLED" in calls
    assert "cloudformation deploy --stack-name fm-sim-cloud-host" in calls


@pytest.mark.parametrize(
    "status, warned",
    [
        ({"budget": {"actual_usd": 0.0}, "instances": [{"instance_id": "i-0abc"}]}, True),
        ({"budget": {"actual_usd": 0.0}, "instances": []}, False),
        ({"budget": {"actual_usd": 12.5}, "instances": [{"instance_id": "i-0abc"}]}, False),
    ],
)
def test_status_warns_when_spend_reads_zero_with_a_host_up(tmp_path, status, warned):
    bin_dir, _ = fake_aws(tmp_path, status=json.dumps(status))
    result = run_deploy(deploy_env(tmp_path, bin_dir), "status")
    assert result.returncode == 0
    assert json.loads(result.stdout) == status
    assert ("cost tag active" in result.stderr) is warned


def test_bootstrap_bounds_every_repo_fetch():
    script = user_data()
    fetch = script[script.index("fetch_commit() {") : script.index("fetch_commit /opt/fm-sim-cloud")]
    assert 'timeout 5m git -C "$1" fetch -q --depth 1 "$2" "$3"' in fetch
    assert fetch.count('git -C "$1" fetch') == 1


def test_bootstrap_refuses_a_host_with_global_ipv6():
    script = user_data()
    assert "if ip -6 addr show scope global | grep inet6 >/dev/null; then" in script
    assert script.index("ip -6 addr show") < script.index("/opt/fm/fm-setup/install.sh")
    data = RESOURCES["HostLaunchTemplate"]["Properties"]["LaunchTemplateData"]
    assert data["MetadataOptions"]["HttpProtocolIpv6"] == "disabled"
    assert not any(r["Type"] == "AWS::EC2::VPCCidrBlock" for r in RESOURCES.values())


@pytest.mark.parametrize("policy", ["OperatorPolicy", "ViewerPolicy"])
def test_operator_and_viewer_can_read_only_this_stack_outputs(policy):
    document = RESOURCES[policy]["Properties"]["PolicyDocument"]["Statement"]
    describe = [s for s in document if "cloudformation:DescribeStacks" in actions(s)]
    assert describe == [
        {"Effect": "Allow", "Action": "cloudformation:DescribeStacks", "Resource": {"Ref": "AWS::StackId"}}
    ]
    assert not any(
        a.startswith("cloudformation:") and a != "cloudformation:DescribeStacks"
        for s in document
        for a in actions(s)
    )


def test_controllers_parameters_default_to_the_public_image():
    for name in ("GhcrTokenParameterName", "GhcrUsername", "ControllersImageDigest"):
        assert PARAMETERS[name]["Default"] == ""
        assert re.fullmatch(PARAMETERS[name]["AllowedPattern"], "")
        assert re.fullmatch(PARAMETERS[name]["AllowedPattern"], CONTROLLERS[name])
    assert not re.fullmatch(PARAMETERS["ControllersImageDigest"]["AllowedPattern"], "ghcr.io/x:latest")
    assert not re.fullmatch(PARAMETERS["GhcrTokenParameterName"]["AllowedPattern"], "ghcr-token")
    assert not re.fullmatch(PARAMETERS["GhcrUsername"]["AllowedPattern"], "-leading-dash")
    condition = TEMPLATE["Conditions"]["UseControllersImage"]
    assert condition == {"Not": [{"Equals": [{"Ref": "GhcrTokenParameterName"}, ""]}]}


def test_a_token_name_requires_a_username():
    rule = TEMPLATE["Rules"]["GhcrTokenNeedsUsername"]
    assert rule["RuleCondition"] == {"Not": [{"Equals": [{"Ref": "GhcrTokenParameterName"}, ""]}]}
    assert rule["Assertions"][0]["Assert"] == {"Not": [{"Equals": [{"Ref": "GhcrUsername"}, ""]}]}


def test_host_env_always_names_the_controllers_choice():
    public = user_data()
    for line in (
        "FM_SIM_CONTROLLERS=0\n",
        "FM_SIM_CONTROLLERS_IMAGE_DIGEST=\n",
        "FM_GHCR_TOKEN_SSM_PARAMETER=\n",
        "FM_GHCR_USERNAME=\n",
    ):
        assert f"\n{line}" in public
    private = user_data(**CONTROLLERS)
    assert "\nFM_SIM_CONTROLLERS=1\n" in private
    assert "\nFM_SIM_CONTROLLERS_IMAGE_DIGEST=sha256:" + "d" * 64 + "\n" in private
    assert "\nFM_GHCR_TOKEN_SSM_PARAMETER=/fm-sim/cloud-host/ghcr-token\n" in private
    assert "\nFM_GHCR_USERNAME=fm-operator\n" in private
    assert "ghp_" not in private and "github_pat_" not in private


def test_host_role_reads_the_ghcr_token_only_when_one_is_named():
    public = {s["Sid"] for s in statements("HostRole", conditions=frozenset())}
    assert not {"ReadGhcrToken", "DecryptGhcrTokenThroughSsm"} & public
    host = statements("HostRole")
    read = next(s for s in host if s["Sid"] == "ReadGhcrToken")
    assert read["Action"] == "ssm:GetParameter"
    assert read["Resource"]["Sub"].endswith("parameter${GhcrTokenParameterName}")
    decrypt = next(s for s in host if s["Sid"] == "DecryptGhcrTokenThroughSsm")
    assert decrypt["Action"] == "kms:Decrypt"
    equals = decrypt["Condition"]["StringEquals"]
    assert equals["kms:ViaService"] == {"Sub": "ssm.${AWS::Region}.amazonaws.com"}
    assert equals["kms:EncryptionContext:PARAMETER_ARN"] == read["Resource"]


def test_bake_is_read_from_an_instance_tag_the_template_exposes():
    options = RESOURCES["HostLaunchTemplate"]["Properties"]["LaunchTemplateData"]["MetadataOptions"]
    assert options["InstanceMetadataTags"] == "enabled"
    assert options["HttpTokens"] == "required"
    script = user_data()
    assert "meta-data/tags/instance/fm-sim-bake" in script
    # Read before the trap is armed, so a failed read never takes the group path.
    assert script.index("tags/instance/fm-sim-bake") < script.index("trap 'leave") < script.index("host.env <<")
    # Only 200 or 404 count; any other answer ends the bootstrap before a step runs.
    read = script[script.index("bake_status=") : script.index("trap 'leave")]
    assert "200)" in read and "404)" in read and "exit 1" in read


def test_bake_skips_only_the_tailnet_join_and_the_readiness_check():
    script = user_data()
    assert "--skip tailscale-authkey,sim-cloud-ready" in script
    assert script.count("--skip") == 1
    # A launch from the group still runs the whole role.
    assert "\n  NONINTERACTIVE=1 timeout 60m /opt/fm/fm-setup/install.sh '--sim-cloud'\n" in script


def test_bake_clears_the_instance_identity_and_stops_before_the_idle_timer():
    script = user_data()
    bake = script[script.index("--skip tailscale-authkey") : script.index("systemctl enable --now fm-sim-idle.timer")]
    for step in (
        "systemctl disable --now fm-sim-agent.service fm-zenoh-bridge.service",
        "rm -f /var/lib/tailscale/tailscaled.state /etc/fm-sim-cloud/host.env",
        "journalctl --vacuum-time=1s",
        "cloud-init clean --logs",
        "shutdown -h now",
    ):
        assert step in bake, step
    assert "machine-id" not in bake.replace("/etc/machine-id stays", "")
    leave = script[script.index("leave() {") : script.index("trap 'leave")]
    # A failed bake stops instead of asking a group it is not in.
    assert leave.index('if [ "$bake" = 1 ]') < leave.index("terminate-instance-in-auto-scaling-group")


def fake_bake_aws(tmp_path, *, console, state="stopped", left="", full=(), denied=()):
    log = tmp_path / "aws.log"
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    (bin_dir / "aws").write_text(
        "#!/bin/sh\n"
        f'echo "$*" >> {log}\n'
        'case "$*" in\n'
        f"  *tag:fm-sim-bake*) echo '{left}' ;;\n"
        "  *HostLaunchTemplate*) echo lt-0abc ;;\n"
        "  *SubnetA*) echo subnet-0abc ;;\n"
        "  *SubnetB*) echo subnet-0def ;;\n"
        "  *HostSecurityGroup*) echo sg-0abc ;;\n"
        "  *HostName*) echo sim-cloud ;;\n"
        "  *CostTagKey*) echo fm-cost ;;\n"
        "  *CostTagValue*) echo sim ;;\n"
        + "".join(
            f"  *run-instances*SubnetId={subnet},*) echo 'InsufficientInstanceCapacity' >&2; exit 254 ;;\n"
            for subnet in full
        )
        + "".join(
            f"  *run-instances*SubnetId={subnet},*) echo 'UnauthorizedOperation' >&2; exit 254 ;;\n"
            for subnet in denied
        )
        + "  *run-instances*) echo i-0bake ;;\n"
        f"  *describe-instances*) echo {state} ;;\n"
        f"  *get-console-output*) echo '{console}' ;;\n"
        "  *create-image*) echo ami-0baked ;;\n"
        "  *describe-images*) echo available ;;\n"
        "esac\n"
    )
    (bin_dir / "aws").chmod(0o755)
    env = {
        **os.environ,
        "PATH": f"{bin_dir}:{os.environ['PATH']}",
        "FM_AWS_PROFILE": "p",
        "FM_AWS_REGION": "us-test-1",
        "FM_SIM_POLL_SECONDS": "0",
    }
    return env, log


def test_bake_launches_outside_the_group_images_the_disk_and_terminates(tmp_path):
    env, log = fake_bake_aws(tmp_path, console="... bake complete")
    result = run_deploy(env, "bake")
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "ami-0baked"
    calls = log.read_text()
    run = next(line for line in calls.splitlines() if "run-instances" in line)
    assert "LaunchTemplateId=lt-0abc" in run and "SubnetId=subnet-0abc" in run
    assert "--instance-initiated-shutdown-behavior stop" in run
    assert "Key=fm-sim-bake,Value=1" in run and "Key=fm-cost,Value=sim" in run
    assert "autoscaling" not in calls
    assert calls.index("create-image --instance-id i-0bake") < calls.index("terminate-instances --instance-ids i-0bake")


def test_bake_terminates_and_makes_no_image_when_the_role_did_not_finish(tmp_path):
    env, log = fake_bake_aws(tmp_path, console="bake failed at line 42; stopping")
    result = subprocess.run(
        [str(SIM_DIR / "deploy.sh"), "bake"],
        env={**env, "FM_SIM_BAKE_CONSOLE_MINUTES": "0"},
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "without finishing the bake" in result.stderr
    calls = log.read_text()
    assert "create-image" not in calls
    assert "terminate-instances --instance-ids i-0bake" in calls


def test_bake_makes_no_image_when_the_instance_did_not_stop(tmp_path):
    env, log = fake_bake_aws(tmp_path, console="bake complete", state="terminated")
    result = run_deploy(env, "bake")
    assert result.returncode != 0 and "did not stop" in result.stderr
    calls = log.read_text()
    assert "create-image" not in calls
    assert "terminate-instances --instance-ids i-0bake" in calls


def test_bake_refuses_while_an_earlier_bake_instance_exists(tmp_path):
    env, log = fake_bake_aws(tmp_path, console="bake complete", left="i-0old")
    result = run_deploy(env, "bake")
    assert result.returncode != 0 and "i-0old" in result.stderr
    assert "run-instances" not in log.read_text()


def test_bake_moves_to_the_other_subnet_when_a_zone_has_no_capacity(tmp_path):
    env, log = fake_bake_aws(tmp_path, console="bake complete", full=("subnet-0abc",))
    result = run_deploy(env, "bake")
    assert result.returncode == 0, result.stderr
    assert "no capacity in SubnetA" in result.stderr
    runs = [line for line in log.read_text().splitlines() if "run-instances" in line]
    assert ["SubnetId=subnet-0abc" in runs[0], "SubnetId=subnet-0def" in runs[1]] == [True, True]


def test_bake_stops_when_neither_zone_has_capacity(tmp_path):
    env, log = fake_bake_aws(tmp_path, console="bake complete", full=("subnet-0abc", "subnet-0def"))
    result = run_deploy(env, "bake")
    assert result.returncode != 0 and "no capacity in either" in result.stderr
    assert "create-image" not in log.read_text()


def test_bake_stops_at_once_on_any_other_launch_error(tmp_path):
    env, log = fake_bake_aws(tmp_path, console="bake complete", denied=("subnet-0abc",))
    result = run_deploy(env, "bake")
    assert result.returncode != 0
    assert "UnauthorizedOperation" in result.stderr and "run-instances failed" in result.stderr
    assert sum("run-instances" in line for line in log.read_text().splitlines()) == 1


def test_bake_can_run_on_another_instance_type(tmp_path):
    env, log = fake_bake_aws(tmp_path, console="bake complete")
    result = run_deploy({**env, "FM_SIM_BAKE_INSTANCE_TYPE": "g6.xlarge"}, "bake")
    assert result.returncode == 0, result.stderr
    run = next(line for line in log.read_text().splitlines() if "run-instances" in line)
    assert "--instance-type g6.xlarge" in run
