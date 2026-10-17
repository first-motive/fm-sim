"""Return the AWS sim host to zero after it has held no run for the idle period.

A systemd timer runs ``check`` once a minute on the host. A run is active while
a running container carries the sim agent's ``fm.sim.run_id`` label, which is
exactly when the agent's status holds a run. The catalog fetch container has no
run label, so it does not count. When no run has been seen for
``FM_SIM_IDLE_MINUTES``, the host terminates itself through its Auto Scaling
group and decrements desired capacity, so the group stays at zero.

Each check also makes sure the iptables rule that keeps the agent's account off
the instance metadata service is still in place, re-inserts it when it is not,
and terminates the host if it cannot.

The same check leaves once the host has been up ``FM_SIM_MAX_LIFETIME_MINUTES``,
busy or not. The controller enforces that deadline from outside; this is the
second line, for when its schedule does not run.

The host's role may only terminate an instance in its own group with a decrement.
It cannot raise capacity; only the controller, behind the budget gate, can.

Standard library only: the host runs this with the system ``python3 -I``.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import urllib.request
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Callable

RUN_LABEL = "fm.sim.run_id"  # fm_sim_agent.runner.RUN_LABEL
DEFAULT_STATE_FILE = Path("/var/lib/fm-sim-cloud/idle.json")
IMDS = "http://169.254.169.254/latest"
# The rule fm-sim-imds-block.service applies; re-checked here every minute.
IMDS_BLOCK_RULE = ["OUTPUT", "-d", "169.254.169.254/32", "-m", "owner", "--uid-owner", "fm-sim", "-j", "DROP"]
COMMAND_TIMEOUT_S = 30

Run = Callable[[list[str]], subprocess.CompletedProcess]


def _run(argv: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(argv, capture_output=True, text=True, timeout=COMMAND_TIMEOUT_S, check=False)


def active_run(run: Run = _run) -> bool | None:
    """True while a run container is up, False when none is, None when Docker cannot say."""
    try:
        result = run(
            ["docker", "ps", "--quiet", "--filter", f"label={RUN_LABEL}", "--filter", "status=running"]
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    return bool(result.stdout.split())


def decide(
    now: datetime, last_active: datetime | None, active: bool | None, idle_minutes: int
) -> tuple[datetime, bool]:
    """Return the new last-active time and whether the host should leave.

    The first check after boot has no last-active time and starts the clock,
    which gives a freshly provisioned host one full idle period to be used.
    A Docker that cannot answer is not evidence of a run, so the clock keeps
    running: a host whose Docker is broken for the whole idle period cannot
    serve a run and should not keep billing.
    """
    # tradeoff: a one-minute poll, so the host leaves up to a minute after the
    # idle period; a tighter bound would need the agent to push its state here.
    if active or last_active is None:
        return now, False
    return last_active, now - last_active >= timedelta(minutes=idle_minutes)


def read_last_active(path: Path) -> datetime | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))["last_active"]
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        return None
    return parsed if parsed.tzinfo else None


def write_last_active(path: Path, when: datetime) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps({"last_active": when.strftime("%Y-%m-%dT%H:%M:%SZ")}), encoding="utf-8")
    temporary.replace(path)


def instance_id(opener: Callable = urllib.request.urlopen) -> str:
    """This instance's id over IMDSv2, which the launch template requires."""
    token_request = urllib.request.Request(
        f"{IMDS}/api/token", method="PUT", headers={"X-aws-ec2-metadata-token-ttl-seconds": "60"}
    )
    with opener(token_request, timeout=5) as response:
        token = response.read().decode()
    id_request = urllib.request.Request(
        f"{IMDS}/meta-data/instance-id", headers={"X-aws-ec2-metadata-token": token}
    )
    with opener(id_request, timeout=5) as response:
        value = response.read().decode().strip()
    if not value.startswith("i-") or not value[2:].isalnum():
        raise RuntimeError("IMDS returned an unexpected instance id")
    return value


def terminate_self(region: str, own_id: str, run: Run = _run) -> bool:
    result = run(
        [
            "aws", "--region", region, "autoscaling", "terminate-instance-in-auto-scaling-group",
            "--instance-id", own_id, "--should-decrement-desired-capacity",
        ]
    )  # fmt: skip
    if result.returncode != 0:
        print(f"terminate failed: {result.stderr.strip()}", file=sys.stderr)
    return result.returncode == 0


def imds_block_held(run: Run = _run) -> bool:
    """Re-insert the agent account's IMDS block when it has gone; False when that fails."""
    try:
        if run(["iptables", "-C", *IMDS_BLOCK_RULE]).returncode == 0:
            return True
        print("IMDS block for fm-sim is missing; re-inserting it", file=sys.stderr)
        return run(["iptables", "-I", IMDS_BLOCK_RULE[0], "1", *IMDS_BLOCK_RULE[1:]]).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def read_uptime_s(path: Path = Path("/proc/uptime")) -> float:
    return float(path.read_text(encoding="utf-8").split()[0])


def check(
    *,
    region: str,
    idle_minutes: int,
    max_lifetime_minutes: int,
    state_file: Path,
    now: datetime,
    uptime_s: float,
    run: Run = _run,
    get_instance_id: Callable[[], str] = instance_id,
) -> int:
    if not imds_block_held(run):
        print("could not restore the IMDS block; terminating rather than expose the host role")
        return 0 if terminate_self(region, get_instance_id(), run) else 1
    if uptime_s >= max_lifetime_minutes * 60:
        print(f"up {uptime_s / 60:.0f} minutes, past the {max_lifetime_minutes}-minute lifetime; terminating")
        return 0 if terminate_self(region, get_instance_id(), run) else 1
    active = active_run(run)
    if active is None:
        print("docker did not answer; counting this minute as idle", file=sys.stderr)
    last_active, leave = decide(now, read_last_active(state_file), active, idle_minutes)
    write_last_active(state_file, last_active)
    if not leave:
        return 0
    print(f"no run since {last_active.isoformat()}; terminating to zero")
    return 0 if terminate_self(region, get_instance_id(), run) else 1


def _positive_minutes(name: str) -> int | None:
    try:
        value = int(os.environ.get(name, ""))
    except ValueError:
        value = 0
    if value < 1:
        print(f"{name} must be a positive integer", file=sys.stderr)
        return None
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="run once from the timer")
    parser.parse_args(argv)

    region = os.environ.get("FM_SIM_CLOUD_REGION", "")
    if not region:
        print("FM_SIM_CLOUD_REGION is not set", file=sys.stderr)
        return 2
    idle_minutes = _positive_minutes("FM_SIM_IDLE_MINUTES")
    max_lifetime_minutes = _positive_minutes("FM_SIM_MAX_LIFETIME_MINUTES")
    if idle_minutes is None or max_lifetime_minutes is None:
        return 2
    state_file = Path(os.environ.get("FM_SIM_IDLE_STATE_FILE", str(DEFAULT_STATE_FILE)))
    return check(
        region=region,
        idle_minutes=idle_minutes,
        max_lifetime_minutes=max_lifetime_minutes,
        state_file=state_file,
        now=datetime.now(timezone.utc),
        uptime_s=read_uptime_s(),
    )


if __name__ == "__main__":
    sys.exit(main())
