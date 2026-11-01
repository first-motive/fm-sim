"""Isaac Sim beside a run whose backend is ``isaac``.

The fm-sim image does not carry Isaac Sim: NVIDIA ships it as its own image,
which fm-setup's ``isaac-sim`` step pulls. So for an isaac run the agent starts
that image beside the run, and removes it with the run::

    run container (fm-sim)       TMPDIR=STAGE_MOUNT, so run.launch.py writes the stage there
    Isaac container (isaac-sim)  waits for that stage.json, then runs fm_sim_isaac.py on it

The two share one per-run directory, the run's ``stage/``, mounted at the same
path in both. The Isaac container reads the fm-sim image's own ``/ws`` and
``/ws_ctl`` (the stage runner, the robot's meshes) through read-only image
mounts at the same paths, so every absolute path in the stage resolves alike.

It runs as the agent's own user, as a run does, with every capability dropped:
the stage and the caches are the agent's, mode 0700, and nothing is
world-writable. Isaac Sim's image assumes root, so its home and the three
directories it writes under ``/isaac-sim/kit`` are mounted from the cache.

It joins the run's DDS graph on CycloneDDS, through Isaac Sim's bundled Humble
libraries, with the discovery Jazzy's LOCALHOST range uses (``CYCLONE``). On
Isaac's default Fast DDS its memory grew by about 40 MB a physics step once the
run's CycloneDDS subscribed to its cameras, until the host ran out.

The stage runner writes ``stage/isaac.ready`` once its stage plays, which the
runner waits for before the run counts as up, and its output goes to
``stage/isaac.log``, kept with the run once the container is removed.

With ``--assets-dir``, the USD assets a scenario names are mounted read-only into
this container alone: only Isaac Sim opens them.
"""

from __future__ import annotations

import os
import stat
from pathlib import Path

#: The run's directory both containers share, under the run's own directory.
STAGE_NAME = "stage"
STAGE_MOUNT = "/fm-run/stage"
LOG_NAME = "isaac.log"
READY_NAME = "isaac.ready"
CYCLONE_NAME = "cyclonedds.xml"

#: The stage runner, as the fm-sim image installs it (read through the /ws mount).
RUNNER = "/ws/install/fm_sim_backends/share/fm_sim_backends/isaac/fm_sim_isaac.py"
#: The fm-sim image's workspaces the stage names absolute paths in. ``ws_ctl``
#: is the controllers image's overlay (the OpenArm description's meshes); the
#: public image has none, so a mount is made only for a workspace the image has.
WORKSPACES = ("ws", "ws_ctl")

#: Jazzy's rmw_cyclonedds config for ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST,
#: which Humble's libraries cannot select by name, with 120 participant slots
#: as the runner's own probes use (PROBE_CYCLONE): a run has more nodes than
#: Jazzy's default 32.
CYCLONE = (
    '<CycloneDDS><Domain Id="any"><General><AllowMulticast>false</AllowMulticast></General>'
    "<Discovery><ParticipantIndex>auto</ParticipantIndex>"
    "<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>"
    '<Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>'
)

#: Where ``--assets-dir`` is mounted, and the variable that tells the stage runner.
ASSETS_MOUNT = "/fm-assets"
ASSETS_DIR_ENV = "FM_ISAAC_ASSETS_DIR"

#: Isaac Sim's home inside its container, mounted from the cache.
HOME_MOUNT = "/fm-isaac/home"
#: Isaac Sim's own ROS 2 libraries, which its bridge loads with no ROS sourced.
ENV = (
    f"HOME={HOME_MOUNT}",
    "ROS_DISTRO=humble",
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
    "LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib",
    f"CYCLONEDDS_URI=file://{STAGE_MOUNT}/{CYCLONE_NAME}",
    f"FM_ISAAC_READY_FILE={STAGE_MOUNT}/{READY_NAME}",
)
#: The image refuses to start without both. The agent asserts them on the
#: operator's behalf: running this image at all follows the acceptance recorded
#: in fm-setup's manifest (FM_ISAAC_ACCEPT_EULA, FM_ISAAC_PRIVACY_CONSENT).
CONSENT_ENV = ("ACCEPT_EULA=Y", "PRIVACY_CONSENT=Y")

#: What persists between runs: ``(cache subdirectory, container path)``. The
#: home holds the shader, compute, warp and Omniverse caches; the rest are the
#: image's root-owned paths Isaac Sim writes. A cold start compiles shaders for
#: about three minutes; a warm one is ready in seconds.
CACHES = (
    ("home", HOME_MOUNT),
    ("kit-cache", "/isaac-sim/kit/cache"),
    ("kit-logs", "/isaac-sim/kit/logs"),
    ("kit-data", "/isaac-sim/kit/data"),
)

#: tradeoff: one run's Isaac settles near 6 GiB; the ceiling leaves it room and
#: keeps a runaway one from taking the host. 8192 processes covers its threads.
MEMORY = "24g"
PIDS_LIMIT = 8192
#: How long the Isaac container waits for the run to write its stage.
STAGE_WAIT_S = 300
#: tradeoff: an agent that runs Isaac waits this long for any run to come up,
#: since a cold Isaac start compiles shaders for about three minutes. A run on
#: another backend that never comes up fails that much later.
STARTUP_TIMEOUT_S = 480.0
#: Stopping a stepping Isaac Sim lets it finish writing its caches.
STOP_TIMEOUT_S = 20

#: Waits for the stage, then runs the stage runner on it with its output kept.
#: Positional arguments: the wait in seconds, the stage directory, the runner.
#: The stage directory is the agent's, mode 0700, so only the run writes there.
SCRIPT = (
    'for _ in $(seq "$1"); do '
    'for spec in "$2"/fm_sim_world_*/stage.json; do '
    f'[ -f "$spec" ] && exec /isaac-sim/python.sh "$3" --spec "$spec" >"$2/{LOG_NAME}" 2>&1; '
    "done; sleep 1; done; "
    'echo "no stage under $2" >&2; exit 1'
)


class IsaacError(Exception):
    """A stage or cache directory the agent cannot hand to Isaac Sim safely."""


def stage_dir(runs_dir: Path, run_id: str) -> Path:
    return runs_dir / run_id / STAGE_NAME


def _private_dir(path: Path) -> None:
    """Make ``path`` (and missing parents) a directory only this user can enter.

    A path that is a symlink, another user's, or open to the group or others is
    refused rather than changed: Isaac Sim loads code from these directories.
    """
    path.mkdir(mode=0o700, parents=True, exist_ok=True)
    info = path.lstat()
    if not stat.S_ISDIR(info.st_mode):
        raise IsaacError(f"{path} is not a directory")
    if info.st_uid != os.getuid():
        raise IsaacError(f"{path} belongs to uid {info.st_uid}, not the agent's {os.getuid()}")
    if info.st_mode & 0o077:
        raise IsaacError(f"{path} is open to other users (mode {stat.S_IMODE(info.st_mode):o}); make it 0700")


def prepare(stage: Path, cache: Path) -> None:
    """Create the stage with the Cyclone profile in it, and the caches, all the agent's own."""
    _private_dir(stage)
    _private_dir(cache)
    for name, _ in CACHES:
        _private_dir(cache / name)
    profile = os.open(stage / CYCLONE_NAME, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
    with os.fdopen(profile, "w") as out:
        out.write(CYCLONE)


def ready(stage: Path) -> bool:
    return (stage / READY_NAME).is_file()


def argv(
    *,
    name: str,
    labels: tuple[str, ...],
    isaac_image: str,
    sim_image: str,
    workspaces: tuple[str, ...],
    stage: Path,
    cache: Path,
    network: tuple[str, ...],
    memory: str = MEMORY,
    assets: Path | None = None,
) -> list[str]:
    """The ``docker run`` arguments for a run's Isaac container."""
    command = [
        "run", "--detach", "--pull=never", "--name", name,
        *(arg for label in labels for arg in ("--label", label)),
        *network,
        "--gpus", "all",
        "--user", f"{os.getuid()}:{os.getgid()}",
        "--cap-drop", "ALL",
        "--security-opt", "no-new-privileges",
        "--pids-limit", str(PIDS_LIMIT),
        "--memory", memory, "--memory-swap", memory,
        "--volume", f"{stage}:{STAGE_MOUNT}",
        # The URDF importer writes its material layers to the working directory,
        # and the image's own, /isaac-sim, is root's.
        "--workdir", STAGE_MOUNT,
    ]  # fmt: skip
    for host_name, path in CACHES:
        command += ["--volume", f"{cache / host_name}:{path}"]
    for workspace in workspaces:
        command += [
            "--mount",
            f"type=image,source={sim_image},target=/{workspace},image-subpath={workspace}",
        ]
    if assets is not None:
        command += ["--volume", f"{assets}:{ASSETS_MOUNT}:ro", "--env", f"{ASSETS_DIR_ENV}={ASSETS_MOUNT}"]
    for value in (*CONSENT_ENV, *ENV):
        command += ["--env", value]
    if os.environ.get("ROS_DOMAIN_ID"):
        command += ["--env", "ROS_DOMAIN_ID"]  # as the run gets it
    return [
        *command,
        "--entrypoint", "/bin/bash",
        isaac_image, "-c", SCRIPT, "isaac-stage", str(STAGE_WAIT_S), STAGE_MOUNT, RUNNER,
    ]  # fmt: skip
