"""The zenoh bridge a run gets when it is off the host network (a Mac).

On a Linux sim host a run shares the host's network, and the host's own sim
bridge (fm-comms' ``fm-zenoh-bridge-sim.service``) routes its topics to the
router. On a Mac the agent runs with ``--no-host-network``: a run's DDS stays
inside its container, where no bridge on the Mac can see it. So beside each such
run the agent starts one more container, of the pinned bridge image, in the
run's network namespace (``--network container:<run>``), and removes it with
the run.

The bridge reads two files fm-comms renders for this machine: its
``bridge-sim.json5`` (router endpoint, the ``/fm_<host>`` namespace, the domain,
the sim topic allowlist) and the ``cyclonedds.xml`` beside it. Only those two
are mounted, read-only. Without them a run starts as before, with no live
topics; :func:`missing` says which file is absent. The container itself is
:class:`~fm_sim_agent.runner.DockerRunner`'s to start and remove.
"""

from __future__ import annotations

import csv
import io
import os
from pathlib import Path

#: zenoh-bridge-ros2dds 1.9.0, fm-comms' ``zenoh.version``, pinned to its
#: multi-arch index so a Mac runs exactly what the fleet's bridges run.
#: ``fm sim host pull local`` pulls it; a run starts it with ``--pull=never``.
IMAGE = "eclipse/zenoh-bridge-ros2dds@sha256:5aca6798ae112e390a97788450fd37360ddd72ac092032a25acf9fac17b2d9af"
#: Overrides where the bridge config is read from (also the service's ``--bridge-config``).
CONFIG_ENV = "FM_SIM_BRIDGE_CONFIG"
DEFAULT_CONFIG = "~/.config/fm-comms/bridge-sim.json5"
#: The Cyclone profile the bridge joins the run's DDS with, read from beside the config.
CYCLONE_NAME = "cyclonedds.xml"

CONFIG_MOUNT = "/fm-bridge/bridge-sim.json5"
CYCLONE_MOUNT = "/fm-bridge/cyclonedds.xml"
#: The image is built for several ROS distros and picks by this; runs are Jazzy.
ROS_DISTRO = "jazzy"

#: tradeoff: a bridge is one process with a few dozen threads and a small
#: cache; these keep a runaway one from taking the host and leave it room.
PIDS_LIMIT = 256
MEMORY = "512m"


def config_path(flag: str | Path | None = None) -> Path:
    """The bridge config: ``flag``, else ``$FM_SIM_BRIDGE_CONFIG``, else ``DEFAULT_CONFIG``."""
    chosen = flag or os.environ.get(CONFIG_ENV, "").strip() or DEFAULT_CONFIG
    return Path(chosen).expanduser().resolve()


def cyclone_path(config: Path) -> Path:
    return config.with_name(CYCLONE_NAME)


def missing(config: Path) -> list[Path]:
    """The bridge files that are not regular files here; empty when the bridge can run."""
    return [path for path in (config, cyclone_path(config)) if not path.is_file()]


def active_config(flag: str | Path | None = None) -> Path | None:
    """The bridge config runs read when both its files are here (see :func:`config_path`), else None."""
    config = config_path(flag)
    return None if missing(config) else config


def mount(source: Path, target: str) -> str:
    """A read-only bind ``--mount`` value. Docker reads it as one CSV record, so
    each field is quoted as CSV needs: a ``,`` or ``:`` in a path stays the path's."""
    fields = ["type=bind", f"source={source}", f"target={target}", "readonly"]
    out = io.StringIO()
    csv.writer(out, lineterminator="").writerow(fields)
    return out.getvalue()
