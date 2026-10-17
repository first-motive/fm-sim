"""This host's name, read from its identity card.

The same card fm-robot-agent reads (``fm machine init`` in fm-setup writes it;
nothing here ever does), found the same way: ``FM_MACHINE_FILE`` first, then
``~/.config/fm/machine.json`` on macOS and ``/etc/fm/machine.json`` elsewhere.
A sim host may hold any role — a workstation, a cloud box — so only the name is
read. The host segment of every key is derived from it, never stored.
"""

from __future__ import annotations

import json
import os
import platform
import socket
from pathlib import Path

from fm_sim_agent.spec import valid_name

SCHEMA_VERSION = 1
ENV_OVERRIDE = "FM_MACHINE_FILE"
LINUX_PATH = Path("/etc/fm/machine.json")
MACOS_RELATIVE = Path("fm/machine.json")


class CardError(Exception):
    """This host's card is absent, unreadable, or names no usable host."""


def card_path() -> Path:
    override = os.environ.get(ENV_OVERRIDE, "").strip()
    if override:
        return Path(override).expanduser()
    if platform.system() == "Darwin":
        config_home = os.environ.get("XDG_CONFIG_HOME", "").strip()
        base = Path(config_home) if config_home else Path.home() / ".config"
        return base / MACOS_RELATIVE
    return LINUX_PATH


def host_segment(name: str) -> str:
    """The key segment a machine name derives (``fm-ws-01`` → ``fm_ws_01``), or raise.

    The segment sits inside a key expression, where ``*`` and ``/`` are syntax:
    a name carrying either would widen this host's verbs over every host's.
    """
    segment = name.replace("-", "_")
    if not valid_name(segment):
        raise CardError(f"host name {name!r} does not derive a valid key segment")
    return segment


def read_host(path: Path | None = None) -> str:
    """This host's key segment, or raise :class:`CardError` explaining why not."""
    path = path or card_path()
    if not path.is_file():
        raise CardError(f"{path} is missing or not a file; run `fm machine init` on this host or pass --host")
    try:
        card = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CardError(f"{path} is not readable as JSON: {exc}") from exc
    if not isinstance(card, dict):
        raise CardError(f"{path} is not a machine identity card")
    if card.get("schema_version") != SCHEMA_VERSION:
        raise CardError(
            f"{path} is schema_version {card.get('schema_version')!r}; this build reads {SCHEMA_VERSION}"
        )
    name = card.get("name")
    if not isinstance(name, str) or not name:
        raise CardError(f"{path} has no name")
    return host_segment(name)


def this_machine_host(path: Path | None = None, hostname=None) -> str:
    """This machine's key segment: the card's name, else the hostname.

    What ``--host local`` stands for on the fabric. A card that is there but
    unusable is an error, never a reason to fall back: it would serve under a
    name the fleet does not know this machine by. With no card, the hostname's
    first label is used, lower-cased (``Nishs-MacBook-Pro.local`` →
    ``nishs_macbook_pro``), since a key segment is lower case.
    """
    path = path or card_path()
    if path.exists() or path.is_symlink():
        return read_host(path)  # a directory or dangling link is an unusable card, not no card
    name = (hostname or socket.gethostname)().split(".", 1)[0].lower()
    if not name:
        raise CardError(f"{path} does not exist and this machine has no hostname")
    return host_segment(name)
