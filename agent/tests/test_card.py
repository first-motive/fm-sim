from __future__ import annotations

import json
from pathlib import Path

import pytest

from fm_sim_agent import card as card_module
from fm_sim_agent.card import CardError, card_path, host_segment, read_host


def card(tmp_path, **fields):
    path = tmp_path / "machine.json"
    path.write_text(json.dumps({"schema_version": 1, "role": "workstation", **fields}))
    return path


def test_the_host_segment_derives_from_the_name(tmp_path):
    assert read_host(card(tmp_path, name="fm-ws-01")) == "fm_ws_01"


def test_the_override_names_the_card(tmp_path, monkeypatch):
    monkeypatch.setenv("FM_MACHINE_FILE", str(card(tmp_path, name="fm-aws-01")))
    assert read_host() == "fm_aws_01"


@pytest.mark.parametrize("name", ["fm/*", "FM-WS-01", "", "1host"])
def test_a_name_that_is_not_a_key_segment_is_refused(tmp_path, name):
    with pytest.raises(CardError):
        read_host(card(tmp_path, name=name))


def test_an_absent_card_says_how_to_fix_it(tmp_path):
    with pytest.raises(CardError, match="--host"):
        read_host(tmp_path / "missing.json")


def test_another_schema_version_is_refused(tmp_path):
    with pytest.raises(CardError, match="schema_version"):
        read_host(card(tmp_path, name="fm-ws-01", schema_version=2))


def test_local_is_a_host():
    assert host_segment("local") == "local"


def test_macos_reads_the_card_under_xdg_config_home(tmp_path, monkeypatch):
    monkeypatch.delenv("FM_MACHINE_FILE", raising=False)
    monkeypatch.setattr(card_module.platform, "system", lambda: "Darwin")
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    assert card_path() == tmp_path / "fm" / "machine.json"
    monkeypatch.delenv("XDG_CONFIG_HOME")
    assert card_path() == Path.home() / ".config" / "fm" / "machine.json"


def test_linux_reads_the_card_under_etc(monkeypatch):
    monkeypatch.delenv("FM_MACHINE_FILE", raising=False)
    monkeypatch.setattr(card_module.platform, "system", lambda: "Linux")
    assert card_path() == Path("/etc/fm/machine.json")
