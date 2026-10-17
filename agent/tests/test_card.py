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


# -- this machine (what --host local serves as) -------------------------------


def test_this_machine_is_the_cards_name(tmp_path):
    path = card(tmp_path, name="nish-mbp")
    assert card_module.this_machine_host(path, hostname=lambda: "ignored") == "nish_mbp"


def test_with_no_card_this_machine_is_the_hostnames_first_label_lower_cased(tmp_path):
    missing = tmp_path / "machine.json"
    host = card_module.this_machine_host(missing, hostname=lambda: "Nishs-MacBook-Pro.local")
    assert host == "nishs_macbook_pro"


@pytest.mark.parametrize("hostname", ["", "fm*ws", "9lives", "a/b.local"])
def test_a_hostname_that_is_not_a_key_segment_is_refused(tmp_path, hostname):
    with pytest.raises(CardError):
        card_module.this_machine_host(tmp_path / "machine.json", hostname=lambda: hostname)


def test_an_unusable_card_is_refused_not_passed_over_for_the_hostname(tmp_path):
    path = card(tmp_path, name="fm/*")
    with pytest.raises(CardError):
        card_module.this_machine_host(path, hostname=lambda: "fine")
    path.write_text("not json")
    with pytest.raises(CardError):
        card_module.this_machine_host(path, hostname=lambda: "fine")


def test_a_dangling_symlink_or_directory_card_is_refused_not_passed_over(tmp_path):
    dangling = tmp_path / "machine.json"
    dangling.symlink_to(tmp_path / "nowhere.json")
    with pytest.raises(CardError):
        card_module.this_machine_host(dangling, hostname=lambda: "fine")
    directory = tmp_path / "dir.json"
    directory.mkdir()
    with pytest.raises(CardError):
        card_module.this_machine_host(directory, hostname=lambda: "fine")
