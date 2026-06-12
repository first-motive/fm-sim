"""Smoke test: package imports cleanly."""

import importlib


def test_import_package():
    importlib.import_module("fm_sim_backends")
