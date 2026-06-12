"""Smoke test: package modules import cleanly."""

import importlib


def test_import_modules():
    importlib.import_module("fm_sim_core.stepper")
    importlib.import_module("fm_sim_core.sim_loop")
