"""Smoke test: package modules import cleanly."""

import importlib


def test_import_modules():
    importlib.import_module("fm_sim_models.models")
