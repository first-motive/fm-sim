"""Puts infra/aws/sim on the import path, so tests import the shipped modules."""

import sys
from pathlib import Path

SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))
