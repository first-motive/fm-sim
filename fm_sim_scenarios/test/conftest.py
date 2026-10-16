"""Shared fixtures: the shipped can_on_table scenario and the probe arm's MJCF."""

import copy
import os

import pytest
import yaml

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
ARM_MJCF = os.path.join(TEST_DIR, 'fixtures', 'arm', 'arm.xml')
# The catalog entry fm_sim_bringup ships, read from the source tree: fm_sim_bringup
# depends on this package, so it cannot be a dependency here.
CAN_ON_TABLE_YAML = os.path.join(
    TEST_DIR, '..', '..', 'fm_sim_bringup', 'catalog', 'scenarios', 'can_on_table.yaml'
)
# A test-only extra: a sphere, and a free object with no spawn ranges.
BALL = {
    'name': 'ball', 'shape': 'sphere', 'size': [0.02], 'mass': 0.05, 'friction': 0.5,
    'color': [1.0, 0.0, 0.0, 1.0], 'pose': {'xyz': [0.6, 0.2, 0.228]},
}

with open(CAN_ON_TABLE_YAML) as entry_file:
    CAN_ON_TABLE = yaml.safe_load(entry_file)
CAN_ON_TABLE['objects'].append(BALL)


@pytest.fixture
def can_on_table():
    return copy.deepcopy(CAN_ON_TABLE)
