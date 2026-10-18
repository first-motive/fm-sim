"""Shared fixtures: the shipped can_on_table scenario and the probe arm's MJCF.

The tests also get a ROS domain that no other test process is using. colcon tests
packages in parallel, and on one shared domain these tests receive the live
scenario runs' /sim/objects and pose topics from fm_sim_bringup.
domain_coordinator reserves a free domain id for as long as this pytest process
runs; every node here inherits it through ROS_DOMAIN_ID.
"""

from contextlib import ExitStack
import copy
import importlib.util
import os

import pytest
import yaml

_reservation = ExitStack()


def pytest_configure(config):
    if importlib.util.find_spec('rclpy') is None:
        return  # A run without ROS has no graph to share.
    from domain_coordinator import domain_id
    os.environ['ROS_DOMAIN_ID'] = str(_reservation.enter_context(domain_id()))


def pytest_unconfigure(config):
    _reservation.close()


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
