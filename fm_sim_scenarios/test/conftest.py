"""Shared fixtures: the shipped can_on_table scenario and the probe arm's MJCF.

The tests also get a ROS domain that no other test process is using. colcon tests
packages in parallel, and on one shared domain these tests receive the live
scenario runs' /sim/objects and pose topics from fm_sim_bringup.
domain_coordinator reserves a free domain id for as long as this pytest process
runs; every node here inherits it through ROS_DOMAIN_ID.

The nodes run on CycloneDDS, the RMW the sim agent defaults to. On
rmw_fastrtps_cpp, a subscription's default QoS event handlers wake the executor's
wait set while that subscription has a message not yet taken. A
MultiThreadedExecutor excludes such a subscription from the wait set until a
worker takes the message, but the event handlers stay in it. The executor then
rebuilds its wait set thousands of times a second and holds the GIL. The 50 Hz
fakes publish only a few times a second, and the reset deadlines run out.
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
    # tradeoff: overrides any RMW the shell sets, so rmw_fastrtps_cpp goes untested
    # here; test_the_nodes_run_on_cyclonedds fails if the override stops applying.
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
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

# Threads for the test executors, which spin a fake engine, the scenario node and a
# probe together. rclpy sizes a default MultiThreadedExecutor by the CPUs the process
# may use, and on a two-CPU runner a blocked reset and a slow fake reply hold every
# thread: nothing is left to publish poses or deliver replies.
EXECUTOR_THREADS = 8

with open(CAN_ON_TABLE_YAML) as entry_file:
    CAN_ON_TABLE = yaml.safe_load(entry_file)
CAN_ON_TABLE['objects'].append(BALL)


@pytest.fixture
def can_on_table():
    return copy.deepcopy(CAN_ON_TABLE)
