"""Give this package's tests a ROS domain that no other test process is using.

colcon tests packages in parallel, and on one shared domain a test in one package
receives another package's /robot_description, /clock and controller_manager.
domain_coordinator reserves a free domain id for as long as this pytest process
runs; every node and launched process here inherits it through ROS_DOMAIN_ID.
"""

from contextlib import ExitStack
import importlib.util
import os

_reservation = ExitStack()


def pytest_configure(config):
    if importlib.util.find_spec('rclpy') is None:
        return  # A run without ROS has no graph to share.
    from domain_coordinator import domain_id
    os.environ['ROS_DOMAIN_ID'] = str(_reservation.enter_context(domain_id()))


def pytest_unconfigure(config):
    _reservation.close()
