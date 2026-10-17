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


# Shared test constants. The OpenArm cameras file fm_sim_bringup ships, read from
# the source tree: fm_sim_bringup depends on this package, so it cannot be a
# dependency here.
OPENARM_CAMERAS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', 'fm_sim_bringup', 'config', 'openarm', 'cameras.yaml',
)
# fm-skills src/openarm_v2_skills_config/config/camera_chest.yaml at aa91a3f.
CHEST_K = [740.244, 0.0, 958.504, 0.0, 750.103, 573.152, 0.0, 0.0, 1.0]
CHEST_D = [0.111681, -0.028494, 0.0, 0.0]
