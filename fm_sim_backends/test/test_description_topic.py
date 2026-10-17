"""Live check: the mujoco and isaac backends publish /robot_description on Jazzy.

The Jazzy controller_manager initialises only from that topic, so a backend that
stops publishing it hangs on "Waiting for data on 'robot_description' topic".
Each backend launches with a one-joint probe robot from ``fixtures/``. After
shutdown no controller_manager may be left: a survivor answers the next launch's
spawners in place of that launch's own. Each row hands its controller_manager a
copy of the controllers file at a path of its own, so the check finds that row's
process and nothing else on the host.

The launch is stopped only once the controller_manager has served its last
service. A Jazzy controller_manager signalled while it still initialises from the
description can hang past SIGTERM or abort in ``init_services``; that is its own
race, not the backend's shutdown wiring this file checks.
"""

import os
import shutil
import tempfile
import time
import unittest

from controller_manager_msgs.srv import SetHardwareComponentState
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES = os.path.join(HERE, 'fixtures')
TIMEOUT_S = 30.0


def _probe_description(backend):
    with open(os.path.join(FIXTURES, f'probe_{backend}.urdf')) as urdf:
        return urdf.read().replace('MJCF_PATH', os.path.join(FIXTURES, 'probe.xml'))


@pytest.mark.launch_test
@launch_testing.parametrize('backend', ['mujoco', 'isaac'])
def generate_test_description(backend):
    controllers_file = os.path.join(
        tempfile.mkdtemp(prefix=f'fm_sim_backends_{backend}_'), 'controllers.yaml'
    )
    shutil.copy(os.path.join(FIXTURES, 'controllers.yaml'), controllers_file)
    backend_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(HERE, '..', 'launch', f'{backend}.launch.py')
        ),
        launch_arguments={
            'robot_description': _probe_description(backend),
            'controllers_file': controllers_file,
        }.items(),
    )
    # The returned dict is how launch_testing hands its values to the test methods.
    return (
        LaunchDescription([backend_launch, launch_testing.actions.ReadyToTest()]),
        {'backend': backend, 'controllers_file': controllers_file},
    )


class TestRobotDescriptionTopic(unittest.TestCase):

    def test_controller_manager_finishes_initialising(self, backend):
        # set_hardware_component_state is the last service init_services creates.
        rclpy.init()
        node = rclpy.create_node('controller_manager_probe')
        client = node.create_client(
            SetHardwareComponentState, '/controller_manager/set_hardware_component_state'
        )
        try:
            ready = client.wait_for_service(timeout_sec=TIMEOUT_S)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        self.assertTrue(ready, f'{backend}: controller_manager never finished initialising')

    def test_robot_description_published(self, backend):
        rclpy.init()
        node = rclpy.create_node('robot_description_probe')
        received = []
        # robot_state_publisher latches the description (transient local).
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(
            String, '/robot_description', lambda msg: received.append(msg.data), qos
        )
        deadline = time.monotonic() + TIMEOUT_S
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.5)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        self.assertTrue(received, f'{backend}: nothing on /robot_description')
        self.assertIn(f'probe_{backend}', received[0])


def _processes_naming(path):
    """Return ``pid: command line`` for each live process whose command line has ``path``."""
    found = {}
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue  # /proc/self and the like: the scan itself, not a process to find.
        try:
            with open(os.path.join('/proc', entry, 'cmdline'), 'rb') as cmdline:
                args = cmdline.read()
        except OSError:
            continue  # Not a process, or it exited while being read.
        if path.encode() in args:
            found[int(entry)] = args.replace(b'\0', b' ').decode(errors='replace')[:200]
    return found


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    @unittest.skipUnless(os.path.isdir('/proc'), 'finds processes through /proc')
    def test_controller_manager_exits_with_the_launch(self, backend, controllers_file):
        # The controller_manager is the one process given this row's controllers file.
        # launch has waited for every process it started, and each of those waits for
        # the controller_manager, so it must be gone already: no grace period.
        found = _processes_naming(controllers_file)
        self.assertEqual(found, {}, f'{backend}: controller_manager outlived the launch')
        shutil.rmtree(os.path.dirname(controllers_file))
