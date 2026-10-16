"""Live check: the mujoco and isaac backends publish /robot_description on Jazzy.

The Jazzy controller_manager initialises only from that topic, so a backend that
stops publishing it hangs on "Waiting for data on 'robot_description' topic".
Each backend launches with a one-joint probe robot from ``fixtures/``.
"""

import os
import time
import unittest

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
    backend_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(HERE, '..', 'launch', f'{backend}.launch.py')
        ),
        launch_arguments={
            'robot_description': _probe_description(backend),
            'controllers_file': os.path.join(FIXTURES, 'controllers.yaml'),
        }.items(),
    )
    # The returned dict is how launch_testing hands `backend` to the test method.
    return (
        LaunchDescription([backend_launch, launch_testing.actions.ReadyToTest()]),
        {'backend': backend},
    )


class TestRobotDescriptionTopic(unittest.TestCase):

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
