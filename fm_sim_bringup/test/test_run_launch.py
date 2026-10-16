"""Live: OpenArm on MuJoCo and on Gazebo, empty scenario, idle controller.

run.launch.py must bring the controller_manager up with the broadcaster active,
every other OpenArm controller loaded inactive, and both arms in one hardware
component (one MuJoCo world; the one system gz_ros2_control initialises). Needs
the openarm_mujoco external (vcs import fm-sim.repos) and xvfb-run; both engines
run headless on software GL, Gazebo server-only.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy

TIMEOUT_S = 180.0
EXPECTED_ACTIVE = {'joint_state_broadcaster'}
EXPECTED_INACTIVE = {
    'openarm_left_arm_controller',
    'openarm_right_arm_controller',
    'openarm_left_forward_position_controller',
    'openarm_right_forward_position_controller',
    'openarm_left_gripper_controller',
    'openarm_right_gripper_controller',
}


@pytest.mark.launch_test
@launch_testing.parametrize('backend', ['mujoco', 'gazebo'])
def generate_test_description(backend):
    run_launch = os.path.join(
        get_package_share_directory('fm_sim_bringup'), 'launch', 'run.launch.py'
    )
    # The returned dict is how launch_testing hands `backend` to the test method.
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(run_launch),
            launch_arguments={
                'backend': backend,
                'robot': 'openarm',
                'scenario': 'empty',
                'controller': 'idle',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {'backend': backend}


def _call(node, client, request):
    if not client.wait_for_service(timeout_sec=1.0):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    return future.result()


class TestOpenArmIdle(unittest.TestCase):

    def test_controllers_reach_their_states(self, backend):
        rclpy.init()
        node = rclpy.create_node('run_launch_probe')
        controllers = node.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        hardware = node.create_client(
            ListHardwareComponents, '/controller_manager/list_hardware_components'
        )
        states, components = {}, []
        deadline = time.monotonic() + TIMEOUT_S
        try:
            while time.monotonic() < deadline:
                result = _call(node, controllers, ListControllers.Request())
                if result is not None:
                    states = {c.name: c.state for c in result.controller}
                    if all(states.get(n) == 'active' for n in EXPECTED_ACTIVE) and all(
                        states.get(n) == 'inactive' for n in EXPECTED_INACTIVE
                    ):
                        break
                time.sleep(1.0)
            result = _call(node, hardware, ListHardwareComponents.Request())
            components = [c.name for c in result.component] if result else []
        finally:
            node.destroy_node()
            rclpy.shutdown()
        for name in EXPECTED_ACTIVE:
            self.assertEqual(states.get(name), 'active', f'{backend} {name}: {states}')
        for name in EXPECTED_INACTIVE:
            self.assertEqual(states.get(name), 'inactive', f'{backend} {name}: {states}')
        self.assertEqual(components, ['openarm_system'], backend)
