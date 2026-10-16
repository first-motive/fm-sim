"""Live: OpenArm on MuJoCo, empty scenario, idle controller.

run.launch.py must bring the controller_manager up with the broadcaster active and
every other OpenArm controller loaded inactive. Needs the openarm_mujoco external
(vcs import fm-sim.repos) and xvfb-run; MuJoCo runs headless on software GL.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
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
def generate_test_description():
    run_launch = os.path.join(
        get_package_share_directory('fm_sim_bringup'), 'launch', 'run.launch.py'
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(run_launch),
            launch_arguments={
                'backend': 'mujoco',
                'robot': 'openarm',
                'scenario': 'empty',
                'controller': 'idle',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestOpenArmIdleOnMujoco(unittest.TestCase):

    def test_controllers_reach_their_states(self):
        rclpy.init()
        node = rclpy.create_node('run_launch_probe')
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        states = {}
        deadline = time.monotonic() + TIMEOUT_S
        try:
            while time.monotonic() < deadline:
                if client.wait_for_service(timeout_sec=1.0):
                    future = client.call_async(ListControllers.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                    if future.result() is not None:
                        states = {c.name: c.state for c in future.result().controller}
                        if all(states.get(n) == 'active' for n in EXPECTED_ACTIVE) and all(
                            states.get(n) == 'inactive' for n in EXPECTED_INACTIVE
                        ):
                            break
                time.sleep(1.0)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        for name in EXPECTED_ACTIVE:
            self.assertEqual(states.get(name), 'active', f'{name}: {states}')
        for name in EXPECTED_INACTIVE:
            self.assertEqual(states.get(name), 'inactive', f'{name}: {states}')
