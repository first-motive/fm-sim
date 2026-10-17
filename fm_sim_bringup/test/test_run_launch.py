"""Live: every robot on every backend it supports, empty scenario, idle controller.

run.launch.py must bring the controller_manager up with the broadcaster active and
every other robot controller loaded inactive. On MuJoCo and Gazebo every joint sits
in one hardware component (one MuJoCo world; the one system gz_ros2_control
initialises); Isaac keeps the description's per-arm systems and needs no Isaac Sim
running for the controllers to come up. Needs the MJCF and description externals
(vcs import fm-sim.repos), topic_based_ros2_control and xvfb-run; the engines run
headless on software GL, Gazebo server-only.
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
# Per robot: the controllers idle loads inactive.
EXPECTED_INACTIVE = {
    'openarm': {
        'openarm_left_arm_controller',
        'openarm_right_arm_controller',
        'openarm_left_forward_position_controller',
        'openarm_right_forward_position_controller',
        'openarm_left_gripper_controller',
        'openarm_right_gripper_controller',
    },
    'so101': {'so101_arm_controller', 'so101_gripper_controller'},
    'axol': {'axol_left_arm_controller', 'axol_right_arm_controller'},
}
# Per robot: its hardware components on the simulators, and on Isaac.
EXPECTED_COMPONENTS = {
    'openarm': ({'openarm_system'}, {'openarm_left_system', 'openarm_right_system'}),
    'so101': ({'so101_system'}, {'so101_system'}),
    'axol': ({'axol_system'}, {'axol_left_system', 'axol_right_system'}),
}
ROWS = [
    (robot, backend)
    for robot in EXPECTED_INACTIVE
    for backend in ('mujoco', 'gazebo', 'isaac')
]
# A row's engine can outlive its launch (ros_gz_sim runs gz sim under a shell that
# shutdown kills, leaving the server and its controller_manager running). Each row
# gets its own ROS domain and Gazebo partition from here, so a survivor of an earlier
# row cannot answer this one.
_FIRST_DOMAIN_ID = 20


@pytest.mark.launch_test
@launch_testing.parametrize('robot,backend', ROWS)
def generate_test_description(robot, backend):
    # Set in this process too: the launched processes inherit it, and the probe
    # node in the test reads it at rclpy.init.
    os.environ['ROS_DOMAIN_ID'] = str(_FIRST_DOMAIN_ID + ROWS.index((robot, backend)))
    os.environ['GZ_PARTITION'] = f'fm_sim_test_{robot}_{backend}'
    run_launch = os.path.join(
        get_package_share_directory('fm_sim_bringup'), 'launch', 'run.launch.py'
    )
    # The returned dict is how launch_testing hands the parameters to the test.
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(run_launch),
            launch_arguments={
                'backend': backend,
                'robot': robot,
                'scenario': 'empty',
                'controller': 'idle',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {'robot': robot, 'backend': backend}


def _call(node, client, request):
    if not client.wait_for_service(timeout_sec=1.0):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    return future.result()


class TestIdle(unittest.TestCase):

    def test_controllers_reach_their_states(self, robot, backend):
        expected_inactive = EXPECTED_INACTIVE[robot]
        simulators, isaac = EXPECTED_COMPONENTS[robot]
        expected_components = isaac if backend == 'isaac' else simulators
        run = f'{robot} on {backend}'
        rclpy.init()
        node = rclpy.create_node('run_launch_probe')
        controllers = node.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        hardware = node.create_client(
            ListHardwareComponents, '/controller_manager/list_hardware_components'
        )
        states, components = {}, set()
        deadline = time.monotonic() + TIMEOUT_S
        try:
            while time.monotonic() < deadline:
                result = _call(node, controllers, ListControllers.Request())
                if result is not None:
                    states = {c.name: c.state for c in result.controller}
                    if all(states.get(n) == 'active' for n in EXPECTED_ACTIVE) and all(
                        states.get(n) == 'inactive' for n in expected_inactive
                    ):
                        break
                time.sleep(1.0)
            result = _call(node, hardware, ListHardwareComponents.Request())
            components = {c.name for c in result.component} if result else set()
        finally:
            node.destroy_node()
            rclpy.shutdown()
        for name in EXPECTED_ACTIVE:
            self.assertEqual(states.get(name), 'active', f'{run} {name}: {states}')
        for name in expected_inactive:
            self.assertEqual(states.get(name), 'inactive', f'{run} {name}: {states}')
        self.assertEqual(set(states), EXPECTED_ACTIVE | expected_inactive, run)
        self.assertEqual(components, expected_components, run)
