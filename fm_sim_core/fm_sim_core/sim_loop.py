"""Headless MuJoCo sim loop for control / orchestration dev.

Runs native arm64 on M5 (CPU, no GPU). Steps a MuJoCo model via
``MujocoStepper`` and publishes sensor_msgs/JointState so the rest of the graph
(and Foxglove) sees sim joints. This node handles ROS comms only — the physics
lives in ``fm_sim_core.stepper``. Falls back to a built-in 1-DOF model when no
model_path is given.

Params (see config/sim.yaml):
    model_path (string, ""):    MJCF path; empty loads the built-in fallback.
    rate_hz    (double, 100.0): step + publish rate.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from fm_sim_core.stepper import MujocoStepper


class SimLoop(Node):
    """Step the sim and publish joint states at a fixed rate."""

    def __init__(self):
        super().__init__("sim_loop")
        self.declare_parameter("model_path", "")
        self.declare_parameter("rate_hz", 100.0)

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.stepper = MujocoStepper(model_path)

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        rate = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate, self._step)
        self.get_logger().info(
            f"sim_loop up: {self.stepper.njoints} joints @ {rate} Hz"
        )

    def _step(self):
        sample = self.stepper.step()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = sample.names
        msg.position = sample.positions
        msg.velocity = sample.velocities
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
