"""Read-only Panda joint trajectory tracking diagnostics."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from panda_nmpc.reference_trajectory import ReferenceTrajectory
from panda_nmpc.safety import ordered_joint_positions, position_error_norm


class PandaNMPCNode(Node):
    def __init__(self):
        super().__init__("panda_nmpc")
        self.joint_names = [f"panda_joint{i}" for i in range(1, 8)]
        self.q = None
        self.reference = ReferenceTrajectory(self.joint_names)
        self.reference_start = None
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("reference_topic", "/panda_nmpc/reference_trajectory")
        self.declare_parameter("diagnostics_rate_hz", 10.0)
        rate = self.get_parameter("diagnostics_rate_hz").value
        if rate <= 0:
            raise ValueError("diagnostics_rate_hz must be positive")
        state_topic = self.get_parameter("joint_state_topic").value
        reference_topic = self.get_parameter("reference_topic").value
        self.joint_subscription = self.create_subscription(
            JointState, state_topic, self.joint_state_callback, qos_profile_sensor_data
        )
        self.reference_subscription = self.create_subscription(
            JointTrajectory, reference_topic, self.reference_callback, 10
        )
        self.timer = self.create_timer(1.0 / rate, self.show_tracking)
        self.get_logger().info("Read-only tracking node started; no commands are sent")

    def joint_state_callback(self, msg):
        q = ordered_joint_positions(msg, self.joint_names)
        if q is not None:
            self.q = q

    def reference_callback(self, msg):
        try:
            duration = self.reference.set_trajectory(msg)
        except ValueError as exc:
            self.get_logger().warn(f"Rejected reference trajectory: {exc}")
            return
        self.reference_start = time.monotonic()
        self.get_logger().info(f"Accepted reference trajectory ({duration:.2f} s)")

    def show_tracking(self):
        if self.q is None or self.reference_start is None:
            return
        elapsed = time.monotonic() - self.reference_start
        desired = self.reference.sample(elapsed)
        error = position_error_norm(self.q, desired)
        self.get_logger().info(
            f"t={elapsed:.2f} s, joint position error={error:.4f} rad"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PandaNMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
