"""Forward new MoveIt display plans to the read-only Panda reference topic.

This bridge publishes references only. It does not execute a trajectory or
synchronize the reference with controller execution.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory

from panda_nmpc.reference_trajectory import ReferenceTrajectory


class MoveItReferenceBridge(Node):
    def __init__(self):
        super().__init__("moveit_reference_bridge")
        self.declare_parameter("display_topic", "/display_planned_path")
        self.declare_parameter(
            "reference_topic", "/panda_nmpc/reference_trajectory"
        )
        display_topic = self.get_parameter("display_topic").value
        reference_topic = self.get_parameter("reference_topic").value
        self.validator = ReferenceTrajectory(
            [f"panda_joint{i}" for i in range(1, 8)]
        )
        self.publisher = self.create_publisher(
            JointTrajectory, reference_topic, 10
        )
        # Reliable, volatile QoS: consume new plans rather than a latched
        # plan created before this bridge started.
        self.subscription = self.create_subscription(
            DisplayTrajectory, display_topic, self.forward_reference, 10
        )
        self.get_logger().info(
            f"Bridge ready: {display_topic} -> {reference_topic}. "
            "Create a NEW plan in RViz; no robot commands are sent."
        )

    def forward_reference(self, msg):
        if len(msg.trajectory) != 1:
            self.get_logger().warning(
                "Expected one robot trajectory; empty or multi-segment "
                "display messages are not forwarded."
            )
            return

        trajectory = msg.trajectory[0].joint_trajectory
        try:
            duration = self.validator.set_trajectory(trajectory)
            if duration <= 0.0:
                raise ValueError("Trajectory duration must be positive")
        except ValueError as exc:
            self.get_logger().warning(f"Rejected reference: {exc}")
            return

        self.publisher.publish(trajectory)
        self.get_logger().info(
            f"Forwarded {len(trajectory.points)} trajectory points "
            f"({duration:.2f} s)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MoveItReferenceBridge()
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
