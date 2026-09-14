import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class PandaNMPCNode(Node):
    def __init__(self):
        super().__init__("panda_nmpc")

        # Required order for the seven arm joints.
        self.joint_names = [
            f"panda_joint{i}" for i in range(1, 8)
        ]

        self.q = None

        # Receive the robot's measured joint positions.
        self.joint_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        # Print the latest measurements once per second.
        self.timer = self.create_timer(1.0, self.show_state)

        self.get_logger().info(
            "Panda node started. Waiting for /joint_states..."
        )

    def joint_state_callback(self, msg):
        # Match positions by joint name, not message order.
        positions = dict(zip(msg.name, msg.position))

        if not all(name in positions for name in self.joint_names):
            return

        self.q = [
            positions[name] for name in self.joint_names
        ]

    def show_state(self):
        if self.q is None:
            self.get_logger().info(
                "Still waiting for all seven Panda joints..."
            )
            return

        values = ", ".join(f"{angle:.3f}" for angle in self.q)
        self.get_logger().info(f"Joint angles [rad]: [{values}]")


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