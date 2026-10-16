##""Online kinematic MPC diagnostics for Panda/FR3; never commands hardware."""

import math
import time

import casadi as ca
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory

from panda_nmpc.reference_trajectory import ReferenceTrajectory
from panda_nmpc.safety import ordered_joint_positions, position_error_norm


class JointSpaceMPC:
    """Finite-horizon optimizer using a simple joint-position model."""

    def __init__(
        self,
        num_joints=7,
        horizon=10,
        dt=0.1,
        max_joint_velocity=0.4,
    ):
        self.num_joints = num_joints
        self.horizon = horizon
        self.dt = dt
        if num_joints <= 0 or horizon <= 0 or not math.isfinite(dt) or dt <= 0:
            raise ValueError("Invalid MPC dimensions or dt")
        if not math.isfinite(max_joint_velocity) or max_joint_velocity <= 0:
            raise ValueError("max_joint_velocity must be positive")
        self.previous_solution = None

        opt = ca.Opti()

        # Current measured joint positions, updated before each optimization.
        self.q_measured = opt.parameter(num_joints)

        # Reference joint positions over the prediction horizon.
        self.q_reference = opt.parameter(num_joints, horizon)

        # Decision variables: joint velocities for each horizon step.
        self.u = opt.variable(num_joints, horizon)

        # Starting prediction is the latest measured joint position.
        q_predicted = self.q_measured
        objective = 0

        # Cost weights. Increase Q to prioritize tracking;
        # increase R to penalize large joint velocities more strongly.
        Q = 10.0
        R = 0.05

        for k in range(horizon):
            # Prediction model:
            # q[k+1] = q[k] + dt * u[k]
            q_predicted = q_predicted + dt * self.u[:, k]

            # Predicted tracking error at this step:
            # e[k+1] = q_predicted - q_reference[k]
            error = q_predicted - self.q_reference[:, k]

            # Minimize tracking error and excessive control effort.
            objective += Q * ca.sumsqr(error)
            objective += R * ca.sumsqr(self.u[:, k])

        # Bound each proposed joint velocity.
        opt.subject_to(
            opt.bounded(
                -max_joint_velocity,
                self.u,
                max_joint_velocity,
            )
        )

        opt.minimize(objective)
        opt.solver(
            "ipopt",
            {"print_time": False},
            {
                "print_level": 0,
                "max_iter": 50,
            },
        )

        self.opt = opt

    def solve(self, measured_q, future_references):
        """Return the first velocity vector from the optimal sequence."""
        self.opt.set_value(self.q_measured, measured_q)

        # Input is horizon × joints; CasADi parameter is joints × horizon.
        reference_matrix = ca.DM(future_references).T
        self.opt.set_value(self.q_reference, reference_matrix)

        if self.previous_solution is not None:
            self.opt.set_initial(self.u, self.previous_solution)
        solution = self.opt.solve()
        sequence = ca.DM(solution.value(self.u))
        if not bool(sequence.is_regular()):
            raise RuntimeError("Solver returned non-finite velocities")
        self.previous_solution = ca.horzcat(sequence[:, 1:], sequence[:, -1])

        # MPC applies only the first input, then solves again using new feedback.
        first_velocity = solution.value(self.u[:, 0])
        return ca.DM(first_velocity).full().flatten().tolist()


class PandaNMPCNode(Node):
    def __init__(self):
        super().__init__("panda_nmpc")

        self.declare_parameter("joint_names", [f"panda_joint{i}" for i in range(1, 8)])
        self.joint_names = list(self.get_parameter("joint_names").value)
        if len(self.joint_names) != 7 or len(set(self.joint_names)) != 7:
            raise ValueError("Expected seven unique joint names")
        self.declare_parameter("horizon", 10)
        self.declare_parameter("prediction_dt", 0.1)
        self.declare_parameter("max_joint_velocity", 0.4)
        self.declare_parameter("max_state_age_s", 0.1)
        self.max_state_age = float(self.get_parameter("max_state_age_s").value)
        if not math.isfinite(self.max_state_age) or self.max_state_age <= 0:
            raise ValueError("max_state_age_s must be positive")

        self.q_measured = None
        self.last_joint_state_time = None

        self.reference = ReferenceTrajectory(self.joint_names)
        self.reference_start = None
        self.reference_duration = None

        self.controller = JointSpaceMPC(
            num_joints=7,
            horizon=int(self.get_parameter("horizon").value),
            dt=float(self.get_parameter("prediction_dt").value),
            max_joint_velocity=float(self.get_parameter("max_joint_velocity").value),
        )

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "reference_topic",
            "/panda_nmpc/reference_trajectory",
        )
        self.declare_parameter("diagnostics_rate_hz", 10.0)

        rate = float(self.get_parameter("diagnostics_rate_hz").value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("diagnostics_rate_hz must be positive")

        self.period_s = 1.0 / rate
        self.reference_publisher = self.create_publisher(JointState, "~/reference_sample", 10)
        self.error_publisher = self.create_publisher(JointState, "~/tracking_error", 10)
        self.proposal_publisher = self.create_publisher(JointState, "~/proposed_velocity", 10)
        self.solve_time_publisher = self.create_publisher(Float64, "~/solve_time_ms", 10)

        joint_state_topic = self.get_parameter("joint_state_topic").value
        reference_topic = self.get_parameter("reference_topic").value

        self.joint_subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        self.reference_subscription = self.create_subscription(
            JointTrajectory,
            reference_topic,
            self.reference_callback,
            10,
        )

        self.timer = self.create_timer(
            1.0 / rate,
            self.optimize_tracking,
        )

        self.get_logger().info(
            "Joint-space optimizer started in read-only mode; "
            "it does not send robot commands"
        )

    def joint_state_callback(self, msg):
        """Store measured positions in the configured robot joint order."""
        positions = ordered_joint_positions(msg, self.joint_names)

        if positions is None:
            return

        positions = [float(value) for value in positions]

        if not all(math.isfinite(value) for value in positions):
            self.get_logger().warning(
                "Ignoring joint state containing non-finite values"
            )
            return

        self.q_measured = positions
        self.last_joint_state_time = time.monotonic()

    def reference_callback(self, msg):
        """Validate and store a new joint trajectory reference."""
        try:
            duration = self.reference.set_trajectory(msg)
        except ValueError as exc:
            self.get_logger().warning(
                f"Rejected reference trajectory: {exc}"
            )
            return

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns:
            self.reference_start = Time(
                nanoseconds=stamp_ns, clock_type=self.get_clock().clock_type)
        else:
            self.reference_start = self.get_clock().now()
            self.get_logger().warning(
                "Reference has zero start stamp: using receipt time. "
                "Tracking metrics are not execution-synchronized.")
        self.controller.previous_solution = None
        self.reference_duration = float(duration)

        self.get_logger().info(
            f"Accepted reference trajectory "
            f"({self.reference_duration:.2f} s)"
        )

    def optimize_tracking(self):
        """Compute current error and solve the finite-horizon problem."""
        if self.q_measured is None:
            return

        if self.reference_start is None:
            return

        # Do not optimize using old joint feedback.
        state_age = time.monotonic() - self.last_joint_state_time
        if state_age > self.max_state_age:
            self.get_logger().warning(
                f"Joint state is stale ({state_age:.2f} s); skipping solve"
            )
            return

        # Use ROS time so this follows /clock when use_sim_time is enabled.
        elapsed = (
            self.get_clock().now() - self.reference_start
        ).nanoseconds * 1e-9

        if elapsed < 0.0:
            # A future header stamp schedules a reference; wait until it starts.
            return

        if elapsed >= self.reference_duration:
            self.get_logger().info("Reference trajectory finished")
            self.reference_start = None
            return

        measured_q = list(self.q_measured)
        sample_stamp = self.get_clock().now().to_msg()
        try:
            # Sample the desired joint positions across the prediction horizon.
            future_references = []

            for k in range(self.controller.horizon):
                sample_time = min(
                    elapsed + (k + 1) * self.controller.dt,
                    self.reference_duration,
                )
                reference_q = self.reference.sample(sample_time)
                future_references.append(
                    [float(value) for value in reference_q]
                )

            # Current desired position and measured tracking error.
            desired_now = self.reference.sample(elapsed)
            current_error = [
                float(desired_now[i]) - measured_q[i]
                for i in range(7)
            ]
            error_norm = position_error_norm(
                measured_q,
                desired_now,
            )

            # Solve for the best future joint-velocity sequence.
            solve_start = time.monotonic()
            optimized_velocity = self.controller.solve(
                measured_q,
                future_references,
            )
            solve_time_ms = (
                time.monotonic() - solve_start
            ) * 1000.0

        except (ValueError, RuntimeError) as exc:
            self.get_logger().warning(
                f"Optimization failed: {exc}"
            )
            return

        timing_msg = Float64()
        timing_msg.data = solve_time_ms
        self.solve_time_publisher.publish(timing_msg)
        if solve_time_ms > self.period_s * 1000.0:
            self.get_logger().warning("Solver exceeded requested timer period")

        reference_msg = JointState()
        reference_msg.header.stamp = sample_stamp
        reference_msg.name = self.joint_names
        reference_msg.position = [float(v) for v in desired_now]
        self.reference_publisher.publish(reference_msg)

        error_msg = JointState()
        error_msg.header.stamp = sample_stamp
        error_msg.name = self.joint_names
        error_msg.position = current_error
        self.error_publisher.publish(error_msg)

        proposal_msg = JointState()
        proposal_msg.header.stamp = sample_stamp
        proposal_msg.name = self.joint_names
        proposal_msg.velocity = [float(v) for v in optimized_velocity]
        # Diagnostic topic only: do NOT remap directly to a robot command topic.
        self.proposal_publisher.publish(proposal_msg)

        error_for_log = [
            round(value, 4) for value in current_error
        ]
        velocity_for_log = [
            round(value, 3) for value in optimized_velocity
        ]

        self.get_logger().info(
            f"t={elapsed:.2f} s | "
            f"error vector [rad]={error_for_log} | "
            f"error norm={error_norm:.4f} rad | "
            f"optimized first velocity [rad/s]={velocity_for_log} | "
            f"solve={solve_time_ms:.1f} ms"
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
