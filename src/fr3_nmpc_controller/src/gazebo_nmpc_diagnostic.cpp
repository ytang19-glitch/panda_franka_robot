// Read-only online NMPC diagnostic: no controller claims or effort writes.
#include "fr3_nmpc_controller/robot_model.hpp"
#include "fr3_nmpc_controller/optimizer.hpp"
#include "fr3_nmpc_controller/solver_worker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace fc = fr3_nmpc_controller;
using namespace std::chrono_literals;

class GazeboNmpcDiagnostic final : public rclcpp::Node
{
public:
  GazeboNmpcDiagnostic() : Node("gazebo_nmpc_diagnostic")
  {
    const auto urdf = declare_parameter<std::string>("model_urdf_path", "");
    if (urdf.empty()) {
      throw std::invalid_argument("Set model_urdf_path to a verified seven-joint FR3 URDF");
    }

    model_ = std::make_shared<fc::RobotModel>();
    model_->load(urdf);
    const auto limits = model_->limits();

    fc::OptimizerConfig cfg;
    cfg.horizon = 2;
    cfg.dt = 0.04;
    cfg.max_iterations = 300;
    // Diagnostic budgets, NOT real-time or hardware parameters.
    cfg.max_cpu_time_s = 2.0;
    cfg.max_wall_time_s = 2.0;
    for (std::size_t i = 0; i < fc::kJoints; ++i) {
      cfg.torque_limit[i] = limits.effort[i];
      cfg.torque_rate_limit[i] = 200.0;  // Illustrative, simulation-only.
      cfg.acceleration_limit[i] = 2.0;   // Illustrative, simulation-only.
    }
    cfg_ = cfg;
    optimizer_ = std::make_shared<fc::Optimizer>();
    optimizer_->configure(model_, cfg_);
    worker_.start(optimizer_, cfg_.horizon, 3000000000LL);

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { on_state(*msg); });
    timer_ = create_wall_timer(250ms, [this]() { on_timer(); });
    RCLCPP_INFO(get_logger(),
      "READ-ONLY NMPC diagnostic started (no actuator commands). "
      "Hold reference is captured from first valid Gazebo state.");
  }

  ~GazeboNmpcDiagnostic() override { worker_.stop(); }

private:
  void on_state(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() != msg.name.size() ||
        msg.velocity.size() != msg.name.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "JointState names/positions/velocities have different lengths");
      return;
    }
    fc::RobotState incoming{};
    for (std::size_t j = 0; j < fc::kJoints; ++j) {
      const std::string name = "fr3_joint" + std::to_string(j + 1);
      auto it = std::find(msg.name.begin(), msg.name.end(), name);
      if (it == msg.name.end()) { return; }
      const auto idx = static_cast<std::size_t>(std::distance(msg.name.begin(), it));
      const double q = msg.position[idx], dq = msg.velocity[idx];
      if (!std::isfinite(q) || !std::isfinite(dq)) { return; }
      incoming.q[j] = q;
      incoming.dq[j] = dq;
    }
    latest_state_ = incoming;
    latest_state_ns_ = fc::SolverWorker::now_ns();
    if (!reference_ready_) {
      reference_ = incoming;
      reference_.dq.fill(0.0);
      previous_torque_ = model_->gravity(incoming.q);
      reference_ready_ = true;
      RCLCPP_INFO(get_logger(), "Captured initial hold reference from Gazebo state");
    }
  }

  void on_timer()
  {
    if (!reference_ready_) { return; }
    const auto now = fc::SolverWorker::now_ns();
    if (now - latest_state_ns_ > 1000000000LL) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Joint feedback stale for >1s; skipping solves");
      return;
    }

    // Read-only: optimizer uses latest measured q/dq, but its output is never
    // applied. Torque history is an illustrative gravity seed, not actuator history.
    fc::SolverRequest request;
    request.state = latest_state_;
    request.reference_horizon.assign(cfg_.horizon + 1, reference_);
    request.previous_total_torque = previous_torque_;
    request.state_time_ns = latest_state_ns_;
    request.reference_id = 1;  // Constant hold reference throughout this run.
    if (!worker_.submit(std::move(request))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Could not submit solver request: %s", worker_.last_error().c_str());
    }

    const auto result = worker_.latest();
    if (result && result->request_id != last_reported_request_) {
      last_reported_request_ = result->request_id;
      double max_error = 0.0;
      for (std::size_t j = 0; j < fc::kJoints; ++j) {
        max_error = std::max(max_error,
          std::abs(latest_state_.q[j] - reference_.q[j]));
      }
      RCLCPP_INFO(get_logger(),
        "NMPC read-only: solve=%.2f ms violation=%.3e "
        "max_hold_error=%.6f rad candidate_tau1=%.3f Nm (NOT applied)",
        result->solve_time_ms, result->max_constraint_violation,
        max_error, result->total_torque[0]);
      // Do not replace previous_torque_: no candidate torque was executed.
    }
  }

  std::shared_ptr<fc::RobotModel> model_;
  std::shared_ptr<fc::Optimizer> optimizer_;
  fc::SolverWorker worker_;
  fc::OptimizerConfig cfg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  fc::RobotState latest_state_{}, reference_{};
  fc::JointVector previous_torque_{};
  std::int64_t latest_state_ns_{0};
  std::uint64_t last_reported_request_{0};
  bool reference_ready_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GazeboNmpcDiagnostic>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("gazebo_nmpc_diagnostic"),
      "Diagnostic stopped: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
