#pragma once
#include "fr3_nmpc_controller/robot_model.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace fr3_nmpc_controller
{
struct SolverResult
{
  JointVector total_torque{};
  // Hardware validity is NOT assigned by this offline optimizer.
  std::int64_t state_time_ns{0}, valid_until_ns{0};
  std::uint64_t reference_id{0};
  bool converged{false};
  double solve_time_ms{0.0};
  std::vector<RobotState> prediction;
  std::vector<JointVector> torque_sequence;
};
struct OptimizerConfig
{
  std::size_t horizon{10};
  double dt{0.05};
  double q_weight{100.0}, dq_weight{5.0}, effort_weight{0.001}, delta_weight{0.01};
  double terminal_weight{200.0}, feasibility_tolerance{1e-5};
  int max_iterations{100};
  double max_cpu_time_s{1.0}; // IPOPT CPU budget; not a hard wall-time deadline.
  // Required explicit limits; zero default arrays deliberately fail configure.
  // TOTAL modeled torques, not Franka effort commands. Rates are N*m/s.
  JointVector torque_limit{}, torque_rate_limit{}, acceleration_limit{};
};
class Optimizer
{
public:
  Optimizer();
  ~Optimizer();
  Optimizer(const Optimizer &) = delete;
  Optimizer & operator=(const Optimizer &) = delete;
  void configure(std::shared_ptr<const RobotModel> model, const OptimizerConfig & config);
  bool ready() const noexcept;
  void reset();
  // N+1 references sampled at t, t+dt, ... t+N*dt. Single-worker use only.
  std::optional<SolverResult> solve(const RobotState & state,
    const std::vector<RobotState> & reference_horizon, const JointVector & previous_total_torque);
  // Compatibility for old unloaded-backend tests; holds a constant reference.
  std::optional<SolverResult> solve(const RobotState & state, const RobotState & reference);
  const std::string & last_error() const noexcept;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::string last_error_;
};
}  // namespace fr3_nmpc_controller
