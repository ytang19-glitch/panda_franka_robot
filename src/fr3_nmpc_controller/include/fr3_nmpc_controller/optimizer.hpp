#pragma once
#include "fr3_nmpc_controller/robot_model.hpp"
#include <cstdint>
#include <optional>

namespace fr3_nmpc_controller
{
// Times are monotonic nanoseconds, NOT ROS/simulation time. Validity must be
// assigned by the future worker from measured latency and validated policy.
struct SolverResult
{
  JointVector total_torque{};
  std::int64_t state_time_ns{0};
  std::int64_t valid_until_ns{0};
  std::uint64_t reference_id{0};
  bool converged{false};
};
class Optimizer
{
public:
  bool ready() const noexcept;
  std::optional<SolverResult> solve(const RobotState & state, const RobotState & reference) const;
};
}  // namespace fr3_nmpc_controller
