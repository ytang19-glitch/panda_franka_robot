#pragma once

#include <array>
#include <cstddef>

namespace fr3_nmpc_controller
{
constexpr std::size_t kJoints = 7;
using JointVector = std::array<double, kJoints>;
struct RobotState
{
  JointVector q{};
  JointVector dq{};
};

// Boundary for a future validated FR3 rigid-body model. No synthetic fallback.
class RobotModel
{
public:
  bool ready() const noexcept;
  JointVector acceleration(const RobotState & state, const JointVector & total_torque) const;
};
}  // namespace fr3_nmpc_controller
