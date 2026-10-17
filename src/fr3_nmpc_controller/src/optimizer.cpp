#include "fr3_nmpc_controller/optimizer.hpp"

namespace fr3_nmpc_controller
{
bool Optimizer::ready() const noexcept { return false; }
std::optional<SolverResult> Optimizer::solve(const RobotState &, const RobotState &) const
{
  // TODO: implement horizon, nonlinear dynamics, costs, limits and warm start.
  // No result is preferable to presenting zero torque as an optimized command.
  return std::nullopt;
}
}  // namespace fr3_nmpc_controller
