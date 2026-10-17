#include "fr3_nmpc_controller/robot_model.hpp"
#include <stdexcept>

namespace fr3_nmpc_controller
{
bool RobotModel::ready() const noexcept { return false; }
JointVector RobotModel::acceleration(const RobotState &, const JointVector &) const
{
  throw std::logic_error("FR3 dynamics backend is not implemented or validated");
}
}  // namespace fr3_nmpc_controller
