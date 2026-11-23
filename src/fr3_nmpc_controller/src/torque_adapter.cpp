#include "fr3_nmpc_controller/torque_adapter.hpp"

#include <cmath>
#include <stdexcept>

namespace fr3_nmpc_controller
{

JointVector TorqueAdapter::to_gazebo_effort(
  const JointVector & total_torque,
  const JointVector & gravity_torque)
{
  JointVector command{};
  for (std::size_t i = 0; i < kJoints; ++i) {
    if (!std::isfinite(total_torque[i]) || !std::isfinite(gravity_torque[i])) {
      throw std::invalid_argument("TorqueAdapter requires finite torque inputs");
    }
    command[i] = total_torque[i] - gravity_torque[i];
    if (!std::isfinite(command[i])) {
      throw std::invalid_argument("TorqueAdapter produced non-finite effort");
    }
  }
  return command;
}

}  // namespace fr3_nmpc_controller
