#pragma once

#include "fr3_nmpc_controller/robot_model.hpp"

namespace fr3_nmpc_controller
{

// Pure conversion of modeled total torque into a gravity-compensated
// Gazebo effort command, assuming the hardware adds gravity exactly once.
// This is NOT a controller, torque limiter, or real-time safety mechanism.
class TorqueAdapter
{
public:
  // output = total_torque - gravity_torque
  // Requires both inputs and resulting commands to be finite.
  static JointVector to_gazebo_effort(
    const JointVector & total_torque,
    const JointVector & gravity_torque);
};

}  // namespace fr3_nmpc_controller
