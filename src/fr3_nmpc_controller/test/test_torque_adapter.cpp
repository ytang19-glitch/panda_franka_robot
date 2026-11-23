#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "fr3_nmpc_controller/torque_adapter.hpp"

namespace fc = fr3_nmpc_controller;

TEST(TorqueAdapter, GravityOnlyProducesZeroCommand)
{
  const fc::JointVector gravity{{1.0, -25.0, 0.5, 18.0, 0.8, 1.5, -0.04}};
  const auto command = fc::TorqueAdapter::to_gazebo_effort(gravity, gravity);
  for (const auto value : command) { EXPECT_DOUBLE_EQ(value, 0.0); }
}

TEST(TorqueAdapter, ExtraEffortIsRetained)
{
  const fc::JointVector gravity{{1.0, -25.0, 0.5, 18.0, 0.8, 1.5, -0.04}};
  const fc::JointVector extra{{0.1, -0.2, 0.0, 0.4, -0.5, 0.6, -0.7}};
  fc::JointVector total{};
  for (std::size_t i = 0; i < fc::kJoints; ++i) { total[i] = gravity[i] + extra[i]; }
  const auto command = fc::TorqueAdapter::to_gazebo_effort(total, gravity);
  for (std::size_t i = 0; i < fc::kJoints; ++i) {
    EXPECT_NEAR(command[i], extra[i], 1e-12);
    EXPECT_NEAR(command[i] + gravity[i], total[i], 1e-12);
  }
}

TEST(TorqueAdapter, RejectsNonfiniteInputsAndOutput)
{
  fc::JointVector total{}, gravity{};
  total[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(fc::TorqueAdapter::to_gazebo_effort(total, gravity), std::invalid_argument);
  total[0] = 0.0;
  gravity[6] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(fc::TorqueAdapter::to_gazebo_effort(total, gravity), std::invalid_argument);
  gravity[6] = 0.0;
  total[1] = std::numeric_limits<double>::max();
  gravity[1] = -std::numeric_limits<double>::max();
  EXPECT_THROW(fc::TorqueAdapter::to_gazebo_effort(total, gravity), std::invalid_argument);
}
