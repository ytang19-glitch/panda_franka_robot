#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include "fr3_nmpc_controller/robot_model.hpp"

TEST(RobotModel, GravityEquilibriumAndInvalidReload)
{
  namespace f = fr3_nmpc_controller;
  f::RobotModel model;
  ASSERT_NO_THROW(model.load(TEST_MODEL_URDF));
  ASSERT_TRUE(model.ready());
  f::RobotState x;
  x.q.fill(0.1);
  const auto g=model.gravity(x.q);
  for (double a:model.acceleration(x,g)) { EXPECT_NEAR(a,0.0,1e-8); }
  const auto y=model.step(x,g,0.01);
  for (std::size_t i=0;i<7;++i) { EXPECT_NEAR(y.q[i],x.q[i],1e-8); EXPECT_NEAR(y.dq[i],0.0,1e-8); }
  x.q[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(model.acceleration(x,g),std::invalid_argument);
  EXPECT_ANY_THROW(model.load("/nonexistent_fr3_model.urdf"));
  EXPECT_FALSE(model.ready());
}

TEST(RobotModel, InverseForwardDynamicsRoundTrip)
{
  namespace f=fr3_nmpc_controller;
  f::RobotModel model;
  model.load(TEST_MODEL_URDF);
  f::RobotState x;
  f::JointVector ddq{};
  for (std::size_t i=0;i<f::kJoints;++i) {
    x.q[i]=0.05*(i+1); x.dq[i]=-0.02*(i+1); ddq[i]=0.03*(i+1);
  }
  const auto recovered=model.acceleration(x,model.inverse_dynamics(x,ddq));
  for (std::size_t i=0;i<f::kJoints;++i) { EXPECT_NEAR(recovered[i],ddq[i],1e-8); }
  ddq[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(model.inverse_dynamics(x,ddq),std::invalid_argument);
}

