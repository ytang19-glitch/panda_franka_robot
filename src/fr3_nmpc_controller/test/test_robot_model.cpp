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
