#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "fr3_nmpc_controller/optimizer.hpp"

TEST(Optimizer, FeasibleGravityHoldAndBadReference)
{
  namespace f = fr3_nmpc_controller;
  auto model=std::make_shared<f::RobotModel>();
  model->load(TEST_MODEL_URDF); // Synthetic fixture only; not FR3 validation.
  f::OptimizerConfig c;
  c.horizon=2; c.dt=.02; c.max_cpu_time_s=30;
  c.torque_limit.fill(100); c.torque_rate_limit.fill(1000); c.acceleration_limit.fill(100);
  f::Optimizer optimizer;
  optimizer.configure(model,c);
  f::RobotState x;
  std::vector<f::RobotState> refs(c.horizon+1,x);
  auto previous=model->gravity(x.q);
  for (int k=0;k<2;++k) {
    auto result=optimizer.solve(x,refs,previous);
    ASSERT_TRUE(result.has_value())<<optimizer.last_error();
    EXPECT_TRUE(result->converged);
    EXPECT_EQ(result->valid_until_ns,0); // Offline candidate, not hardware-authorized.
    EXPECT_EQ(result->torque_sequence.size(),c.horizon);
    for (std::size_t i=0;i<7;++i) {
      EXPECT_LE(std::abs(result->total_torque[i]),c.torque_limit[i]+1e-5);
      EXPECT_LE(std::abs(result->total_torque[i]-previous[i]),c.dt*c.torque_rate_limit[i]+1e-5);
    }
    previous=result->total_torque;
    x=model->step(x,previous,c.dt);
  }
  refs[0].q[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(optimizer.solve(x,refs,previous).has_value());
  EXPECT_FALSE(optimizer.last_error().empty());
}
