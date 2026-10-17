#include <gtest/gtest.h>
#include <stdexcept>
#include "fr3_nmpc_controller/robot_model.hpp"
#include "fr3_nmpc_controller/optimizer.hpp"
#include "fr3_nmpc_controller/solver_worker.hpp"

TEST(UnimplementedBackends, CannotProduceAnActuatorCommand)
{
  namespace nmpc = fr3_nmpc_controller;
  nmpc::RobotModel model;
  nmpc::Optimizer optimizer;
  nmpc::SolverWorker worker;
  EXPECT_FALSE(model.ready());
  EXPECT_FALSE(optimizer.ready());
  EXPECT_FALSE(worker.ready());
  EXPECT_THROW(model.acceleration(nmpc::RobotState{}, nmpc::JointVector{}), std::logic_error);
  EXPECT_FALSE(optimizer.solve(nmpc::RobotState{}, nmpc::RobotState{}).has_value());
  EXPECT_FALSE(worker.latest().has_value());
}
