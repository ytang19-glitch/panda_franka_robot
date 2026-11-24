#include "panda_moveit/reference_tracker.hpp"
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <stdexcept>
namespace panda_moveit {
void timeParameterize(robot_trajectory::RobotTrajectory& t,double v,double a){
  if(!(v>0&&v<=1&&a>0&&a<=1)) throw std::invalid_argument("Invalid scaling");
  trajectory_processing::TimeOptimalTrajectoryGeneration timing(0.001);
  if(!timing.computeTimeStamps(t,v,a)) throw std::runtime_error("Retiming failed");
}
}
