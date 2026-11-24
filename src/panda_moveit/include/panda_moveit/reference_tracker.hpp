#pragma once
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
namespace panda_moveit {
void validateTiming(const moveit_msgs::msg::RobotTrajectory&, const std::string&);
void timeParameterize(robot_trajectory::RobotTrajectory&, double velocity, double acceleration);
}
