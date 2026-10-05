#ifndef PANDA_REFERENCE_TRAJECTORIES__TRAJECTORY_BUILDER_HPP_
#define PANDA_REFERENCE_TRAJECTORIES__TRAJECTORY_BUILDER_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <vector>

namespace panda_reference_trajectories
{

class TrajectoryBuilder
{
public:
  explicit TrajectoryBuilder(
    moveit::planning_interface::MoveGroupInterface & move_group);

  bool computeCartesianTrajectory(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double eef_step,
    double jump_threshold,
    moveit_msgs::msg::RobotTrajectory & trajectory);

  bool executeTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory);

private:
  moveit::planning_interface::MoveGroupInterface & move_group_;
};

}  // namespace panda_reference_trajectories

#endif