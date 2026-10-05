#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

// Command-line overrides may already have declared a parameter.
template<typename T>
T get_or_declare_parameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("cartesian_path_executor", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
  int return_code = 0;

  try {
    const auto path_topic = get_or_declare_parameter<std::string>(
      node, "path_topic", "/panda_reference/cartesian_path");
    const auto planning_group = get_or_declare_parameter<std::string>(
      node, "planning_group", "arm");
    const bool execute_motion = get_or_declare_parameter<bool>(node, "execute", false);
    const bool move_to_start = get_or_declare_parameter<bool>(node, "move_to_start", true);
    const bool keep_current_orientation = get_or_declare_parameter<bool>(
      node, "keep_current_orientation", true);
    const bool avoid_collisions = get_or_declare_parameter<bool>(
      node, "avoid_collisions", true);
    const double velocity_scaling = get_or_declare_parameter<double>(
      node, "velocity_scaling", 0.1);
    const double acceleration_scaling = get_or_declare_parameter<double>(
      node, "acceleration_scaling", 0.1);
    const double eef_step = get_or_declare_parameter<double>(node, "eef_step", 0.01);
    const double minimum_fraction = get_or_declare_parameter<double>(
      node, "minimum_fraction", 0.99);
    const double planning_time = get_or_declare_parameter<double>(
      node, "planning_time", 10.0);
    const double wait_timeout = get_or_declare_parameter<double>(
      node, "wait_timeout", 30.0);

    if (!std::isfinite(wait_timeout) || wait_timeout <= 0.0 ||
      !std::isfinite(planning_time) || planning_time <= 0.0 ||
      !std::isfinite(eef_step) || eef_step <= 0.0)
    {
      throw std::runtime_error("wait_timeout, planning_time and eef_step must be finite and positive.");
    }
    if (!std::isfinite(velocity_scaling) || velocity_scaling <= 0.0 || velocity_scaling > 1.0 ||
      !std::isfinite(acceleration_scaling) || acceleration_scaling <= 0.0 ||
      acceleration_scaling > 1.0 || !std::isfinite(minimum_fraction) ||
      minimum_fraction <= 0.0 || minimum_fraction > 1.0)
    {
      throw std::runtime_error("Scaling factors and minimum_fraction must be in (0, 1].");
    }

    RCLCPP_INFO(node->get_logger(), "Waiting for one Cartesian path on %s...", path_topic.c_str());
    nav_msgs::msg::Path path_message;
    const auto timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(wait_timeout));

    // Do not add the node to our executor until wait_for_message has finished.
    if (!rclcpp::wait_for_message(path_message, node, path_topic, timeout)) {
      throw std::runtime_error("No Cartesian path received within the configured wait_timeout.");
    }
    if (path_message.poses.empty()) {
      throw std::runtime_error("Received path contains no waypoints.");
    }
    if (path_message.header.frame_id.empty()) {
      throw std::runtime_error("The received path has an empty frame_id.");
    }

    RCLCPP_INFO(node->get_logger(), "Received %zu waypoints in frame '%s'.",
      path_message.poses.size(), path_message.header.frame_id.c_str());

    executor.add_node(node);
    spinner = std::thread([&executor]() {executor.spin();});

    moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
    move_group.setPlanningTime(planning_time);
    move_group.setMaxVelocityScalingFactor(velocity_scaling);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
    move_group.setPoseReferenceFrame(path_message.header.frame_id);

    const auto current_state = move_group.getCurrentState(10.0);
    if (!current_state) {
      throw std::runtime_error("Could not obtain the robot's current state.");
    }
    const auto current_pose = move_group.getCurrentPose();
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(path_message.poses.size());
    for (const auto & stamped_pose : path_message.poses) {
      if (!stamped_pose.header.frame_id.empty() &&
        stamped_pose.header.frame_id != path_message.header.frame_id)
      {
        throw std::runtime_error("A waypoint frame differs from the path frame.");
      }
      auto pose = stamped_pose.pose;
      if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(pose.position.z))
      {
        throw std::runtime_error("A waypoint contains a non-finite position.");
      }
      const double quaternion_norm = std::sqrt(
        pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
      if (keep_current_orientation || quaternion_norm < 1.0e-6) {
        pose.orientation = current_pose.pose.orientation;
      } else {
        if (!std::isfinite(quaternion_norm)) {
          throw std::runtime_error("A waypoint contains a non-finite quaternion.");
        }
        pose.orientation.x /= quaternion_norm;
        pose.orientation.y /= quaternion_norm;
        pose.orientation.z /= quaternion_norm;
        pose.orientation.w /= quaternion_norm;
      }
      waypoints.push_back(pose);
    }

    move_group.setStartStateToCurrentState();
    if (move_to_start) {
      RCLCPP_INFO(node->get_logger(), "Planning an approach to the first waypoint.");
      move_group.setPoseTarget(waypoints.front());
      moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
      const bool approach_success =
        move_group.plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS;
      move_group.clearPoseTargets();
      if (!approach_success) {
        throw std::runtime_error("MoveIt could not plan an approach to the first waypoint.");
      }

      if (execute_motion) {
        RCLCPP_WARN(node->get_logger(), "Executing approach trajectory.");
        if (move_group.execute(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          throw std::runtime_error("Failed to execute the approach trajectory.");
        }
        move_group.setStartStateToCurrentState();
        RCLCPP_INFO(node->get_logger(), "Robot reached the first waypoint.");
      } else {
        // Preview the Cartesian segment from the planned approach endpoint.
        const auto & trajectory = approach_plan.trajectory.joint_trajectory;
        if (trajectory.points.empty() || trajectory.joint_names.empty() ||
          trajectory.points.back().positions.size() != trajectory.joint_names.size())
        {
          throw std::runtime_error("Approach trajectory has no valid endpoint.");
        }
        moveit::core::RobotState preview_start(*current_state);
        preview_start.setVariablePositions(
          trajectory.joint_names, trajectory.points.back().positions);
        preview_start.update();
        move_group.setStartState(preview_start);
        RCLCPP_INFO(node->get_logger(),
          "Dry run: approach planned; Cartesian preview starts at its planned endpoint.");
      }
    }

    moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
    RCLCPP_INFO(node->get_logger(), "Computing Cartesian trajectory with eef_step=%.4f m.", eef_step);
    const double fraction = move_group.computeCartesianPath(
      waypoints, eef_step, cartesian_trajectory, avoid_collisions);
    RCLCPP_INFO(node->get_logger(), "Cartesian path completion: %.2f%%", fraction * 100.0);
    if (!std::isfinite(fraction) || fraction < minimum_fraction ||
      cartesian_trajectory.joint_trajectory.points.empty())
    {
      throw std::runtime_error("Cartesian trajectory is empty or below the required minimum fraction.");
    }

    if (!execute_motion) {
      RCLCPP_INFO(node->get_logger(), "Dry run successful. No motion was executed.");
    } else {
      moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
      cartesian_plan.trajectory = cartesian_trajectory;
      RCLCPP_WARN(node->get_logger(), "Executing Cartesian trajectory.");
      if (move_group.execute(cartesian_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error("The Cartesian trajectory controller reported failure.");
      }
      RCLCPP_INFO(node->get_logger(), "Cartesian trajectory completed successfully.");
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(node->get_logger(), "Executor failed: %s", exception.what());
    return_code = 1;
  }

  executor.cancel();
  if (spinner.joinable()) {
    spinner.join();
  }
  rclcpp::shutdown();
  return return_code;
}
