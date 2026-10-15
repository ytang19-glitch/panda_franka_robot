#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
// Preview by default; execute only when explicitly enabled.
template<typename T>
T param(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & fallback)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, fallback);
}
// Validate timing within one trajectory segment.
// Each segment has its own time_from_start.
void validateTiming(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::string & segment_name)
{
  const auto & points = trajectory.joint_trajectory.points;
  if (points.size() < 2) {
    throw std::runtime_error(
      segment_name + ": requires at least two trajectory points");
  }
  if (rclcpp::Duration(points.front().time_from_start).nanoseconds() < 0) {
    throw std::runtime_error(
      segment_name + ": first timestamp is negative");
  }
  for (size_t i = 1; i < points.size(); ++i) {
    const auto previous =
      rclcpp::Duration(points[i - 1].time_from_start);
    const auto current =
      rclcpp::Duration(points[i].time_from_start);
    if (current <= previous) {
      throw std::runtime_error(
        segment_name +
        ": timestamps are not strictly increasing at point " +
        std::to_string(i));
    }
  }
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared(
    "cartesian_path_executor", options);
  auto display_publisher =
    node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path",
      rclcpp::QoS(1).reliable().transient_local());
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
  int result = 0;
  try {
    const auto topic = param<std::string>(
      node, "path_topic", "/fr3_reference/cartesian_path");
    const auto group = param<std::string>(
      node, "planning_group", "fr3_arm");
    const auto tcp_link = param<std::string>(
      node, "tcp_link", "fr3_hand_tcp");
    const bool execute =
      param<bool>(node, "execute", false);
    const bool move_to_start =
      param<bool>(node, "move_to_start", true);
    const bool keep_orientation =
      param<bool>(node, "keep_current_orientation", true);
    const bool avoid_collisions =
      param<bool>(node, "avoid_collisions", true);
    const double eef_step =
      param<double>(node, "eef_step", 0.005);
    const double minimum_fraction =
      param<double>(node, "minimum_fraction", 0.99);
    const double planning_time =
      param<double>(node, "planning_time", 10.0);
    const double wait_timeout =
      param<double>(node, "wait_timeout", 30.0);
    const double display_wait =
      param<double>(node, "display_wait", 10.0);

    if (topic.empty() || group.empty() || tcp_link.empty()) {
      throw std::runtime_error(
        "Topic, planning group and TCP link must not be empty");
    }
    if (!std::isfinite(eef_step) || eef_step <= 0.0 ||
        !std::isfinite(minimum_fraction) ||
        minimum_fraction <= 0.0 || minimum_fraction > 1.0 ||
        !std::isfinite(planning_time) || planning_time <= 0.0 ||
        !std::isfinite(wait_timeout) || wait_timeout <= 0.0 ||
        !std::isfinite(display_wait) || display_wait < 0.0) {
      throw std::runtime_error(
        "Invalid numeric planning/display parameters");
    }
    // 1. Receive the Cartesian path.
    nav_msgs::msg::Path path;
    const auto timeout =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(wait_timeout));
    RCLCPP_INFO(
      node->get_logger(),
      "Waiting for path on %s",
      topic.c_str());
    if (!rclcpp::wait_for_message(path, node, topic, timeout)) {
      throw std::runtime_error(
        "Timed out waiting for path; check bridge and QoS");
    }
    if (path.header.frame_id.empty() || path.poses.empty()) {
      throw std::runtime_error(
        "Path frame_id is empty or path has no waypoints");
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Received %zu waypoints in %s",
      path.poses.size(),
      path.header.frame_id.c_str());
    // 2. Start ROS callback processing.
    executor.add_node(node);
    spinner = std::thread(
      [&executor]() {
        executor.spin();
      });
    // 3. Configure MoveIt.
    moveit::planning_interface::MoveGroupInterface move_group(
      node, group);
    move_group.setPlanningTime(planning_time);
    move_group.setEndEffectorLink(tcp_link);
    const double velocity_scaling = param<double>(node, "velocity_scaling", 0.1);
    const double acceleration_scaling = param<double>(node, "acceleration_scaling", 0.1);
    if (!std::isfinite(velocity_scaling) || velocity_scaling <= 0 || velocity_scaling > 1 ||
        !std::isfinite(acceleration_scaling) || acceleration_scaling <= 0 || acceleration_scaling > 1) {
      throw std::runtime_error("Scaling factors must be in (0, 1]");
    }
    move_group.setMaxVelocityScalingFactor(velocity_scaling);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
    // No TF2 conversion is performed here.
    // Coordinates must already be valid in this reference frame.
    move_group.setPoseReferenceFrame(path.header.frame_id);
    const auto current_state = move_group.getCurrentState(10.0);
    if (!current_state) {
      throw std::runtime_error(
        "MoveIt did not provide a current robot state");
    }
    const auto current_pose = move_group.getCurrentPose(tcp_link);
    if (keep_orientation &&
        current_pose.header.frame_id != path.header.frame_id) {
      throw std::runtime_error(
        "Current TCP pose frame differs from path frame; TF2 required");
    }
    // 4. Validate and prepare Cartesian waypoints.
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(path.poses.size());
    for (const auto & stamped : path.poses) {
      if (!stamped.header.frame_id.empty() &&
          stamped.header.frame_id != path.header.frame_id) {
        throw std::runtime_error(
          "Waypoint frame differs from path frame");
      }
      auto pose = stamped.pose;
      if (!std::isfinite(pose.position.x) ||
          !std::isfinite(pose.position.y) ||
          !std::isfinite(pose.position.z)) {
        throw std::runtime_error(
          "Waypoint has a non-finite position");
      }
      const double qnorm = std::sqrt(
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w);
      if (!std::isfinite(qnorm) || qnorm < 1e-6) {
        throw std::runtime_error(
          "Waypoint has an invalid quaternion");
      }
      if (keep_orientation) {
        pose.orientation = current_pose.pose.orientation;
      } else {
        pose.orientation.x /= qnorm;
        pose.orientation.y /= qnorm;
        pose.orientation.z /= qnorm;
        pose.orientation.w /= qnorm;
      }
      waypoints.push_back(pose);
    }
    // 5. Set the initial state for RViz playback.
    moveit_msgs::msg::DisplayTrajectory display_msg;
    display_msg.model_id =
      move_group.getRobotModel()->getName();
    moveit::core::robotStateToRobotStateMsg(
      *current_state, display_msg.trajectory_start);
    move_group.setStartState(*current_state);
    // 6. Optionally plan the approach to the first waypoint.
    // This plans a preview; it does not move the robot.
    if (move_to_start) {
      move_group.setPoseTarget(waypoints.front(), tcp_link);
      moveit::planning_interface::MoveGroupInterface::Plan approach;
      const auto status = move_group.plan(approach);
      move_group.clearPoseTargets();
      if (status != moveit::core::MoveItErrorCode::SUCCESS) {
        throw std::runtime_error(
          "Failed to plan approach to first waypoint");
      }
      const auto & joint_traj =
        approach.trajectory.joint_trajectory;
      if (joint_traj.points.empty() ||
          joint_traj.joint_names.empty() ||
          joint_traj.points.back().positions.size() !=
          joint_traj.joint_names.size()) {
        throw std::runtime_error(
          "Invalid approach trajectory endpoint");
      }
      validateTiming(approach.trajectory, "Approach trajectory");
      RCLCPP_INFO(
        node->get_logger(),
        "Approach trajectory timing validation passed");
      moveit::core::RobotState preview(*current_state);
      preview.setVariablePositions(
        joint_traj.joint_names,
        joint_traj.points.back().positions);
      preview.update();
      // The Cartesian preview starts at the approach endpoint.
      move_group.setStartState(preview);
      display_msg.trajectory.push_back(approach.trajectory);
      RCLCPP_INFO(
        node->get_logger(),
        "Approach planned; Cartesian preview starts at its endpoint");
    }
    // 7. Compute the Cartesian trajectory.
    moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
    const double fraction = move_group.computeCartesianPath(
      waypoints,
      eef_step,
      cartesian_trajectory,
      avoid_collisions);
    RCLCPP_INFO(
      node->get_logger(),
      "Cartesian path completion: %.2f%%",
      fraction * 100.0);
    if (!std::isfinite(fraction) ||
        fraction < minimum_fraction ||
        cartesian_trajectory.joint_trajectory.points.empty()) {
      throw std::runtime_error(
        "Cartesian planning failed or path incomplete");
    }
    // 8. Validate Cartesian trajectory timestamps.
    moveit::core::RobotState cartesian_start(*current_state);
    if (move_to_start) {
      const auto & joints = display_msg.trajectory.front().joint_trajectory;
      cartesian_start.setVariablePositions(joints.joint_names, joints.points.back().positions);
      cartesian_start.update();
    }
    robot_trajectory::RobotTrajectory timed(move_group.getRobotModel(), group);
    timed.setRobotTrajectoryMsg(cartesian_start, cartesian_trajectory);
    // Zero path tolerance avoids intentional path blending during retiming.
    trajectory_processing::TimeOptimalTrajectoryGeneration timing(0.001);
    if (!timing.computeTimeStamps(timed, velocity_scaling, acceleration_scaling)) {
      throw std::runtime_error("Cartesian time parameterization failed");
    }
    timed.getRobotTrajectoryMsg(cartesian_trajectory);
    validateTiming(cartesian_trajectory, "Cartesian trajectory");
    RCLCPP_INFO(
      node->get_logger(),
      "Cartesian trajectory timing validation passed");
    const auto & points =
      cartesian_trajectory.joint_trajectory.points;
    const auto & last = points.back().time_from_start;
    RCLCPP_INFO(
      node->get_logger(),
      "Cartesian trajectory points: %zu; final time: %d.%09u s",
      points.size(),
      last.sec,
      last.nanosec);
    // 9. Publish both segments for RViz preview.
    display_msg.trajectory.push_back(cartesian_trajectory);
    display_publisher->publish(display_msg);
    RCLCPP_INFO(
      node->get_logger(),
      "Published %zu trajectory segment(s) to /display_planned_path",
      display_msg.trajectory.size());
    if (!execute) {
      RCLCPP_INFO(node->get_logger(), "Planning-only success; NO robot motion executed");
    } else {
      // Plan and validate BOTH segments before motion. Keep MoveIt's own checks enabled.
      for (size_t i = 0; i < display_msg.trajectory.size(); ++i) {
        const auto & segment = display_msg.trajectory[i];
        const auto & joints = segment.joint_trajectory;
        const auto actual = move_group.getCurrentState(10.0);
        if (!actual) {
          throw std::runtime_error("No current state before execution");
        }
        if (joints.points.empty() || joints.joint_names.empty() ||
            joints.points.front().positions.size() != joints.joint_names.size()) {
          throw std::runtime_error("Invalid segment start point");
        }
        constexpr double start_tolerance = 0.01;  // radians; do not widen to hide errors
        for (size_t j = 0; j < joints.joint_names.size(); ++j) {
          const auto & name = joints.joint_names[j];
          const double measured = actual->getVariablePosition(name);
          const double planned = joints.points.front().positions[j];
          if (!std::isfinite(measured) || !std::isfinite(planned) ||
              std::abs(measured - planned) > start_tolerance) {
            throw std::runtime_error("Segment start differs from actual state: " + name);
          }
        }
        if (!rclcpp::ok()) {
          throw std::runtime_error("ROS shutdown before execution");
        }
        RCLCPP_WARN(node->get_logger(), "Executing segment %zu/%zu",
                    i + 1, display_msg.trajectory.size());
        const auto status = move_group.execute(segment);
        if (status != moveit::core::MoveItErrorCode::SUCCESS) {
          move_group.stop();
          throw std::runtime_error("Execution failed for segment " + std::to_string(i + 1));
        }
      }
      RCLCPP_INFO(node->get_logger(), "All trajectory segments executed successfully");
    }
    std::this_thread::sleep_for(
      std::chrono::duration<double>(display_wait));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Executor failed: %s",
      e.what());
    result = 1;
  }

  executor.cancel();

  if (spinner.joinable()) {
    spinner.join();
  }
  
  rclcpp::shutdown();
  return result;
}

