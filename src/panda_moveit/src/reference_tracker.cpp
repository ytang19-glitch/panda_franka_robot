#include "panda_moveit/reference_tracker.hpp"
#include "panda_moveit/cost_functions.hpp"
#include "panda_moveit/application_cost.hpp"
#include "panda_moveit/collision_distance.hpp"
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <future>
#include <algorithm>
#include <limits>
#include <fstream>
#include <iomanip>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
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
using panda_moveit::validateTiming;
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared(
    "reference_tracker", options);
  auto display_publisher =
    node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path",
      rclcpp::QoS(1).reliable().transient_local());
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
  int result = 0;
  try {
    const auto topic = param<std::string>(
      node, "path_topic", "/panda_reference/cartesian_path");
    const auto group = param<std::string>(
      node, "planning_group", "arm");
    const auto tcp_link = param<std::string>(
      node, "tcp_link", "panda_hand");
    const bool execute =
      param<bool>(node, "execute", false);
    if (execute) { throw std::runtime_error("New tracker is planning-only; use the original executor for separately validated execution"); }
    const bool move_to_start =
      param<bool>(node, "move_to_start", true);
    const bool keep_orientation =
      param<bool>(node, "keep_current_orientation", true);
    const bool avoid_collisions =
      param<bool>(node, "avoid_collisions", true);
    const double eef_step =
      param<double>(node, "eef_step", 0.005);
    const double minimum_fraction =
      param<double>(node, "minimum_fraction", 1.0);
    const double planning_time =
      param<double>(node, "planning_time", 10.0);
    const double wait_timeout =
      param<double>(node, "wait_timeout", 30.0);
    const double display_wait =
      param<double>(node, "display_wait", 10.0);

    if (!avoid_collisions || minimum_fraction != 1.0) throw std::runtime_error("New tracker requires collision checking and full completion");
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
    panda_moveit::timeParameterize(timed, velocity_scaling, acceleration_scaling);
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
    // Evaluate the retimed process segment only, using a frozen scene snapshot.
    // This is a diagnostic, NOT a modification to MoveIt's optimization objective.
    const auto application=param<std::string>(node,"application_name","baseline");
    const auto weights=param<std::vector<double>>(node,"joint_weights",{1,1,10,10,1,1,1});
    const panda_moveit::CostTerms coefficients{
      param<double>(node,"joint_cost_weight",1.0),
      param<double>(node,"singularity_cost_weight",0.0),
      param<double>(node,"collision_cost_weight",0.0),
      param<double>(node,"smoothness_cost_weight",0.0),
      param<double>(node,"joint_limit_cost_weight",0.0),
      param<double>(node,"orientation_cost_weight",0.0),
      param<double>(node,"wrist_cost_weight",0.0)};
    const double safe=param<double>(node,"desired_clearance",0.10);
    const double length=param<double>(node,"characteristic_length",0.30);
    const double sigma_scale=param<double>(node,"singularity_scale",0.10);
    const double step_scale=param<double>(node,"joint_step_scale",0.10);
    const auto csv=param<std::string>(node,"cost_csv_path","");
    const double joint_margin=param<double>(node,"desired_joint_margin",0.20);
    const double orientation_scale=param<double>(node,"orientation_scale",0.0872664626);
    const bool margin_mode=param<bool>(node,"singularity_margin_mode",false);
    const double desired_sigma=param<double>(node,"desired_sigma",0.10);
    const auto wrist_name=param<std::string>(node,"wrist_joint_name","");
    const double wrist_desired=param<double>(node,"wrist_desired_angle",0.3490658504);
    const bool wrist_positive=param<bool>(node,"wrist_positive_branch",true);
    for(double s:{joint_margin,orientation_scale,desired_sigma,wrist_desired})
      if(!std::isfinite(s)||s<=0) throw std::runtime_error("Invalid surfacing scale");
    for(double scale:{safe,length,sigma_scale,step_scale})
      if(!std::isfinite(scale)||scale<=0) throw std::runtime_error("Cost scales must be positive");
    const auto& jt=cartesian_trajectory.joint_trajectory;
    const auto model=move_group.getRobotModel();
    const auto* jmg=model->getJointModelGroup(group);
    const auto* link=model->getLinkModel(tcp_link);
    if(!jmg||!link||jmg->getVariableNames()!=jt.joint_names||weights.size()!=jt.joint_names.size())
      throw std::runtime_error("Expected trajectory joint order to match group and cost weights");
    Eigen::VectorXd w=Eigen::Map<const Eigen::VectorXd>(weights.data(),weights.size());
    Eigen::MatrixXd path_matrix(jt.points.size(),weights.size());
    for(size_t k=0;k<jt.points.size();++k)
      path_matrix.row(k)=Eigen::Map<const Eigen::VectorXd>(jt.points[k].positions.data(),weights.size()).transpose();
    double movement=0;
    for(Eigen::Index k=1;k<path_matrix.rows();++k)
      movement+=panda_moveit::jointCost(path_matrix.row(k).transpose(),path_matrix.row(k-1).transpose(),w);

    auto client=node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if(!client->wait_for_service(std::chrono::seconds(10))) throw std::runtime_error("No /get_planning_scene service");
    auto request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    using C=moveit_msgs::msg::PlanningSceneComponents;
    request->components.components=C::SCENE_SETTINGS|C::ROBOT_STATE|C::ROBOT_STATE_ATTACHED_OBJECTS|
      C::WORLD_OBJECT_NAMES|C::WORLD_OBJECT_GEOMETRY|C::OCTOMAP|C::TRANSFORMS|C::ALLOWED_COLLISION_MATRIX|C::LINK_PADDING_AND_SCALING;
    auto future=client->async_send_request(request);
    if(future.wait_for(std::chrono::seconds(10))!=std::future_status::ready)
      throw std::runtime_error("Planning scene request timed out");
    planning_scene::PlanningScene scene(model);
    const auto snapshot=future.get()->scene;
    if(snapshot.is_diff) throw std::runtime_error("Expected a full planning scene");
    scene.setPlanningSceneMsg(snapshot);
    moveit::core::RobotState state(scene.getCurrentState());
    double singularity=0, clearance=0, joint_limit=0, orientation=0, wrist=0;
    double min_sigma=std::numeric_limits<double>::infinity();
    double min_margin=std::numeric_limits<double>::infinity();
    double min_manip=std::numeric_limits<double>::infinity(), max_condition=0, max_orientation=0;
    // For constant nominal orientation only; varying-orientation paths require
    // an explicit reference correspondence after retiming.
    const auto& nominal=waypoints.front().orientation;
    const Eigen::Quaterniond desired(nominal.w,nominal.x,nominal.y,nominal.z);
    if(coefficients[5]>0) {
      if(!scene.knowsFrameTransform(path.header.frame_id))
        throw std::runtime_error("Unknown orientation reference frame");
      for(const auto& p:waypoints) {
        const Eigen::Quaterniond r(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
        if(panda_moveit::orientationError(r,desired)>1e-6)
          throw std::runtime_error("Orientation scoring currently requires constant nominal orientation");
      }
    }
    if(coefficients[6]>0 && (wrist_name.empty() ||
       std::find(jt.joint_names.begin(),jt.joint_names.end(),wrist_name)==jt.joint_names.end() ||
       model->getName().find("panda")!=std::string::npos || model->getName().find("fr3")!=std::string::npos))
      throw std::runtime_error("Wrist policy requires verified non-Panda/FR3 robot and joint name");
    Eigen::Quaterniond desired_global=desired;
    if(coefficients[5]>0)
      desired_global=Eigen::Quaterniond(scene.getFrameTransform(path.header.frame_id).linear())*desired;
    std::vector<std::string> bounded;
    for(const auto& name:jt.joint_names)
      if(model->getVariableBounds(name).position_bounded_) bounded.push_back(name);
    Eigen::VectorXd lo(bounded.size()), hi(bounded.size()), values(bounded.size());
    for(size_t i=0;i<bounded.size();++i) {
      const auto& bounds=model->getVariableBounds(bounded[i]);
      lo[i]=bounds.min_position_; hi[i]=bounds.max_position_;
    }
    double min_distance=std::numeric_limits<double>::infinity();
    size_t invalid_samples=0;
    for(const auto& point:jt.points) {
      state.setVariablePositions(jt.joint_names,point.positions);
      state.update();
      Eigen::MatrixXd jac;
      if(!state.getJacobian(jmg,link,Eigen::Vector3d::Zero(),jac))
        throw std::runtime_error("Cannot compute TCP Jacobian");
      // Convert angular rows to a comparable translational scale.
      jac.bottomRows(3)*=length;
      const auto metrics=panda_moveit::jacobianMetrics(jac);
      min_sigma=std::min(min_sigma,metrics.sigma_min);
      min_manip=std::min(min_manip,metrics.manipulability);
      max_condition=std::max(max_condition,metrics.condition_number);
      singularity+=margin_mode?panda_moveit::singularityMarginCost(metrics.sigma_min,desired_sigma):
        sigma_scale*sigma_scale/(metrics.sigma_min*metrics.sigma_min+1e-6);
      for(size_t i=0;i<bounded.size();++i){
        values[i]=state.getVariablePosition(bounded[i]);
        min_margin=std::min(min_margin,std::min(values[i]-lo[i],hi[i]-values[i]));
      }
      if(!bounded.empty()) joint_limit+=panda_moveit::jointLimitCost(values,lo,hi,joint_margin)/bounded.size();
      if(coefficients[5]>0){
        const Eigen::Quaterniond actual(state.getGlobalLinkTransform(link).linear());
        const double error=panda_moveit::orientationError(actual,desired_global);
        max_orientation=std::max(max_orientation,error);
        orientation+=error*error/(orientation_scale*orientation_scale);
      }
      if(coefficients[6]>0)
        wrist+=panda_moveit::wristSingularityCost(state.getVariablePosition(wrist_name),wrist_desired,wrist_positive);
      const auto measured=panda_moveit::measureClearance(scene,state);
      if(measured.colliding||!state.satisfiesBounds()) ++invalid_samples;
      min_distance=std::min(min_distance,measured.distance);
      clearance+=panda_moveit::collisionCost(measured.distance,safe)/(safe*safe);
    }
    const double n=static_cast<double>(jt.points.size());
    const panda_moveit::CostTerms terms{
      movement/((n-1)*step_scale*step_scale),
      singularity/n,
      clearance/n,
      panda_moveit::smoothnessCost(path_matrix)/(std::max(1.0,n-2)*step_scale*step_scale),
      joint_limit/n,orientation/n,wrist/n};
    const double total=panda_moveit::combinedCost(terms,coefficients);
    RCLCPP_INFO(node->get_logger(),"Application %s (SCORING ONLY)",application.c_str());
    RCLCPP_INFO(node->get_logger(),"Normalized terms: joint=%.8f singularity=%.8f clearance=%.8f smoothness=%.8f",terms[0],terms[1],terms[2],terms[3]);
    RCLCPP_INFO(node->get_logger(),"Weights: %.3f %.3f %.3f %.3f; total=%.8f; min_world_clearance=%.6g m; invalid_samples=%zu",coefficients[0],coefficients[1],coefficients[2],coefficients[3],total,min_distance,invalid_samples);
    RCLCPP_INFO(node->get_logger(),"Surfacing terms: joint_margin=%.8f orientation=%.8f wrist=%.8f",terms[4],terms[5],terms[6]);
    RCLCPP_INFO(node->get_logger(),"Metrics: min_sigma=%.8g min_manipulability=%.8g max_condition=%.8g min_joint_margin=%.8g rad",min_sigma,min_manip,max_condition,min_margin);
    if(coefficients[5]>0) RCLCPP_INFO(node->get_logger(),"Max nominal orientation error: %.8g rad",max_orientation);
    if(!csv.empty()) {
      std::ofstream out(csv);
      if(!out) throw std::runtime_error("Cannot open cost CSV");
      out<<std::setprecision(16)<<"application,joint,singularity,clearance,smoothness,total,min_world_clearance_m,invalid_samples,joint_margin,orientation,wrist,min_sigma,min_manipulability,max_condition,min_joint_margin_rad,max_orientation_error_rad\n"
         <<application<<","<<terms[0]<<","<<terms[1]<<","<<terms[2]<<","<<terms[3]<<","<<total<<","<<min_distance<<","<<invalid_samples<<","<<terms[4]<<","<<terms[5]<<","<<terms[6]<<","<<min_sigma<<","<<min_manip<<","<<max_condition<<","<<min_margin<<",";
      if(coefficients[5]>0) out<<max_orientation;
      out<<"\n";
      out.flush();
      if(!out) throw std::runtime_error("Cannot write cost CSV");
    }
    if(invalid_samples) throw std::runtime_error("Retimed process path contains invalid sampled states; preview rejected");
    // Sample checks are not continuous collision or Cartesian accuracy validation.
    // 9. Publish both segments for RViz preview.
    display_msg.trajectory.push_back(cartesian_trajectory);
    display_publisher->publish(display_msg);
    RCLCPP_INFO(
      node->get_logger(),
      "Published %zu trajectory segment(s) to /display_planned_path",
      display_msg.trajectory.size());
    RCLCPP_INFO(node->get_logger(), "Planning-only preview; no robot motion executed");
    // Export deliberately disabled until retimed path validation is implemented.
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

