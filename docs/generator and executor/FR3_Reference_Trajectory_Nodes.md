# Panda / FR3 Reference Trajectory Nodes

Complete source for the generator and executor. Execution defaults to disabled.

These sources have not been compiled in this environment because ROS is unavailable.

## reference_generator.cpp

```cpp
#include "panda_reference_trajectories/generators/path_generator.hpp"
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;
namespace panda_reference_trajectories {
class ReferenceGeneratorNode : public rclcpp::Node {
public:
  ReferenceGeneratorNode() : Node("reference_generator") {
    const auto frame = declare_parameter<std::string>("frame_id", "panda_link0");
    const auto topic = declare_parameter<std::string>("path_topic", "/panda_reference/cartesian_path");
    const auto type = declare_parameter<std::string>("path_type", "linear");
    Pose start;
    start.position.x = declare_parameter<double>("start_x", 0.40);
    start.position.y = declare_parameter<double>("start_y", -0.15);
    start.position.z = declare_parameter<double>("start_z", 0.35);
    start.orientation.x = declare_parameter<double>("orientation_x", 0.0);
    start.orientation.y = declare_parameter<double>("orientation_y", 0.0);
    start.orientation.z = declare_parameter<double>("orientation_z", 0.0);
    start.orientation.w = declare_parameter<double>("orientation_w", 1.0);
    Pose goal = start;
    goal.position.x = declare_parameter<double>("goal_x", 0.40);
    goal.position.y = declare_parameter<double>("goal_y", 0.15);
    goal.position.z = declare_parameter<double>("goal_z", 0.35);
    const auto step = declare_parameter<double>("step_size", 0.005);
    const auto radius = declare_parameter<double>("radius", 0.05);
    const auto a0 = declare_parameter<double>("start_angle", 0.0);
    const auto a1 = declare_parameter<double>("end_angle", 1.5707963267948966);
    if (frame.empty() || topic.empty() || !std::isfinite(step) || step <= 0.0)
      throw std::runtime_error("Frame/topic must be nonempty and step_size positive.");
    for (double v : {start.position.x, start.position.y, start.position.z,
         goal.position.x, goal.position.y, goal.position.z, radius, a0, a1})
      if (!std::isfinite(v)) throw std::runtime_error("Nonfinite path parameter.");
    const double n = std::sqrt(start.orientation.x * start.orientation.x +
      start.orientation.y * start.orientation.y + start.orientation.z * start.orientation.z +
      start.orientation.w * start.orientation.w);
    if (!std::isfinite(n) || n < 1e-6) throw std::runtime_error("Invalid quaternion.");
    start.orientation.x /= n; start.orientation.y /= n;
    start.orientation.z /= n; start.orientation.w /= n;
    goal.orientation = start.orientation;
    PoseSequence poses;
    if (type == "linear") poses = generateLinearPath(start, goal, step);
    else if (type == "arc") {
      if (radius <= 0.0) throw std::runtime_error("Arc radius must be positive.");
      poses = generateArcPath(start, radius, a0, a1, step);
    } else throw std::runtime_error("Unknown path_type: " + type);
    if (poses.empty()) throw std::runtime_error("Empty path.");
    path_.header.frame_id = frame;
    for (const auto & p : poses) {
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header.frame_id = frame; stamped.pose = p;
      path_.poses.push_back(stamped);
    }
    publisher_ = create_publisher<nav_msgs::msg::Path>(topic,
      rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(1s, [this]() {
      path_.header.stamp = now();
      for (auto & p : path_.poses) p.header.stamp = path_.header.stamp;
      publisher_->publish(path_);
    });
    RCLCPP_INFO(get_logger(), "%s path: %zu waypoints; topic=%s; frame=%s",
      type.c_str(), path_.poses.size(), topic.c_str(), frame.c_str());
  }
private:
  nav_msgs::msg::Path path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try { rclcpp::spin(std::make_shared<panda_reference_trajectories::ReferenceGeneratorNode>()); }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("reference_generator"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}
```

## cartesian_path_executor.cpp

```cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

template<class T> T parameter(const rclcpp::Node::SharedPtr & node,
  const std::string & name, const T & fallback) {
  if (node->has_parameter(name)) return node->get_parameter(name).get_value<T>();
  return node->declare_parameter<T>(name, fallback);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("cartesian_path_executor", options);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spinner;
  int result = 0;
  try {
    const auto topic = parameter<std::string>(node, "path_topic", "/panda_reference/cartesian_path");
    const auto group = parameter<std::string>(node, "planning_group", "arm");
    const auto tcp = parameter<std::string>(node, "eef_link", "");
    const bool execute = parameter<bool>(node, "execute", false);
    const bool keep_orientation = parameter<bool>(node, "keep_current_orientation", true);
    const double velocity = parameter<double>(node, "velocity_scaling", 0.05);
    const double acceleration = parameter<double>(node, "acceleration_scaling", 0.05);
    const double step = parameter<double>(node, "eef_step", 0.005);
    const double timeout = parameter<double>(node, "wait_timeout", 30.0);
    const double preview_seconds = parameter<double>(node, "preview_seconds", 10.0);
    const double start_tolerance = parameter<double>(node, "start_tolerance", 0.003);
    for (double v : {velocity, acceleration})
      if (!std::isfinite(v) || v <= 0 || v > 1) throw std::runtime_error("Scaling must be in (0,1].");
    for (double v : {step, timeout, start_tolerance})
      if (!std::isfinite(v) || v <= 0) throw std::runtime_error("Step/timeout/tolerance must be positive.");
    if (!std::isfinite(preview_seconds) || preview_seconds < 0)
      throw std::runtime_error("Invalid preview_seconds.");
    nav_msgs::msg::Path path;
    RCLCPP_INFO(node->get_logger(), "Waiting for one path on %s", topic.c_str());
    if (!rclcpp::wait_for_message(path, node, topic,
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout))))
      throw std::runtime_error("Path timeout. Start generator in the same ROS domain.");
    if (path.poses.size() < 2 || path.header.frame_id.empty())
      throw std::runtime_error("Path needs a frame and at least two poses.");
    executor.add_node(node);
    spinner = std::thread([&]() {executor.spin();});
    moveit::planning_interface::MoveGroupInterface move_group(node, group);
    if (!tcp.empty() && !move_group.setEndEffectorLink(tcp))
      throw std::runtime_error("Invalid eef_link.");
    // Require the model frame: no silent mixing of Cartesian coordinate frames.
    if (path.header.frame_id != move_group.getPlanningFrame())
      throw std::runtime_error("Path frame must equal MoveIt planning frame: " + move_group.getPlanningFrame());
    move_group.setPoseReferenceFrame(path.header.frame_id);
    move_group.setMaxVelocityScalingFactor(velocity);
    move_group.setMaxAccelerationScalingFactor(acceleration);
    auto start = move_group.getCurrentState(10.0);
    if (!start) throw std::runtime_error("No current robot state.");
    const auto link = move_group.getEndEffectorLink();
    if (link.empty() || !start->getRobotModel()->hasLinkModel(link))
      throw std::runtime_error("Set eef_link to a valid TCP.");
    const auto transform = start->getGlobalLinkTransform(link);
    const Eigen::Quaterniond orientation(transform.rotation());
    RCLCPP_INFO(node->get_logger(), "Group=%s; TCP=%s; frame=%s", group.c_str(), link.c_str(), path.header.frame_id.c_str());
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (const auto & stamped : path.poses) {
      if (!stamped.header.frame_id.empty() && stamped.header.frame_id != path.header.frame_id)
        throw std::runtime_error("Mixed waypoint frames.");
      auto p = stamped.pose;
      for (double v : {p.position.x, p.position.y, p.position.z})
        if (!std::isfinite(v)) throw std::runtime_error("Nonfinite waypoint.");
      if (keep_orientation) {
        p.orientation.x = orientation.x(); p.orientation.y = orientation.y();
        p.orientation.z = orientation.z(); p.orientation.w = orientation.w();
      } else {
        const double n = std::sqrt(p.orientation.x*p.orientation.x + p.orientation.y*p.orientation.y +
          p.orientation.z*p.orientation.z + p.orientation.w*p.orientation.w);
        if (!std::isfinite(n) || n < 1e-6) throw std::runtime_error("Invalid waypoint quaternion.");
        p.orientation.x /= n; p.orientation.y /= n; p.orientation.z /= n; p.orientation.w /= n;
      }
      waypoints.push_back(p);
    }
    const auto & first = waypoints.front().position;
    const double distance = (Eigen::Vector3d(first.x, first.y, first.z) - transform.translation()).norm();
    if (distance > start_tolerance)
      throw std::runtime_error("First waypoint is too far from current TCP; distance=" + std::to_string(distance));
    // No automatic approach motion. Exactly one Cartesian trajectory per invocation.
    move_group.setStartState(*start);
    moveit_msgs::msg::RobotTrajectory message;
    const double fraction = move_group.computeCartesianPath(waypoints, step, message, true);
    RCLCPP_INFO(node->get_logger(), "Cartesian completion: %.3f%%", fraction * 100);
    if (!std::isfinite(fraction) || fraction < 1.0 - 1e-6 || message.joint_trajectory.points.size() < 2)
      throw std::runtime_error("Require a complete, nontrivial Cartesian path.");
    robot_trajectory::RobotTrajectory trajectory(move_group.getRobotModel(), group);
    trajectory.setRobotTrajectoryMsg(*start, message);
    // IPTP assigns timing without resampling the geometric waypoints.
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, velocity, acceleration))
      throw std::runtime_error("Time parameterization failed.");
    trajectory.getRobotTrajectoryMsg(message);
    double previous = -1.0;
    for (const auto & point : message.joint_trajectory.points) {
      const double t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
      if (t < 0 || t <= previous) throw std::runtime_error("Trajectory timestamps must increase.");
      previous = t;
      const auto count = message.joint_trajectory.joint_names.size();
      if (point.positions.size() != count || point.velocities.size() != count || point.accelerations.size() != count)
        throw std::runtime_error("Incomplete trajectory fields.");
      for (const auto * values : {&point.positions, &point.velocities, &point.accelerations})
        for (double v : *values) if (!std::isfinite(v)) throw std::runtime_error("Nonfinite trajectory field.");
    }
    auto publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).reliable().transient_local());
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = move_group.getRobotModel()->getName();
    moveit::core::robotStateToRobotStateMsg(*start, display.trajectory_start);
    display.trajectory.push_back(message);
    // Keep publishing while RViz discovers the publisher.
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(preview_seconds);
    do {
      publisher->publish(display);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } while (rclcpp::ok() && std::chrono::steady_clock::now() < end);
    RCLCPP_INFO(node->get_logger(), "Timed trajectory duration: %.3f s", previous);
    if (execute && rclcpp::ok()) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::robotStateToRobotStateMsg(*start, plan.start_state);
      plan.trajectory = message;
      if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        throw std::runtime_error("Trajectory execution failed.");
      RCLCPP_INFO(node->get_logger(), "Execution completed.");
    } else RCLCPP_INFO(node->get_logger(), "Dry run complete; no motion executed.");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what()); result = 1;
  }
  executor.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown(); return result;
}
```

## Build and run instructions


These replace the two node sources inside your existing
`panda_reference_trajectories` package. They depend on your existing
`path_generator.hpp` and generator implementation library; this folder is not
a standalone ROS package. No local ROS installation was available for compilation.

## Build integration

Keep the existing generator target linked to your path-generation library.
For the executor, ensure CMakeLists.txt includes:

```cmake
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# If these targets already exist, edit them rather than adding duplicates.
add_executable(cartesian_path_executor src/cartesian_path_executor.cpp)
ament_target_dependencies(cartesian_path_executor
  rclcpp nav_msgs geometry_msgs moveit_core
  moveit_ros_planning_interface moveit_msgs)
install(TARGETS cartesian_path_executor DESTINATION lib/${PROJECT_NAME})
```

Use your actual source locations. Add any missing `<depend>` entries for the
above packages to package.xml. If your MoveIt distribution only provides `.h`
compatibility headers, adjust the corresponding `.hpp` includes to `.h`.

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
colcon build --packages-select panda_reference_trajectories
source install/setup.bash
```

## Real FR3: terminal setup

In EACH real-robot terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/panda_franka_robot/install/setup.bash
export ROS_DOMAIN_ID=20
```

Keep Gazebo on a separate domain such as 10. Do not launch Gazebo in domain 20.
Start official real FR3 MoveIt, replacing the IP placeholder:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=YOUR_FR3_IP
```

Obtain the current TCP pose:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

Verify these frames exist. The executor requires the path frame to equal its
MoveIt planning frame; use the frame it reports if your configuration differs.
With keep_current_orientation=true, the executor replaces generator orientations.

## Generator

Replace ALL uppercase coordinate placeholders with decimal numbers. Start at
the measured current TCP position and choose a short displacement in clear space.
These are templates, not directly runnable numeric commands.

```bash
ros2 run panda_reference_trajectories reference_generator --ros-args \
  -p use_sim_time:=false \
  -p frame_id:=fr3_link0 \
  -p path_topic:=/fr3_reference/cartesian_path \
  -p path_type:=linear \
  -p start_x:=CURRENT_X -p start_y:=CURRENT_Y -p start_z:=CURRENT_Z \
  -p goal_x:=GOAL_X -p goal_y:=GOAL_Y -p goal_z:=GOAL_Z \
  -p step_size:=0.005
```

Arc generation uses your existing generateArcPath implementation; consult its
definition for center/angle conventions before applying it to hardware.

## Executor: preview only

```bash
ros2 run panda_reference_trajectories cartesian_path_executor --ros-args \
  -p use_sim_time:=false \
  -p planning_group:=fr3_arm \
  -p eef_link:=fr3_hand_tcp \
  -p path_topic:=/fr3_reference/cartesian_path \
  -p keep_current_orientation:=true \
  -p execute:=false \
  -p velocity_scaling:=0.05 \
  -p acceleration_scaling:=0.05 \
  -p preview_seconds:=15.0
```

In RViz, select /display_planned_path in the MotionPlanning planned-path topic
or add a Trajectory display using that topic. The publisher stays alive for the
preview interval; rerun the dry run if RViz connects afterward.

The executor consumes one path, holds the current orientation by default,
requires the first position within 3 mm of the current TCP, rejects incomplete
Cartesian paths, and uses IPTP to retime the joint waypoints. It always enables
Cartesian collision checking against MoveIt's planning scene. Populate that
scene with the real table and obstacles; physical obstacles are not discovered
automatically. Timing covers velocity and acceleration, not a separate jerk
guarantee. Hardware compatibility must be verified on your installed stack.

There is NO automatic approach-to-start motion in this version. The old
move_to_start/minimum_fraction/avoid_collisions parameters are not used.
The current state must be available and the frame/TCP must match the real robot.

After building and inspecting a successful dry run and the actual trajectory,
the same executor command with execute:=true sends one trajectory to MoveIt.
The preview interval is a delay, not an interactive approval gate. A fresh
execution run recomputes from the current state and latest generated path.

## Simulation

Use domain 10, the simulation's actual planning group and TCP, and
use_sim_time:=true when Gazebo publishes /clock. Override frame_id and path_topic
for the generator accordingly. Package names do not determine the target robot.
