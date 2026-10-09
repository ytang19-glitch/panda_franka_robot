#include "panda_reference_trajectories/generators/path_generator.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>


using namespace std::chrono_literals;

namespace panda_reference_trajectories
{

class ReferenceGeneratorNode : public rclcpp::Node
{
public:
  ReferenceGeneratorNode()
  : Node("reference_generator")
  {
    // Parameters can later be loaded from an application YAML file.
    const std::string frame_id =
      declare_parameter<std::string>(
      "frame_id", "panda_link0");

    const double start_x =
      declare_parameter<double>("start_x", 0.40);

    const double start_y =
      declare_parameter<double>("start_y", -0.15);

    const double start_z =
      declare_parameter<double>("start_z", 0.35);

    const double goal_x =
      declare_parameter<double>("goal_x", 0.40);

    const double goal_y =
      declare_parameter<double>("goal_y", 0.15);

    const double goal_z =
      declare_parameter<double>("goal_z", 0.35);

    const double step_size =
      declare_parameter<double>("step_size", 0.005);
    
    const std::string path_type =
      declare_parameter<std::string>("path_type", "linear");

    const double radius =
      declare_parameter<double>("radius", 0.05);

    const double start_angle =
      declare_parameter<double>("start_angle", 0.0);
 
    const double end_angle =
      declare_parameter<double>("end_angle", 1.5707963267948966);


    Pose start;
    start.position.x = start_x;
    start.position.y = start_y;
    start.position.z = start_z;

    // Valid identity quaternion.
    start.orientation.x = 0.0;
    start.orientation.y = 0.0;
    start.orientation.z = 0.0;
    start.orientation.w = 1.0;

    Pose goal = start;
    goal.position.x = goal_x;
    goal.position.y = goal_y;
    goal.position.z = goal_z;

    PoseSequence waypoints;

    if (path_type == "linear") {
      waypoints = generateLinearPath(start, goal, step_size);
    } else if (path_type == "arc") {
      waypoints = generateArcPath(start, radius, start_angle, end_angle, step_size);
    } else {
      throw std::runtime_error("Invalid path_type parameter: " + path_type);
    }

    if (waypoints.empty()) {
      throw std::runtime_error("The generator returned no waypoints.");
    }

    cartesian_path_.header.frame_id = frame_id;

    for (const Pose & pose : waypoints) {
      geometry_msgs::msg::PoseStamped stamped_pose;

      stamped_pose.header.frame_id = frame_id;
      stamped_pose.pose = pose;

      cartesian_path_.poses.push_back(stamped_pose);
    }

    /*
     * Transient-local QoS allows RViz to receive the most recent path
     * even if RViz subscribes after the first publication.
     */
    publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/panda_reference/cartesian_path",
      rclcpp::QoS(1).reliable().transient_local());

    // Republish once per second for easier command-line testing.
    timer_ = create_wall_timer(
      1s,
      [this]()
      {
        const rclcpp::Time stamp = now();

        cartesian_path_.header.stamp = stamp;

        for (auto & pose : cartesian_path_.poses) {
          pose.header.stamp = stamp;
        }

        publisher_->publish(cartesian_path_);
      });

    RCLCPP_INFO(
      get_logger(),
      "Linear Cartesian reference generated with %zu waypoints.",
      path_type.c_str(),
      cartesian_path_.poses.size());

    RCLCPP_INFO(
      get_logger(),
      "Publishing: /panda_reference/cartesian_path");
  }

private:
  nav_msgs::msg::Path cartesian_path_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace panda_reference_trajectories


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<
    panda_reference_trajectories::ReferenceGeneratorNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
