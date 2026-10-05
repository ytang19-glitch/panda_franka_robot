class ReferenceGeneratorNode : public rclcpp::Node
{
public:
  ReferenceGeneratorNode();

private:
  void generateReference();
  void publishCartesianPath();
  void publishJointTrajectory();

  std::string application_;
  std::string mode_;
  std::string base_frame_;
  std::string tcp_link_;

  double eef_step_;
  double jump_threshold_;
  double velocity_scale_;
  bool execute_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    reference_publisher_;
};