#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class SliderControl : public rclcpp::Node
{
    public:
        SliderControl(const std::string& node_name):rclcpp::Node(node_name){
            // 创建两个发布者
            arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "arm_controller/joint_trajectory", 10);
            gripper_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "gripper_controller/joint_trajectory", 10);

            // 订阅 joint_commands 话题
            sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_commands", 10,
            std::bind(&SliderControl::sliderCallback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "Slider Control Node started");
        }

    private:
        void sliderCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
            // 检查位置数量是否足够
            if (msg->position.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Received JointState has only %zu positions, expected at least 8.",
                        msg->position.size());
            return;
            }

            // 构建臂部轨迹消息
            trajectory_msgs::msg::JointTrajectory arm_traj;
            arm_traj.joint_names = {
            "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
            "panda_joint5", "panda_joint6", "panda_joint7"
            };
            trajectory_msgs::msg::JointTrajectoryPoint arm_point;
            arm_point.positions.assign(msg->position.begin(), msg->position.begin() + 7);
            arm_traj.points.push_back(arm_point);

            // 构建夹爪轨迹消息
            trajectory_msgs::msg::JointTrajectory gripper_traj;
            gripper_traj.joint_names = {"panda_finger_joint1"};
            trajectory_msgs::msg::JointTrajectoryPoint gripper_point;
            gripper_point.positions.push_back(msg->position[7]);
            gripper_traj.points.push_back(gripper_point);

            // 发布
            arm_pub_->publish(arm_traj);
            gripper_pub_->publish(gripper_traj);
        }

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SliderControl>("test_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}