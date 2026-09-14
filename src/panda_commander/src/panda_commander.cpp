#include <rclcpp/rclcpp.hpp>
#include <moveit_ros_planning_interface/moveit/move_group_interface/move_group_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sstream>
#include <vector>
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using JointState = sensor_msgs::msg::JointState;
using String = std_msgs::msg::String;
using namespace std::placeholders;
class Commander{
    public:
        Commander(std::shared_ptr<rclcpp::Node> node):node_(node){
            target_color_ = node_->declare_parameter<std::string>("target_color", "R");
            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
            arm_->setPoseReferenceFrame("panda_link0");arm_->setMaxVelocityScalingFactor(0.5);arm_->setMaxAccelerationScalingFactor(0.5);
            gripper_->setPoseReferenceFrame("panda_link0");gripper_->setMaxVelocityScalingFactor(1.0);gripper_->setMaxAccelerationScalingFactor(1.0);
            color_sub_ = node_->create_subscription<String>("/color_coordinates", 10,
                                std::bind(&Commander::Color_Sub_Callback, this, _1));
            trash_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/model/Trash_01_001/pose", 10,
                                std::bind(&Commander::Trash_Pose_Sub_Callback, this, _1));
            joint_states_sub_ = node_->create_subscription<JointState>("/joint_states", 10,
                                std::bind(&Commander::Joint_State_Sub_Callback, this, _1));
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            RCLCPP_INFO(node_->get_logger(), "Commander initialized. Target color: %s", target_color_.c_str());
        }
        void OpenGripper(){
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("open");
            PlayAndExecute(gripper_);
        }
        void CloseGripper(){
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("close");
            PlayAndExecute(gripper_);
        }
        void GoToPoseTarget(float x, float y, float z, float roll, float pitch, float yaw, 
                            const std::shared_ptr<MoveGroupInterface> interface, 
                            const std::string reference_frame, bool cartesian_path=false){
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q = q.normalize();
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = reference_frame;  
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x = q.getX();
            target_pose.pose.orientation.y = q.getY();
            target_pose.pose.orientation.z = q.getZ();
            target_pose.pose.orientation.w = q.getW();
            if(!cartesian_path){
                interface->setStartStateToCurrentState();
                interface->setPoseTarget(target_pose);
                PlayAndExecute(interface);
            }
            else{
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = interface->computeCartesianPath(waypoints, 0.01, trajectory);
                if(fraction >= 0.90){
                    interface->execute(trajectory);
                }
                else{
                    RCLCPP_WARN(node_->get_logger(), "ComputeCartesianPath failed, falling back to normal planning");
                    interface->setStartStateToCurrentState();
                    interface->setPoseTarget(target_pose);
                    PlayAndExecute(interface);
                }
            }
        }
        void WaitforGripperOpen(){
            // get target value of panda_finger_joint1 from open group_state
            auto target_map = gripper_->getNamedTargetValues("open"); 
            double target_value = target_map["panda_finger_joint1"];
            double tolerance = 0.1;
            float timeout_sec = 5.0;
            // to judge if the gripper fully opend 
            auto start = std::chrono::steady_clock::now();
            rclcpp::Rate rate(10);
            while (rclcpp::ok()){
                if (std::fabs(gripper_finger_value - target_value) <= tolerance) {
                    RCLCPP_INFO(node_->get_logger(), "Gripper fully opened");
                    return;
                }
                auto elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
                if (elapsed > timeout_sec) {
                    RCLCPP_WARN(node_->get_logger(), "WaitforGripperOpen timeout");
                    break;
                }
                rate.sleep();
            }
        }
        void WaitforGripperClose(){
            // get target value of panda_finger_joint1 from close group_state
            auto target_map = gripper_->getNamedTargetValues("close"); 
            double target_value = target_map["panda_finger_joint1"];
            double tolerance = 0.1;
            float timeout_sec = 5.0;
            // to judge if the gripper fully closed 
            auto start = std::chrono::steady_clock::now();
            rclcpp::Rate rate(10);
            while (rclcpp::ok()){
                if (std::fabs(gripper_finger_value - target_value) <= tolerance) {
                    RCLCPP_INFO(node_->get_logger(), "Gripper fully closed");
                    return;
                }
                auto elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
                if (elapsed > timeout_sec) {
                    RCLCPP_WARN(node_->get_logger(), "WaitforGripperClose timeout");
                    break;
                }
                rate.sleep();
            }
        }
        void EnsureGraspBox(){
            double threshold = 0.02;
            float timeout_sec = 5.0;
            float stable_duration = 1.0; 
            auto start = std::chrono::steady_clock::now();
            double last_value = std::numeric_limits<double>::quiet_NaN();
            auto stable_start = start; 
            rclcpp::Rate rate(10);
            while(rclcpp::ok()){
                double current_value = gripper_finger_value;
                if(std::isnan(last_value) || std::fabs(current_value - last_value) < threshold){
                    // 如果尚未记录稳定起点，则记录当前时间
                    if(std::isnan(last_value)){
                        stable_start = std::chrono::steady_clock::now();
                    } 
                    // 检查稳定持续时间
                    else{
                        auto elapsed_stable = std::chrono::duration<float>(std::chrono::steady_clock::now()-stable_start).count();
                        if(elapsed_stable >= stable_duration){
                            RCLCPP_INFO(node_->get_logger(),"Grasp successful");
                            return;
                        }
                    }
                }
                else{
                    // 位置发生变化，重置稳定计时
                    stable_start = std::chrono::steady_clock::now();
                }
                last_value = current_value;

                auto elapsed_total = std::chrono::duration<float>(std::chrono::steady_clock::now()-start).count();
                if(elapsed_total > timeout_sec){
                    RCLCPP_WARN(node_->get_logger(), "Grasp may have failed");
                    return;
                }
                rate.sleep();
            }
        }
    private:
        void PlayAndExecute(const std::shared_ptr<MoveGroupInterface> interface){
            MoveGroupInterface::Plan plan;
            bool success =(interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if(success){
                interface->execute(plan);
            }
            else{
                RCLCPP_WARN(node_->get_logger(), "Planning failed for interface.");
            }
        }
        void Trash_Pose_Sub_Callback(const geometry_msgs::msg::PoseStamped &msg){
            geometry_msgs::msg::PoseStamped msg_world = msg;
            msg_world.header.frame_id = "world";
            try {
                Trash_pose_in_panda_link0 = tf_buffer_->transform(msg_world, "panda_link0", tf2::durationFromSec(0.5));
                RCLCPP_INFO(node_->get_logger(), 
                    "Trash position in panda_link0: x=%.3f, y=%.3f, z=%.3f",Trash_pose_in_panda_link0.pose.position.x,
                    Trash_pose_in_panda_link0.pose.position.y,Trash_pose_in_panda_link0.pose.position.z);
            } 
            catch(const tf2::TransformException &ex){
                RCLCPP_WARN(node_->get_logger(), "Transform failed: %s", ex.what());
            }
            trash_pose_sub_.reset();
        }
        void Color_Sub_Callback(const String &msg){
            std::string data = msg.data;
            std::vector<std::string> parts;
            std::stringstream ss(data);
            std::string item;
            while(std::getline(ss, item, ',')){
                parts.push_back(item);
            }
            if(parts.size() != 4){
                RCLCPP_WARN(node_->get_logger(), "Invalid format: %s", data.c_str());
                return;
            }
            std::string color = parts[0];
            double x = std::stod(parts[1]);
            double y = std::stod(parts[2]);
            double z = std::stod(parts[3]);
            if(color != target_color_){
                RCLCPP_DEBUG(node_->get_logger(), "Ignored color: %s", color.c_str());
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Target color detected! locate at (%.3f, %.3f, %.3f)", x,y,z);
            Pick_and_Place(x, y, z);
            RCLCPP_INFO(node_->get_logger(), "Already picked up target, ignoring new coordinates.");
            color_sub_.reset();
        }
        void Joint_State_Sub_Callback(const JointState &msg){
            auto it = joint_index_map_.find("panda_finger_joint1");
            if (it != joint_index_map_.end()){
                size_t idx = it->second;
                if (idx < msg.position.size()) gripper_finger_value = msg.position[idx];
            }
        }
        void Pick_and_Place(float x, float y, float z){
            //action 1
            float target_pose_offset_x = 0.005; float target_pose_offset_y = 0.005; float target_pose_offset_z = 0.3;
            x -= target_pose_offset_x; y += target_pose_offset_y; z += target_pose_offset_z;
            GoToPoseTarget(x, y, z, M_PI, 0.0, 0.0, arm_, "panda_link0", false);

            //action2
            OpenGripper();
            WaitforGripperOpen();

            //action3
            float grasp_down_offset = 0.15;
            z -= grasp_down_offset;
            GoToPoseTarget(x, y, z, M_PI, 0.0, 0.0, arm_, "panda_link0", true);

            //action4
            CloseGripper();
            EnsureGraspBox();  

            //action5
            float grasp_lift_offset = 0.4;
            z += grasp_lift_offset;
            GoToPoseTarget(x, y, z, M_PI, 0.0, 0.0, arm_, "panda_link0", true);

            //action6
            GoToPoseTarget(Trash_pose_in_panda_link0.pose.position.x, Trash_pose_in_panda_link0.pose.position.y, z, 
                            M_PI, 0.0, 0.0, arm_, "panda_link0", false);

            //action7
            OpenGripper();
            WaitforGripperOpen();

            //action8
            arm_->setStartStateToCurrentState();
            arm_->setNamedTarget("home");
            PlayAndExecute(arm_);

            //action9
            CloseGripper();
            WaitforGripperClose();
        }
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        std::shared_ptr<MoveGroupInterface> gripper_;
        rclcpp::Subscription<String>::SharedPtr color_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trash_pose_sub_;
        rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::PoseStamped Trash_pose_in_panda_link0;
        std::string target_color_;
        std::unordered_map<std::string, size_t> joint_index_map_;
        double gripper_finger_value;
};
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("panda_commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}