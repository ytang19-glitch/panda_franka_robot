#include "panda_moveit/collision_distance.hpp"
#include "panda_moveit/cost_functions.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <future>
#include <thread>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <stdexcept>
using namespace std::chrono_literals;

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=rclcpp::Node::make_shared("collision_cost_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]{executor.spin();});
  int result=0;
  try {
    const auto group=node->declare_parameter<std::string>("planning_group","arm");
    const auto base=node->declare_parameter<std::string>("base_frame","panda_link0");
    const auto link=node->declare_parameter<std::string>("probe_link","panda_hand");
    const auto id=node->declare_parameter<std::string>("obstacle_id","collision_cost_probe");
    const auto size=node->declare_parameter<std::vector<double>>("box_size",{0.1,0.1,0.1});
    const auto offsets=node->declare_parameter<std::vector<double>>("offsets",{0.5,0.3,0.2,0.15,0.1,0.05,0.0});
    const auto safe=node->declare_parameter<double>("desired_clearance",0.1);
    const auto weight=node->declare_parameter<double>("collision_weight",100.0);
    const auto csv=node->declare_parameter<std::string>("csv_path","/tmp/panda_collision_cost.csv");
    if(size.size()!=3||offsets.empty()||id.empty()||!std::isfinite(safe)||safe<=0||!std::isfinite(weight)||weight<0)
      throw std::runtime_error("Invalid experiment parameters");
    for(double x:size) if(!std::isfinite(x)||x<=0) throw std::runtime_error("Invalid box size");
    for(double x:offsets) if(!std::isfinite(x)||x<0) throw std::runtime_error("Invalid offset");

    moveit::planning_interface::MoveGroupInterface move_group(node,group);
    const auto model=move_group.getRobotModel();
    if(!model||!model->hasLinkModel(link)||model->getModelFrame()!=base)
      throw std::runtime_error("Probe link or model frame mismatch; set base_frame to model frame (often world)");
    auto client=node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if(!client->wait_for_service(10s)) throw std::runtime_error("No /get_planning_scene service");
    auto request=std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    using C=moveit_msgs::msg::PlanningSceneComponents;
    request->components.components=C::SCENE_SETTINGS|C::ROBOT_STATE|C::ROBOT_STATE_ATTACHED_OBJECTS|
      C::WORLD_OBJECT_NAMES|C::WORLD_OBJECT_GEOMETRY|C::OCTOMAP|C::TRANSFORMS|C::ALLOWED_COLLISION_MATRIX|C::LINK_PADDING_AND_SCALING;
    auto future=client->async_send_request(request);
    if(future.wait_for(10s)!=std::future_status::ready) throw std::runtime_error("Planning scene request timed out");
    planning_scene::PlanningScene scene(model);
    const auto snapshot=future.get()->scene;
    if(snapshot.is_diff) throw std::runtime_error("Expected full scene snapshot");
    scene.setPlanningSceneMsg(snapshot);
    auto current=move_group.getCurrentState(10.0);
    if(!current) throw std::runtime_error("No current joint state");
    // Freeze the scene snapshot's state (including attachments); verify arm state agrees with live state.
    moveit::core::RobotState state(scene.getCurrentState());
    const auto* jmg=model->getJointModelGroup(group);
    if(!jmg) throw std::runtime_error("Unknown planning group");
    for(const auto& name:jmg->getVariableNames()) {
      const double a=state.getVariablePosition(name), b=current->getVariablePosition(name);
      if(!std::isfinite(a)||!std::isfinite(b)||std::abs(a-b)>0.01)
        throw std::runtime_error("Scene state differs from live state; keep simulated arm stationary");
    }
    state.update();
    if(!state.satisfiesBounds()) throw std::runtime_error("Initial state outside bounds");
    if(scene.isStateColliding(state)) throw std::runtime_error("Initial scene already colliding; inspect RViz");
    if(scene.getWorld()->hasObject(id)) throw std::runtime_error("Probe ID already exists");
    const auto center=state.getGlobalLinkTransform(link).translation().eval();
    std::ofstream out(csv);
    if(!out) throw std::runtime_error("Cannot open CSV");
    out<<"offset_m,distance_m,collision,cost,weighted_cost\n"<<std::setprecision(12);
    bool positive=false, zero=false, hit=false;
    for(double offset:offsets){
      moveit_msgs::msg::CollisionObject obj;
      obj.header.frame_id=base; obj.id=id;
      shape_msgs::msg::SolidPrimitive box;
      box.type=shape_msgs::msg::SolidPrimitive::BOX; box.dimensions=size;
      geometry_msgs::msg::Pose pose;
      pose.orientation.w=1.0;
      pose.position.x=center.x()+offset; pose.position.y=center.y(); pose.position.z=center.z();
      obj.primitives.push_back(box); obj.primitive_poses.push_back(pose);
      obj.operation=moveit_msgs::msg::CollisionObject::ADD;
      // Local snapshot only: never modify live move_group scene or command robot.
      if(!scene.processCollisionObjectMsg(obj)) throw std::runtime_error("Cannot add virtual obstacle");
      auto m=panda_moveit::measureClearance(scene,state);
      const double cost=panda_moveit::collisionCost(m.distance,safe);
      positive|=cost>0; zero|=cost==0; hit|=m.colliding;
      out<<offset<<","<<m.distance<<","<<m.colliding<<","<<cost<<","<<weight*cost<<"\n";
      RCLCPP_INFO(node->get_logger(),"offset=%.3f m distance=%.6f m collision=%d cost=%.8f weighted=%.8f",offset,m.distance,static_cast<int>(m.colliding),cost,weight*cost);
      obj.operation=moveit_msgs::msg::CollisionObject::REMOVE;
      if(!scene.processCollisionObjectMsg(obj)) throw std::runtime_error("Cannot remove virtual obstacle");
    }
    out.flush();
    if(!out) throw std::runtime_error("CSV write failed");
    RCLCPP_INFO(node->get_logger(),"CSV: %s",csv.c_str());
    if(!positive||!zero||!hit) {
      RCLCPP_WARN(node->get_logger(),"Incomplete coverage: positive=%d zero=%d collision=%d. Adjust offsets or inspect other world objects.",positive,zero,hit);
      result=2;
    } else RCLCPP_INFO(node->get_logger(),"PASS: sampled positive cost, zero cost and collision cases");
  } catch(const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(),"%s",e.what()); result=1;
  }
  executor.cancel(); spinner.join(); rclcpp::shutdown(); return result;
}
