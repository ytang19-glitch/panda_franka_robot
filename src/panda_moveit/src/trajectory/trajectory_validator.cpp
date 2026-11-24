#include "panda_moveit/reference_tracker.hpp"
#include <rclcpp/duration.hpp>
#include <cmath>
#include <stdexcept>
namespace panda_moveit {
void validateTiming(const moveit_msgs::msg::RobotTrajectory& t,const std::string& label){
 const auto& p=t.joint_trajectory.points;
 const auto n=t.joint_trajectory.joint_names.size();
 if(n==0||p.size()<2) throw std::runtime_error(label+": empty trajectory");
 int64_t previous=-1;
 for(const auto& x:p){
   const auto ns=rclcpp::Duration(x.time_from_start).nanoseconds();
   if(ns<0||ns<=previous||x.positions.size()!=n) throw std::runtime_error(label+": invalid timing/dimensions");
   for(const auto& v:{x.positions,x.velocities,x.accelerations}) {
     if(!v.empty()&&v.size()!=n) throw std::runtime_error(label+": invalid vector size");
     for(double y:v) if(!std::isfinite(y)) throw std::runtime_error(label+": nonfinite value");
   }
   previous=ns;
 }
}
}
