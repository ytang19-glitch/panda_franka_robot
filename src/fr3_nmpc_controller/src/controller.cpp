#include "fr3_nmpc_controller/controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>

namespace fr3_nmpc_controller
{
namespace
{
JointVector seven(const std::vector<double> & v,const char * name)
{
  if (v.size()!=kJoints || !std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);})) {
    throw std::invalid_argument(std::string(name)+" needs seven finite values");
  }
  JointVector a{}; std::copy(v.begin(),v.end(),a.begin()); return a;
}
}
controller_interface::CallbackReturn Fr3NmpcController::on_init()
{
  try {
    std::vector<std::string> defaults;
    for (std::size_t i=1;i<=kJoints;++i) { defaults.push_back("fr3_joint"+std::to_string(i)); }
    auto_declare<std::vector<std::string>>("joints",defaults);
    auto_declare<bool>("backend_preflight",false); // Configure only; never enables motion.
    auto_declare<std::string>("model_urdf_path","");
    auto_declare<std::vector<double>>("gravity_base",{0.0,0.0,-9.81});
    auto_declare<int>("horizon",10);
    auto_declare<double>("prediction_dt",0.05);
    auto_declare<double>("max_state_age_s",0.1);
    auto_declare<double>("max_cpu_time_s",0.05);
    auto_declare<double>("max_wall_time_s",0.05);
    for (const auto & name:{"total_torque_limits","total_torque_rate_limits","acceleration_limits"}) {
      auto_declare<std::vector<double>>(name,std::vector<double>(kJoints,0.0));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(),"Initialization failed: %s",e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}
void Fr3NmpcController::stop_backends()
{
  worker_.stop(); optimizer_.reset(); model_.reset();
}
controller_interface::CallbackReturn Fr3NmpcController::on_configure(const rclcpp_lifecycle::State &)
{
  stop_backends();
  try {
    const auto node=get_node();
    joints_=node->get_parameter("joints").as_string_array();
    if (joints_.size()!=kJoints) { throw std::invalid_argument("Expected seven FR3 joints"); }
    for (std::size_t i=0;i<kJoints;++i) {
      if (joints_[i]!="fr3_joint"+std::to_string(i+1)) {
        throw std::invalid_argument("Joint order must be fr3_joint1 through fr3_joint7");
      }
    }
    if (!node->get_parameter("backend_preflight").as_bool()) {
      RCLCPP_WARN(node->get_logger(),"Scaffold configured; backend preflight off; activation disabled");
      return controller_interface::CallbackReturn::SUCCESS;
    }
    const auto path=node->get_parameter("model_urdf_path").as_string();
    if (path.empty()) { throw std::invalid_argument("Provide expanded seven-joint model_urdf_path"); }
    const auto g=node->get_parameter("gravity_base").as_double_array();
    if (g.size()!=3) { throw std::invalid_argument("gravity_base needs three values"); }
    const auto n=node->get_parameter("horizon").as_int();
    if (n<2 || n>100) { throw std::invalid_argument("horizon must be 2..100"); }
    const double age=node->get_parameter("max_state_age_s").as_double();
    if (!std::isfinite(age) || age<=0 || age>60) {
      throw std::invalid_argument("max_state_age_s must be positive finite and <=60");
    }
    OptimizerConfig c;
    c.horizon=static_cast<std::size_t>(n);
    c.dt=node->get_parameter("prediction_dt").as_double();
    c.max_cpu_time_s=node->get_parameter("max_cpu_time_s").as_double();
    c.max_wall_time_s=node->get_parameter("max_wall_time_s").as_double();
    if (!std::isfinite(c.max_wall_time_s) || c.max_wall_time_s<=0 || c.max_wall_time_s>=age) {
      throw std::invalid_argument("Require 0 < max_wall_time_s < max_state_age_s");
    }
    c.torque_limit=seven(node->get_parameter("total_torque_limits").as_double_array(),"total_torque_limits");
    c.torque_rate_limit=seven(node->get_parameter("total_torque_rate_limits").as_double_array(),"total_torque_rate_limits");
    c.acceleration_limit=seven(node->get_parameter("acceleration_limits").as_double_array(),"acceleration_limits");
    auto model=std::make_shared<RobotModel>();
    model->load(path,{g[0],g[1],g[2]});
    auto optimizer=std::make_shared<Optimizer>();
    optimizer->configure(model,c);
    worker_.start(optimizer,c.horizon,static_cast<std::int64_t>(age*1e9));
    model_=std::move(model); optimizer_=std::move(optimizer);
    RCLCPP_INFO(node->get_logger(),
      "Preflight initialized model, optimizer and worker. No solve submitted; activation disabled.");
    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    stop_backends();
    RCLCPP_ERROR(get_node()->get_logger(),"Backend configuration failed: %s",e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}
controller_interface::InterfaceConfiguration Fr3NmpcController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration out;
  out.type=controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint:joints_) { out.names.push_back(joint+"/effort"); }
  return out;
}
controller_interface::InterfaceConfiguration Fr3NmpcController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration out;
  out.type=controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint:joints_) {
    out.names.push_back(joint+"/position"); out.names.push_back(joint+"/velocity");
  }
  return out;
}
controller_interface::CallbackReturn Fr3NmpcController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_node()->get_logger(),
    "Activation disabled: model=%s optimizer=%s worker=%s. Preflight is not hardware control.",
    model_ && model_->ready()?"initialized":"missing",
    optimizer_ && optimizer_->ready()?"initialized":"missing",worker_.ready()?"initialized":"missing");
  return controller_interface::CallbackReturn::ERROR;
}
controller_interface::CallbackReturn Fr3NmpcController::on_deactivate(const rclcpp_lifecycle::State &)
{
  worker_.cancel_reference();
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn Fr3NmpcController::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_backends(); return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn Fr3NmpcController::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_backends(); return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::return_type Fr3NmpcController::update(const rclcpp::Time &,const rclcpp::Duration &)
{
  // No mutexes, allocations, solver calls or effort writes in the disabled path.
  return controller_interface::return_type::ERROR;
}
}
PLUGINLIB_EXPORT_CLASS(fr3_nmpc_controller::Fr3NmpcController,controller_interface::ControllerInterface)

