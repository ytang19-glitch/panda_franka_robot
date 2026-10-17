#include "fr3_nmpc_controller/controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <exception>

namespace fr3_nmpc_controller
{
controller_interface::CallbackReturn Fr3NmpcController::on_init()
{
  try {
    std::vector<std::string> defaults;
    for (std::size_t i = 1; i <= kJoints; ++i) {
      defaults.push_back("fr3_joint" + std::to_string(i));
    }
    auto_declare<std::vector<std::string>>("joints", defaults);
  } catch (const std::exception &) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Fr3NmpcController::on_configure(const rclcpp_lifecycle::State &)
{
  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.size() != kJoints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected seven FR3 joints");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < kJoints; ++i) {
    if (joints_[i] != "fr3_joint" + std::to_string(i + 1)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint order must be fr3_joint1 through fr3_joint7");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_WARN(get_node()->get_logger(), "Development scaffold configured; activation is disabled");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration Fr3NmpcController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) { result.names.push_back(joint + "/effort"); }
  return result;
}

controller_interface::InterfaceConfiguration Fr3NmpcController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    result.names.push_back(joint + "/position");
    result.names.push_back(joint + "/velocity");
  }
  return result;
}

controller_interface::CallbackReturn Fr3NmpcController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_node()->get_logger(),
    "Activation disabled: model=%s optimizer=%s worker=%s. No command path implemented.",
    model_.ready() ? "ready" : "missing", optimizer_.ready() ? "ready" : "missing",
    worker_.ready() ? "ready" : "missing");
  // Unconditional refusal, not a parameter that can accidentally enable motion.
  return controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn Fr3NmpcController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Fr3NmpcController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  // No writes, no zero-torque fallback and no blocking solver calls.
  return controller_interface::return_type::ERROR;
}
}  // namespace fr3_nmpc_controller

PLUGINLIB_EXPORT_CLASS(fr3_nmpc_controller::Fr3NmpcController, controller_interface::ControllerInterface)
