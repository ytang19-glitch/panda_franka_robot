#pragma once
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "fr3_nmpc_controller/robot_model.hpp"
#include "fr3_nmpc_controller/optimizer.hpp"
#include "fr3_nmpc_controller/solver_worker.hpp"

namespace fr3_nmpc_controller
{
// Loadable development scaffold. Activation always fails; no effort writes.
class Fr3NmpcController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<std::string> joints_;
  RobotModel model_;
  Optimizer optimizer_;
  SolverWorker worker_;
};
}  // namespace fr3_nmpc_controller
