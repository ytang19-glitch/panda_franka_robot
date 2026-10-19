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
// Backend-preflight plugin. Activation always fails; no effort writes.
class Fr3NmpcController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<std::string> joints_;
  // Worker owns shared optimizer; destroy/stop worker before resetting backends.
  std::shared_ptr<RobotModel> model_;
  std::shared_ptr<Optimizer> optimizer_;
  SolverWorker worker_;
  void stop_backends(); // Non-real-time lifecycle only; may wait for solver.
};
}  // namespace fr3_nmpc_controller

