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

  // FR3 joint names
  std::vector<std::string> joints_;

  // Robot dynamics model
  std::shared_ptr<RobotModel> model_;

  // CasADi/IPOPT optimizer
  std::shared_ptr<Optimizer> optimizer_;

  // Background solver
  SolverWorker worker_;

  // Validate measured joint positions and velocities
  bool validate_joint_state(
      const RobotState & state) const noexcept;

  // Stop optimization backends
  void stop_backends();
};

}
 // namespace fr3_nmpc_controller

