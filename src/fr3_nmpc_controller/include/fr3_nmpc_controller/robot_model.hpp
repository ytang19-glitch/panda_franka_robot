#pragma once
#include <array>
#include <cstddef>
#include <memory>
#include <string>

namespace fr3_nmpc_controller
{
constexpr std::size_t kJoints = 7;
using JointVector = std::array<double, kJoints>;
using MassMatrix = std::array<double, kJoints * kJoints>;  // Column-major.
struct RobotState { JointVector q{}; JointVector dq{}; };
struct ModelLimits
{
  JointVector lower{}, upper{}, velocity{}, effort{};
};
// Numerical offline backend. Methods allocate workspace: not a hard-RT API.
// Load only before solving; do not call load() concurrently with model users.
class RobotModel
{
public:
  RobotModel();
  ~RobotModel();
  RobotModel(const RobotModel &) = delete;
  RobotModel & operator=(const RobotModel &) = delete;
  void load(const std::string & urdf_path,
    const std::array<double, 3> & gravity_base = {0.0, 0.0, -9.81});
  bool ready() const noexcept;
  ModelLimits limits() const;
  MassMatrix mass(const JointVector & q) const;
  JointVector gravity(const JointVector & q) const;
  JointVector nonlinear_effects(const RobotState & state) const;
  JointVector acceleration(const RobotState & state, const JointVector & total_torque) const;
  RobotState step(const RobotState & state, const JointVector & total_torque, double dt) const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace fr3_nmpc_controller
