// Offline diagnostic: callback Jacobian versus fixed-step central differences.
#include "fr3_nmpc_controller/robot_model.hpp"
#include <casadi/casadi.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace fr3_nmpc_controller
{
namespace
{
// DM stores numbers; MX builds symbolic expressions.
using casadi::DM; using casadi::MX; using casadi::Slice;
DM dm(const JointVector & a) { return DM(std::vector<double>(a.begin(),a.end())); }
DM dm(const RobotState & x) { return DM::vertcat({dm(x.q),dm(x.dq)}); }
bool finite(const RobotState & x)
{
  for (std::size_t i=0;i<7;++i) { if (!std::isfinite(x.q[i]) || !std::isfinite(x.dq[i])) { return false; } }
  return true;
}
// Convert a dense [q; dq] vector to robot state.
RobotState state(const DM & x)
{
  const auto v=x.nonzeros();
  if (v.size()!=14) { throw std::runtime_error("Expected dense fourteen-state result"); }
  RobotState out;
  for (std::size_t i=0;i<7;++i) { out.q[i]=v[i]; out.dq[i]=v[i+7]; }
  if (!finite(out)) { throw std::runtime_error("Non-finite prediction"); }
  return out;
}
// Numerical dynamics: finite-difference derivatives, not symbolic Pinocchio AD.
class DynamicsCallback final : public casadi::Callback
{
public:
  DynamicsCallback(std::shared_ptr<const RobotModel> model,double dt):model_(std::move(model)),dt_(dt)
  { construct("fr3_numeric_dynamics", {{"enable_fd",true}}); }
  casadi_int get_n_in() override { return 2; }
  casadi_int get_n_out() override { return 2; }
  casadi::Sparsity get_sparsity_in(casadi_int i) override { return casadi::Sparsity::dense(i==0?14:7,1); }
  casadi::Sparsity get_sparsity_out(casadi_int i) override { return casadi::Sparsity::dense(i==0?14:7,1); }
  std::vector<DM> eval(const std::vector<DM> & args) const override
  {
    const auto x=state(args.at(0));
    const auto values=args.at(1).nonzeros();
    if (values.size()!=kJoints) { throw std::invalid_argument("Expected seven dense torque values"); }
    JointVector u{};
    std::copy(values.begin(),values.end(),u.begin());
    // Total torque includes gravity; outputs are RK4 state and node acceleration.
    return {dm(model_->step(x,u,dt_)),dm(model_->acceleration(x,u))};
  }
private:
  std::shared_ptr<const RobotModel> model_;
  double dt_;
};
}

void check_callback(const std::shared_ptr<const RobotModel> & model)
{
  // Captured step-1 feedback and previous total torque from the hold run.
  const RobotState x{
    {4.5732437711428768e-08, 6.7040546093216323e-07,
     4.3558329176024929e-08, -1.5969503488527537,
     -5.6713566731482808e-09, 2.5306999510682324,
     -4.189427634327483e-10},
    {4.6268561801667744e-06, 6.7107117250311111e-05,
     4.3026485870451378e-06, -3.4811142101824898e-05,
     -5.5049790105157051e-07, -4.9150898003791836e-06,
     -1.2035800568588659e-07}};
  const JointVector u{
    0.00062283854548995043, -30.348938716813269,
    0.00060773657780384144, 22.811627121084445,
    0.74141530936021283, 2.621779403349568,
    -0.086257787015465603};

  DynamicsCallback callback(model, 0.02);
  const MX z = MX::sym("z", 21);
  const auto outputs = callback(std::vector<MX>{z(Slice(0,14)), z(Slice(14,21))});
  const MX y = MX::vertcat({outputs[0], outputs[1]});
  const casadi::Function jacobian(
    "callback_jacobian", std::vector<MX>{z},
    std::vector<MX>{MX::jacobian(y, z)});

  const DM point = DM::vertcat({dm(x), dm(u)});
  const DM supplied = DM::densify(jacobian(std::vector<DM>{point}).at(0));
  if (!supplied.is_regular()) { throw std::runtime_error("Non-finite callback Jacobian"); }

  // Direct numerical evaluations bypass the callback's derivative machinery.
  auto evaluate = [&](const DM & input) {
    const auto out = callback.eval(std::vector<DM>{
      DM(input(Slice(0,14))), DM(input(Slice(14,21)))});
    return DM::densify(DM::vertcat({out[0], out[1]}));
  };
  const auto baseline = evaluate(point);
  const auto repeated = evaluate(point);
  double repeat_error = 0.0;
  for (int row=0; row<21; ++row) {
    repeat_error = std::max(repeat_error,
      std::abs(static_cast<double>(baseline(row))-static_cast<double>(repeated(row))));
  }
  std::cout << "[REPEAT] max_abs=" << repeat_error << '\n';

  const std::array<const char *,3> input_names{{"q", "dq", "tau"}};
  const std::array<const char *,3> output_names{{"q_next", "dq_next", "ddq"}};
  DM last;
  bool have_last=false;
  for (double scale : {1e-4, 1e-5, 1e-6, 1e-7}) {
    DM central = DM::zeros(21,21);
    for (int col=0; col<21; ++col) {
      const double value=static_cast<double>(point(col));
      const double h=scale*std::max(1.0,std::abs(value));
      DM plus=point, minus=point;
      plus(col)=value+h; minus(col)=value-h;
      central(Slice(),col)=(evaluate(plus)-evaluate(minus))/(2.0*h);
    }
    if (!central.is_regular()) { throw std::runtime_error("Non-finite central Jacobian"); }
    for (int output=0; output<3; ++output) {
      for (int input=0; input<3; ++input) {
        double error=0.0, magnitude=0.0, change=0.0;
        for (int row=output*7; row<(output+1)*7; ++row) {
          for (int col=input*7; col<(input+1)*7; ++col) {
            const double actual=static_cast<double>(central(row,col));
            error=std::max(error,std::abs(actual-static_cast<double>(supplied(row,col))));
            magnitude=std::max(magnitude,std::abs(actual));
            if (have_last) {
              change=std::max(change,std::abs(actual-static_cast<double>(last(row,col))));
            }
          }
        }
        std::cout << "[JAC] h_scale=" << scale
                  << " output=" << output_names[output]
                  << " input=" << input_names[input]
                  << " max_abs_diff=" << error
                  << " fd_max_abs=" << magnitude;
        if (have_last) { std::cout << " fd_change=" << change; }
        std::cout << '\n';
      }
    }
    last=central; have_last=true;
  }
  std::cout << "Diagnostic completed; no convergence or hardware validation implied.\n";
}
}  // namespace fr3_nmpc_controller

int main(int argc, char ** argv)
{
  if (argc!=2) {
    std::cerr << "Usage: check_callback_jacobian MODEL.urdf\n";
    return 1;
  }
  try {
    std::cout << std::scientific << std::setprecision(8);
    std::cout << "OFFLINE CALLBACK CHECK: no IPOPT solve or robot commands\n";
    auto model=std::make_shared<fr3_nmpc_controller::RobotModel>();
    model->load(argv[1]);
    fr3_nmpc_controller::check_callback(model);
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Diagnostic failed: " << e.what() << '\n';
    return 1;
  }
}
