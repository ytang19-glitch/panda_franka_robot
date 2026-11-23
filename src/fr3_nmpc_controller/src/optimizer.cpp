// Offline torque NMPC: x=[q; dq], u=total torque. No robot commands.
#include "fr3_nmpc_controller/optimizer.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#ifdef FR3_NMPC_WITH_CASADI
#include <casadi/casadi.hpp>
#endif

namespace fr3_nmpc_controller
{
#ifdef FR3_NMPC_WITH_CASADI
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
template<std::size_t Size>
DM dense_matrix(const std::array<double, Size> & values, casadi_int rows, casadi_int cols)
{
  return DM(casadi::Sparsity::dense(rows, cols),
    std::vector<double>(values.begin(), values.end()));
}

// Inputs: x, u, nominal next_state, nominal acceleration.
// Outputs (CasADi output-major order): dnext/dx, dnext/du, da/dx, da/du.
class DynamicsJacobianCallback final : public casadi::Callback
{
public:
  DynamicsJacobianCallback(
    const std::string & name, std::shared_ptr<const RobotModel> model, double dt)
  : model_(std::move(model)), dt_(dt)
  {
    construct(name, {{"enable_fd", false}});
  }
  casadi_int get_n_in() override { return 4; }
  casadi_int get_n_out() override { return 4; }
  casadi::Sparsity get_sparsity_in(casadi_int i) override
  {
    // Nominal outputs are not needed by this implementation.
    if (i >= 2) { return casadi::Sparsity(i == 2 ? 14 : 7, 1); }
    return casadi::Sparsity::dense(i == 0 ? 14 : 7, 1);
  }
  casadi::Sparsity get_sparsity_out(casadi_int i) override
  {
    return casadi::Sparsity::dense(i < 2 ? 14 : 7, i % 2 == 0 ? 14 : 7);
  }
  std::vector<DM> eval(const std::vector<DM> & args) const override
  {
    const auto x = state(args.at(0));
    const auto values = args.at(1).nonzeros();
    if (values.size() != kJoints) { throw std::invalid_argument("Expected seven torque values"); }
    JointVector u{};
    std::copy(values.begin(), values.end(), u.begin());
    const auto d = model_->linearize_step(x, u, dt_);
    return {dense_matrix(d.step_dx, 14, 14), dense_matrix(d.step_du, 14, 7),
      dense_matrix(d.acceleration_dx, 7, 14), dense_matrix(d.acceleration_du, 7, 7)};
  }
private:
  std::shared_ptr<const RobotModel> model_;
  double dt_;
};

// Numerical dynamics with explicit analytic first derivatives.
class DynamicsCallback final : public casadi::Callback
{
public:
  DynamicsCallback(std::shared_ptr<const RobotModel> model,double dt):model_(std::move(model)),dt_(dt)
  { construct("fr3_analytic_dynamics", {{"enable_fd",false}}); }
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
  bool has_jacobian() const override { return true; }
  casadi::Function get_jacobian(
    const std::string & name, const std::vector<std::string> & inames,
    const std::vector<std::string> & onames, const casadi::Dict & opts) const override
  {
    (void)inames; (void)onames; (void)opts;
    // CasADi Functions do not own Callback C++ objects. Retain all instances
    // until the graph has been destroyed (see Impl member declaration order).
    auto jac = std::make_unique<DynamicsJacobianCallback>(name, model_, dt_);
    casadi::Function function = *jac;
    jacobians_.push_back(std::move(jac));
    return function;
  }
private:
  std::shared_ptr<const RobotModel> model_;
  double dt_;
  mutable std::vector<std::unique_ptr<DynamicsJacobianCallback>> jacobians_;
};
}
// One thread owns the optimizer. Destroy the graph before its callback.
struct Optimizer::Impl
{
  std::shared_ptr<const RobotModel> model;
  OptimizerConfig cfg;
  ModelLimits limits;
  std::unique_ptr<DynamicsCallback> callback;
  casadi::Opti opt;
  // States: 14 x (N+1); torques: 7 x N; remaining symbols are parameters.
  MX X,U,x0,ref,last_u;
  DM warm_x,warm_u;
  bool warm{false};
};
#else
struct Optimizer::Impl {};
#endif
Optimizer::Optimizer() = default;
Optimizer::~Optimizer() = default;
bool Optimizer::ready() const noexcept { return static_cast<bool>(impl_); }
const std::string & Optimizer::last_error() const noexcept { return last_error_; }
// Clear warm-start eligibility without rebuilding the problem.
void Optimizer::reset()
{
#ifdef FR3_NMPC_WITH_CASADI
  if (impl_) { impl_->warm=false; }
#endif
  last_error_.clear();
}
// Validate configuration and build the NLP once.
void Optimizer::configure(std::shared_ptr<const RobotModel> model,const OptimizerConfig & c)
{
  impl_.reset();
  last_error_.clear();
#ifndef FR3_NMPC_WITH_CASADI
  (void)model; (void)c;
  throw std::logic_error("Rebuild with FR3_NMPC_WITH_CASADI=ON and compatible CasADi/IPOPT");
#else
  if (!model || !model->ready()) { throw std::invalid_argument("Load robot model before optimizer"); }
  if (c.horizon<2 || c.horizon>100 || !std::isfinite(c.dt) || c.dt<=0 || c.dt>1 ||
      c.max_iterations<=0 || !std::isfinite(c.max_cpu_time_s) || c.max_cpu_time_s<=0 ||
      !std::isfinite(c.max_wall_time_s) || c.max_wall_time_s<=0 ||
      !std::isfinite(c.feasibility_tolerance) || c.feasibility_tolerance<=0) {
    throw std::invalid_argument("Invalid horizon, dt, iteration budget or feasibility tolerance");
  }
  for (double w:{c.q_weight,c.dq_weight,c.effort_weight,c.delta_weight,c.terminal_weight}) {
    if (!std::isfinite(w) || w<=0) { throw std::invalid_argument("Weights must be positive finite"); }
  }
  // Total-torque bounds must remain within URDF effort limits.
  const auto lim=model->limits();
  for (std::size_t i=0;i<7;++i) {
    if (!std::isfinite(c.torque_limit[i]) || c.torque_limit[i]<=0 || c.torque_limit[i]>lim.effort[i] ||
        !std::isfinite(c.torque_rate_limit[i]) || c.torque_rate_limit[i]<=0 ||
        !std::isfinite(c.acceleration_limit[i]) || c.acceleration_limit[i]<=0) {
      throw std::invalid_argument("Explicit positive limits required; total torque must not exceed URDF effort limit");
    }
  }
  auto p=std::make_unique<Impl>();
  p->model=std::move(model); p->cfg=c; p->limits=lim;
  p->callback=std::make_unique<DynamicsCallback>(p->model,c.dt);
  auto & o=p->opt;
  const int N=static_cast<int>(c.horizon);
  // Multiple shooting: optimize states and torques with dynamics equalities.
  p->X=o.variable(14,N+1); p->U=o.variable(7,N);
  o.set_linear_scale(p->U, DM::repmat(dm(c.torque_limit), 1, N));
  p->x0=o.parameter(14); p->ref=o.parameter(14,N+1); p->last_u=o.parameter(7);
  o.subject_to(p->X(Slice(),0)==p->x0);
  MX cost=0;
  // Node limits do not guarantee continuous-time or collision safety.
  for (int k=0;k<=N;++k) {
    o.subject_to(casadi::Opti::bounded(dm(lim.lower),p->X(Slice(0,7),k),dm(lim.upper)));
    o.subject_to(casadi::Opti::bounded(-dm(lim.velocity),p->X(Slice(7,14),k),dm(lim.velocity)));
  }
  for (int k=0;k<N;++k) {
    const MX u=p->U(Slice(),k);
    const auto prediction=(*p->callback)(std::vector<MX>{p->X(Slice(),k),u});
    // Connect successive shooting nodes using RK4 dynamics.
    o.subject_to(p->X(Slice(),k+1)==prediction[0]);
    o.subject_to(casadi::Opti::bounded(-dm(c.acceleration_limit),prediction[1],dm(c.acceleration_limit)));
    o.subject_to(casadi::Opti::bounded(-dm(c.torque_limit),u,dm(c.torque_limit)));
    // First torque change uses the caller's previous total torque.
    const MX du=u-(k==0?p->last_u:p->U(Slice(),k-1));
    o.subject_to(casadi::Opti::bounded(-c.dt*dm(c.torque_rate_limit),du,c.dt*dm(c.torque_rate_limit)));
    // Track the next state against the reference at the same time.
    const MX error=p->X(Slice(),k+1)-p->ref(Slice(),k+1);
    cost+=c.q_weight*MX::sumsqr(error(Slice(0,7)))+c.dq_weight*MX::sumsqr(error(Slice(7,14)));
    // Penalize normalized total torque, including gravity-hold effort.
    cost+=c.effort_weight*MX::sumsqr(u/dm(c.torque_limit));
    // Smooth torque changes; this is not a direct jerk constraint.
    cost+=c.delta_weight*MX::sumsqr(du/dm(c.torque_limit));
  }
  // Terminal position penalty.
  cost+=c.terminal_weight*MX::sumsqr(p->X(Slice(0,7),N)-p->ref(Slice(0,7),N));
  o.minimize(cost);
 
  o.solver(
    "ipopt",
    {{"print_time", false}},
    {
      {"print_level", 0},
      {"sb", "yes"},
      {"print_user_options", "no"},

      // Existing iteration and CPU-time budgets.
      {"max_iter", c.max_iterations},
      {"max_cpu_time", c.max_cpu_time_s},

      // Existing convergence settings.
      {"tol", 1e-6},
      {"constr_viol_tol", 1e-7},
      {"hessian_approximation", "limited-memory"},

      // Analytic first derivatives; retain limited-memory Hessian approximation.
      {"derivative_test", "none"},
      {"print_timing_statistics", "no"},
    });

  impl_ = std::move(p);

#endif
}
std::optional<SolverResult> Optimizer::solve(const RobotState & x,const RobotState & ref)
{
  if (!ready()) { last_error_="Optimizer is not configured"; return std::nullopt; }
#ifdef FR3_NMPC_WITH_CASADI
  try {
    // Constant reference; assume previous torque equals model gravity.
    return solve(x,std::vector<RobotState>(impl_->cfg.horizon+1,ref),impl_->model->gravity(x.q));
  } catch (const std::exception & e) {
    impl_->warm=false; last_error_=e.what(); return std::nullopt;
  }
#else
  (void)x; (void)ref;
  return std::nullopt;
#endif
}
std::optional<SolverResult> Optimizer::solve(const RobotState & x,
  const std::vector<RobotState> & refs,const JointVector & previous)
{
#ifndef FR3_NMPC_WITH_CASADI
  (void)x; (void)refs; (void)previous;
  last_error_="CasADi backend was not enabled at build time";
  return std::nullopt;
#else
  if (!ready()) { last_error_="Optimizer is not configured"; return std::nullopt; }
  auto & p=*impl_;
  try {
    if (!finite(x) || refs.size()!=p.cfg.horizon+1) { throw std::invalid_argument("Expected finite state and N+1 references"); }
    // Reject invalid feedback and references before solving.
    auto check=[&](const RobotState & s) {
      if (!finite(s)) { throw std::invalid_argument("Non-finite reference/state"); }
      for (std::size_t i=0;i<7;++i) {
        if (s.q[i]<p.limits.lower[i] || s.q[i]>p.limits.upper[i] || std::abs(s.dq[i])>p.limits.velocity[i]) {
          throw std::invalid_argument("Reference/state outside URDF position or velocity limits");
        }
      }
    };
    check(x);
    for (const auto & r:refs) { check(r); }
    for (std::size_t i=0;i<7;++i) {
      if (!std::isfinite(previous[i]) || std::abs(previous[i])>p.cfg.torque_limit[i]) {
        throw std::invalid_argument("Previous total torque outside bounds");
      }
    }
    DM r=DM::zeros(14,static_cast<int>(refs.size()));
    for (std::size_t k=0;k<refs.size();++k) { r(Slice(),static_cast<int>(k))=dm(refs[k]); }
    p.opt.set_value(p.x0,dm(x)); p.opt.set_value(p.ref,r); p.opt.set_value(p.last_u,dm(previous));
    // Shifted primal warm start; cold start repeats state and previous torque.
    if (p.warm) {
      p.opt.set_initial(p.X,p.warm_x); p.opt.set_initial(p.U,p.warm_u);
    } else {
      p.opt.set_initial(p.X,DM::repmat(dm(x),1,static_cast<int>(refs.size())));
      p.opt.set_initial(p.U,DM::repmat(dm(previous),1,static_cast<int>(p.cfg.horizon)));
    }
    // Run IPOPT and measure elapsed wall time.
    const auto started=std::chrono::steady_clock::now();
    auto sol=p.opt.solve(); // Failure is caught; debug iterates are never commands.
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    // Post-return acceptance check; cannot interrupt a running solve.
    if (!std::isfinite(ms) || ms>p.cfg.max_wall_time_s*1000.0) {
      throw std::runtime_error("Solution exceeded wall-time acceptance budget");
    }
    const auto stats=p.opt.stats();
    if (!stats.at("success").to_bool()) { throw std::runtime_error("Solver did not report success"); }
    const DM xx=sol.value(p.X), uu=sol.value(p.U);
    const auto gv=sol.value(p.opt.g()).nonzeros();
    const auto lb=sol.value(p.opt.lbg()).nonzeros(), ub=sol.value(p.opt.ubg()).nonzeros();
    if (gv.size()!=lb.size() || gv.size()!=ub.size()) {
      throw std::runtime_error("Constraint bound dimensions do not match");
    }
    // Independently validate all constraints; residual units are mixed.
    double max_violation=0.0;
    for (std::size_t i=0;i<gv.size();++i) {
      if (std::isnan(lb[i]) || std::isnan(ub[i]) || lb[i]>ub[i]) {
        throw std::runtime_error("Invalid evaluated constraint bounds");
      }
      max_violation=std::max(max_violation,std::max(lb[i]-gv[i],gv[i]-ub[i]));
      if (!std::isfinite(gv[i]) || gv[i]<lb[i]-p.cfg.feasibility_tolerance || gv[i]>ub[i]+p.cfg.feasibility_tolerance) {
        throw std::runtime_error("Post-solve feasibility check failed");
      }
    }
    SolverResult result;
    result.converged=true; result.solve_time_ms=ms;
    result.max_constraint_violation=max_violation;
    for (std::size_t k=0;k<refs.size();++k) { result.prediction.push_back(state(xx(Slice(),static_cast<int>(k)))); }
    for (std::size_t k=0;k<p.cfg.horizon;++k) {
      const auto v=DM(uu(Slice(),static_cast<int>(k))).nonzeros();
      if (v.size()!=kJoints) { throw std::runtime_error("Invalid torque result dimensions"); }
      JointVector t{};
      for (std::size_t i=0;i<7;++i) {
        if (!std::isfinite(v[i])) { throw std::runtime_error("Non-finite optimized torque"); }
        t[i]=v[i];
      }
      result.torque_sequence.push_back(t);
    }
    // Return only the first input for the caller's receding-horizon loop.
    result.total_torque=result.torque_sequence.front();
    // Shift guesses; repeated tail values are not a new dynamics rollout.
    p.warm_x=DM::horzcat({xx(Slice(),Slice(1,static_cast<int>(refs.size()))),xx(Slice(),static_cast<int>(refs.size()-1))});
    p.warm_u=DM::horzcat({uu(Slice(),Slice(1,static_cast<int>(p.cfg.horizon))),uu(Slice(),static_cast<int>(p.cfg.horizon-1))});
    p.warm=true; last_error_.clear();
    return result; // Valid-until remains zero: offline candidate only.
  } catch (const std::exception & e) {
    // Reject failures and clear the warm start.
    p.warm=false; last_error_=e.what(); return std::nullopt;
  }
#endif
}
}  // namespace fr3_nmpc_controller