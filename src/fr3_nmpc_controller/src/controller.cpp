#include "fr3_nmpc_controller/robot_model.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <stdexcept>

namespace fr3_nmpc_controller
{
namespace
{
using Vector = Eigen::Matrix<double, 7, 1>;
using Matrix = Eigen::Matrix<double, 7, 7>;
Eigen::VectorXd vector(const JointVector & a)
{
  // Dynamic vector matches Pinocchio's pre-instantiated numerical API.
  Eigen::VectorXd v(kJoints);
  for (int i=0; i<7; ++i) { v[i]=a[i]; }
  if (!v.allFinite()) { throw std::invalid_argument("Non-finite model input"); }
  return v;
}
JointVector array(const Eigen::VectorXd & v)
{
  if (v.size()!=7 || !v.allFinite()) { throw std::runtime_error("Invalid model result"); }
  JointVector a{};
  for (int i=0; i<7; ++i) { a[i]=v[i]; }
  return a;
}
void check_mass(const Matrix & m)
{
  Eigen::LDLT<Matrix> ldlt(m);
  if (!m.allFinite() || ldlt.info()!=Eigen::Success || ldlt.vectorD().minCoeff()<=1e-10) {
    throw std::runtime_error("Mass matrix is not finite positive definite");
  }
}
// Workspace belongs to this call, not to RobotModel: concurrent readers do not
// share mutable Pinocchio data. RK4 derivative stages reuse this local workspace.
Vector acceleration_impl(
  const pinocchio::Model & model, pinocchio::Data & data,
  const RobotState & x, const JointVector & tau,
  Matrix * aq = nullptr, Matrix * av = nullptr, Matrix * au = nullptr)
{
  const Eigen::VectorXd q = vector(x.q), v = vector(x.dq), u = vector(tau);
  pinocchio::crba(model, data, q);
  const Matrix mass = data.M.selfadjointView<Eigen::Upper>();
  Eigen::LDLT<Matrix> factor(mass);
  if (!mass.allFinite() || factor.info() != Eigen::Success ||
      factor.vectorD().minCoeff() <= 1e-10) {
    throw std::runtime_error("Mass matrix is not finite positive definite");
  }
  const Eigen::VectorXd h = pinocchio::nonLinearEffects(model, data, q, v);
  const Vector a = factor.solve(u - h);
  if (!a.allFinite()) { throw std::runtime_error("Non-finite acceleration"); }
  if (aq && av && au) {
    // At fixed tau: M*da + (d RNEA/dq)*dq + (d RNEA/dv)*dv = dtau.
    const Eigen::VectorXd a_dynamic = a;
    data.dtau_dq.setZero();
    data.dtau_dv.setZero();
    data.M.setZero();
    pinocchio::computeRNEADerivatives(model, data, q, v, a_dynamic);
    *aq = -factor.solve(data.dtau_dq);
    *av = -factor.solve(data.dtau_dv);
    *au = factor.solve(Matrix::Identity());
    if (!aq->allFinite() || !av->allFinite() || !au->allFinite()) {
      throw std::runtime_error("Non-finite acceleration derivatives");
    }
  }
  return a;
}

template<class Derived, std::size_t Size>
void store_matrix(const Eigen::MatrixBase<Derived> & matrix, std::array<double, Size> & out)
{
  if (static_cast<std::size_t>(matrix.size()) != Size || !matrix.allFinite()) {
    throw std::runtime_error("Invalid dynamics derivative matrix");
  }
  for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      out[static_cast<std::size_t>(col * matrix.rows() + row)] = matrix(row, col);
    }
  }
}
RobotState offset(const RobotState & x, const RobotState & dx, double h)
{
  RobotState y;
  for (std::size_t i=0; i<kJoints; ++i) {
    y.q[i]=x.q[i]+h*dx.q[i]; y.dq[i]=x.dq[i]+h*dx.dq[i];
  }
  return y;
}
}
struct RobotModel::Impl { pinocchio::Model model; ModelLimits bounds; };
RobotModel::RobotModel() = default;
RobotModel::~RobotModel() = default;

bool RobotModel::ready() const noexcept { return static_cast<bool>(impl_); }

void RobotModel::load(const std::string & path, const std::array<double,3> & g)
{
  impl_.reset();  // Failed reload cannot leave an obsolete model marked ready.
  double g2=0;
  for (double a:g) { if (!std::isfinite(a)) { throw std::invalid_argument("Invalid gravity"); } g2+=a*a; }
  if (!std::isfinite(g2) || g2<1e-12) { throw std::invalid_argument("Provide nonzero gravity in the URDF base frame"); }
  auto next=std::make_unique<Impl>();
  pinocchio::urdf::buildModel(path, next->model);
  auto & m=next->model;
  if (m.nq!=7 || m.nv!=7 || m.njoints!=8) {
    throw std::invalid_argument("Require fixed-base seven scalar FR3 joints; lock/remove movable fingers first");
  }
  for (int i=0; i<7; ++i) {
    const auto id=static_cast<pinocchio::JointIndex>(i+1);
    if (m.names[id]!="fr3_joint"+std::to_string(i+1) || m.joints[id].nq()!=1 ||
        m.joints[id].nv()!=1 || m.joints[id].idx_q()!=i || m.joints[id].idx_v()!=i) {
      throw std::invalid_argument("Expected scalar joints in exact fr3_joint1..7 order");
    }
    const double lo=m.lowerPositionLimit[i], hi=m.upperPositionLimit[i];
    if (!std::isfinite(lo) || !std::isfinite(hi) || lo>=hi ||
        !std::isfinite(m.velocityLimit[i]) || m.velocityLimit[i]<=0 ||
        !std::isfinite(m.effortLimit[i]) || m.effortLimit[i]<=0) {
      throw std::invalid_argument("URDF must provide finite nonempty position/velocity/effort limits");
    }
    next->bounds.lower[i]=lo; next->bounds.upper[i]=hi;
    next->bounds.velocity[i]=m.velocityLimit[i]; next->bounds.effort[i]=m.effortLimit[i];
    const auto & inertia=m.inertias[id];
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(inertia.inertia().matrix());
    if (!std::isfinite(inertia.mass()) || inertia.mass()<=0 || !inertia.lever().allFinite() ||
        eig.info()!=Eigen::Success || !eig.eigenvalues().allFinite() || eig.eigenvalues().minCoeff()<=0 ||
        eig.eigenvalues()[2]>eig.eigenvalues()[0]+eig.eigenvalues()[1]+1e-8) {
      throw std::invalid_argument("Invalid link mass/COM/inertia; inertial data must be physically plausible");
    }
  }
  m.gravity.linear()=Eigen::Vector3d(g[0],g[1],g[2]);
  pinocchio::Data data(m);
  Eigen::VectorXd q=(m.lowerPositionLimit+m.upperPositionLimit)*0.5;
  pinocchio::crba(m,data,q);
  data.M.triangularView<Eigen::StrictlyLower>()=data.M.transpose().triangularView<Eigen::StrictlyLower>();
  check_mass(data.M);
  impl_=std::move(next);
}
ModelLimits RobotModel::limits() const
{
  if (!ready()) { throw std::logic_error("Load an FR3 URDF first"); }
  return impl_->bounds;
}
MassMatrix RobotModel::mass(const JointVector & q) const
{
  if (!ready()) { throw std::logic_error("Load an FR3 URDF first"); }
  pinocchio::Data d(impl_->model);
  pinocchio::crba(impl_->model,d,vector(q));
  d.M.triangularView<Eigen::StrictlyLower>()=d.M.transpose().triangularView<Eigen::StrictlyLower>();
  check_mass(d.M);
  MassMatrix result{};
  for (int col=0; col<7; ++col) { for (int row=0; row<7; ++row) { result[col*7+row]=d.M(row,col); } }
  return result;
}
JointVector RobotModel::gravity(const JointVector & q) const
{
  if (!ready()) { throw std::logic_error("Load an FR3 URDF first"); }
  pinocchio::Data d(impl_->model);
  return array(pinocchio::computeGeneralizedGravity(impl_->model,d,vector(q)));
}
JointVector RobotModel::nonlinear_effects(const RobotState & x) const
{
  if (!ready()) { throw std::logic_error("Load an FR3 URDF first"); }
  pinocchio::Data d(impl_->model);
  return array(pinocchio::nonLinearEffects(impl_->model,d,vector(x.q),vector(x.dq)));
}
JointVector RobotModel::acceleration(const RobotState & x, const JointVector & tau) const
{
  if (!ready()) { throw std::logic_error("Load an FR3 URDF first"); }
  pinocchio::Data data(impl_->model);
  return array(acceleration_impl(impl_->model, data, x, tau));
}
JointVector RobotModel::inverse_dynamics(const RobotState & x, const JointVector & ddq) const
{
  if (!ready()) { throw std::logic_error("Load an FR3 URDF first"); }
  pinocchio::Data d(impl_->model);
  return array(pinocchio::rnea(impl_->model,d,vector(x.q),vector(x.dq),vector(ddq)));
}
RobotState RobotModel::step(const RobotState & x, const JointVector & tau, double dt) const
{
  if (!std::isfinite(dt) || dt<=0) { throw std::invalid_argument("dt must be positive finite"); }
  auto rhs=[&](const RobotState & y) { return RobotState{y.dq,acceleration(y,tau)}; };
  const auto k1=rhs(x), k2=rhs(offset(x,k1,dt/2)), k3=rhs(offset(x,k2,dt/2)), k4=rhs(offset(x,k3,dt));
  RobotState y=x;
  for (std::size_t i=0; i<kJoints; ++i) {
    y.q[i]+=dt*(k1.q[i]+2*k2.q[i]+2*k3.q[i]+k4.q[i])/6;
    y.dq[i]+=dt*(k1.dq[i]+2*k2.dq[i]+2*k3.dq[i]+k4.dq[i])/6;
  }
  vector(y.q); vector(y.dq);
  return y;
}

DynamicsLinearization RobotModel::linearize_step(
  const RobotState & x, const JointVector & tau, double dt) const
{
  if (!ready()) { throw std::logic_error("Load an FR3 URDF first"); }
  if (!std::isfinite(dt) || dt <= 0) {
    throw std::invalid_argument("dt must be positive finite");
  }
  using StateMatrix = Eigen::Matrix<double, 14, 14>;
  using InputMatrix = Eigen::Matrix<double, 14, 7>;
  struct Stage {
    RobotState rhs;
    StateMatrix dx = StateMatrix::Zero();
    InputMatrix du = InputMatrix::Zero();
  };
  pinocchio::Data data(impl_->model);
  auto evaluate = [&](const RobotState & s) {
    Matrix aq, av, au;
    Stage stage;
    stage.rhs = RobotState{s.dq, array(acceleration_impl(
      impl_->model, data, s, tau, &aq, &av, &au))};
    stage.dx.block<7, 7>(0, 7).setIdentity();
    stage.dx.block<7, 7>(7, 0) = aq;
    stage.dx.block<7, 7>(7, 7) = av;
    stage.du.block<7, 7>(7, 0) = au;
    return stage;
  };
  const StateMatrix identity = StateMatrix::Identity();
  const auto s1 = evaluate(x);
  const StateMatrix k1x = s1.dx;
  const InputMatrix k1u = s1.du;
  const auto s2 = evaluate(offset(x, s1.rhs, dt / 2));
  const StateMatrix k2x = s2.dx * (identity + (dt / 2) * k1x);
  const InputMatrix k2u = s2.dx * ((dt / 2) * k1u) + s2.du;
  const auto s3 = evaluate(offset(x, s2.rhs, dt / 2));
  const StateMatrix k3x = s3.dx * (identity + (dt / 2) * k2x);
  const InputMatrix k3u = s3.dx * ((dt / 2) * k2u) + s3.du;
  const auto s4 = evaluate(offset(x, s3.rhs, dt));
  const StateMatrix k4x = s4.dx * (identity + dt * k3x);
  const InputMatrix k4u = s4.dx * (dt * k3u) + s4.du;

  DynamicsLinearization out;
  out.acceleration = s1.rhs.dq;
  out.next_state = x;
  for (std::size_t i = 0; i < kJoints; ++i) {
    out.next_state.q[i] += dt * (s1.rhs.q[i] + 2 * s2.rhs.q[i] +
      2 * s3.rhs.q[i] + s4.rhs.q[i]) / 6;
    out.next_state.dq[i] += dt * (s1.rhs.dq[i] + 2 * s2.rhs.dq[i] +
      2 * s3.rhs.dq[i] + s4.rhs.dq[i]) / 6;
  }
  vector(out.next_state.q); vector(out.next_state.dq);
  const StateMatrix step_dx = identity + (dt / 6) * (k1x + 2 * k2x + 2 * k3x + k4x);
  const InputMatrix step_du = (dt / 6) * (k1u + 2 * k2u + 2 * k3u + k4u);
  store_matrix(step_dx, out.step_dx);
  store_matrix(step_du, out.step_du);
  store_matrix(s1.dx.bottomRows<7>(), out.acceleration_dx);
  store_matrix(s1.du.bottomRows<7>(), out.acceleration_du);
  return out;
}
}  // namespace fr3_nmpc_controller
