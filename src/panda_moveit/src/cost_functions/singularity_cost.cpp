#include "panda_moveit/cost_functions.hpp"
#include <Eigen/SVD>
#include <cmath>
#include <algorithm>
#include <stdexcept>
namespace panda_moveit {
double singularityCost(const Eigen::MatrixXd& j, double eps) {
  if (!j.allFinite() || j.rows()==0 || j.cols()==0 || !std::isfinite(eps) || eps<=0) throw std::invalid_argument("Invalid Jacobian");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(j);
  const double s = j.rows()>j.cols()?0.0:svd.singularValues().minCoeff();
  return 1.0/(s*s+eps);
}
double singularityMarginCost(double sigma, double desired) {
  if (!std::isfinite(sigma)||sigma<0||!std::isfinite(desired)||desired<=0)
    throw std::invalid_argument("Invalid singularity margin");
  const double e=std::max(0.0,1.0-sigma/desired);
  return e*e;
}
}
