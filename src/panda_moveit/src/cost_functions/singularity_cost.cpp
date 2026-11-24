#include "panda_moveit/cost_functions.hpp"
#include <Eigen/SVD>
#include <cmath>
#include <stdexcept>
namespace panda_moveit {
double singularityCost(const Eigen::MatrixXd& j, double eps) {
  if (!j.allFinite() || j.rows()==0 || j.cols()==0 || !std::isfinite(eps) || eps<=0) throw std::invalid_argument("Invalid Jacobian");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(j);
  const double s = j.rows()>j.cols()?0.0:svd.singularValues().minCoeff();
  return 1.0/(s*s+eps);
}
}
