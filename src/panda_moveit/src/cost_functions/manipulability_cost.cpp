#include "panda_moveit/cost_functions.hpp"
#include <Eigen/SVD>
#include <limits>
#include <stdexcept>
namespace panda_moveit {
JacobianMetrics jacobianMetrics(const Eigen::MatrixXd& j) {
  if(j.rows()==0||j.cols()==0||!j.allFinite()) throw std::invalid_argument("Invalid Jacobian");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(j);
  const auto& s=svd.singularValues();
  const double min=j.rows()>j.cols()?0.0:s.minCoeff(), max=s.maxCoeff();
  return {min,max,j.rows()>j.cols()?0.0:s.prod(),min>0?max/min:std::numeric_limits<double>::infinity()};
}
}
