#include "panda_moveit/cost_functions.hpp"
#include <stdexcept>
namespace panda_moveit {
double smoothnessCost(const Eigen::MatrixXd& q) {
  if (!q.allFinite()) throw std::invalid_argument("Nonfinite path");
  double cost=0;
  for(Eigen::Index k=1;k+1<q.rows();++k) cost+=(q.row(k+1)-2*q.row(k)+q.row(k-1)).squaredNorm();
  return cost;
}
}
