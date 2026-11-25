#include "panda_moveit/cost_functions.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace panda_moveit {
double orientationError(const Eigen::Quaterniond& actual,const Eigen::Quaterniond& desired) {
  if(!actual.coeffs().allFinite()||!desired.coeffs().allFinite()||actual.norm()<1e-12||desired.norm()<1e-12)
    throw std::invalid_argument("Invalid orientation quaternion");
  return actual.normalized().angularDistance(desired.normalized());
}
double orientationCost(const Eigen::Quaterniond& actual,const Eigen::Quaterniond& desired) {
  const double e=orientationError(actual,desired);
  return e*e;
}
}
