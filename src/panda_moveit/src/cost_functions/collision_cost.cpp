#include "panda_moveit/cost_functions.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace panda_moveit {
double collisionCost(double d, double safe) {
  if (!std::isfinite(d) || !std::isfinite(safe) || safe<0) throw std::invalid_argument("Invalid distance");
  const double gap=std::max(0.0,safe-d);
  return gap*gap;
}
}
