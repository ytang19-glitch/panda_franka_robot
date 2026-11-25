#include "panda_moveit/cost_functions.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace panda_moveit {
// Optional local J5-near-zero policy for a verified spherical-wrist robot.
// Not a general singularity detector (does not cover other aligned wrist angles).
double wristSingularityCost(double angle,double desired,bool positive_branch) {
  if(!std::isfinite(angle)||!std::isfinite(desired)||desired<=0) throw std::invalid_argument("Invalid wrist angle");
  const double margin=positive_branch?angle:std::abs(angle);
  const double e=std::max(0.0,1.0-margin/desired);
  return e*e;
}
}
