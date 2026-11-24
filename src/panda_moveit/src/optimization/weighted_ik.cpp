#include "panda_moveit/trajectory_optimizer.hpp"
#include "panda_moveit/cost_functions.hpp"
#include <limits>
#include <cmath>
#include <stdexcept>
namespace panda_moveit {
Configuration weightedIK(const Candidates& candidates,const Configuration& prev,const Configuration& w,const EdgeValid& valid,const StateCost& cost) {
  double best=std::numeric_limits<double>::infinity(); Configuration out;
  for(const auto& q:candidates) {
    if(!valid(prev,q)) continue;
    double c=jointCost(q,prev,w)+cost(q);
    if(std::isfinite(c)&&c<best){best=c;out=q;}
  }
  if(out.size()==0) throw std::runtime_error("No feasible IK candidate");
  return out;
}
}
