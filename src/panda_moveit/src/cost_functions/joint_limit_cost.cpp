#include "panda_moveit/cost_functions.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace panda_moveit {
double jointLimitCost(const Eigen::VectorXd& q,const Eigen::VectorXd& lo,const Eigen::VectorXd& hi,double margin) {
  if(q.size()!=lo.size()||q.size()!=hi.size()||!q.allFinite()||!lo.allFinite()||!hi.allFinite()||!std::isfinite(margin)||margin<=0||(lo.array()>=hi.array()).any())
    throw std::invalid_argument("Invalid bounded joint input");
  double cost=0;
  for(Eigen::Index i=0;i<q.size();++i){
    const double d=std::min(q[i]-lo[i],hi[i]-q[i]);
    const double e=std::max(0.0,1.0-d/margin);
    cost+=e*e;
  }
  return cost;
}
}
