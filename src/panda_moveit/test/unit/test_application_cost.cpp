#include "panda_moveit/application_cost.hpp"
#include "panda_moveit/trajectory_optimizer.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
int main() {
  using namespace panda_moveit;
  if(std::abs(combinedCost({1,2,3,4},{2,3,4,5})-40)>1e-12) return 1;
  Configuration start(1), a(1), b(1), w(1);
  start<<0; a<<0.1; b<<0.2; w<<1;
  const std::vector<Candidates> layers{{a,b}};
  const EdgeValid valid=[](const Configuration&,const Configuration&){return true;};
  auto plain=optimizeTrajectory(layers,start,w,valid,[](const Configuration&){return 0.;});
  auto weighted=optimizeTrajectory(layers,start,w,valid,[](const Configuration& q){
    return combinedCost({0,0,q[0]<0.15?1.:0.,0},{0,0,10,0});});
  if(std::abs(plain[0][0]-0.1)>1e-12||std::abs(weighted[0][0]-0.2)>1e-12) return 1;
  std::cout<<"PASS: combined cost and candidate preference (synthetic graph)\n";
  return 0;
}
