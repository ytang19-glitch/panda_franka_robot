#include "panda_moveit/cost_functions.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <limits>
int main() {
  const double d[] = {0, 0.02, 0.05, 0.10, 0.15};
  const double expected[] = {0.01, 0.0064, 0.0025, 0, 0};
  for (int i=0;i<5;++i) {
    const double actual=panda_moveit::collisionCost(d[i],0.10);
    if(std::abs(actual-expected[i])>1e-12) return 1;
    std::cout << d[i] << "," << actual << "\n";
  }
  bool rejected=false;
  try { panda_moveit::collisionCost(std::numeric_limits<double>::quiet_NaN(),0.1); }
  catch(const std::invalid_argument&) { rejected=true; }
  if(!rejected) return 1;
  rejected=false;
  try { panda_moveit::collisionCost(0.1,-0.1); }
  catch(const std::invalid_argument&) { rejected=true; }
  if(!rejected) return 1;
  std::cout << "PASS: collision cost unit test\n";
  return 0;
}
