#include "panda_moveit/cost_functions.hpp"
#include <cmath>
#include <iostream>
int main(){
 using namespace panda_moveit;
 const auto near=[](double a,double b){return std::abs(a-b)<1e-10;};
 Eigen::MatrixXd j=Eigen::MatrixXd::Identity(6,7);
 auto good=jacobianMetrics(j);
 j.row(5).setZero(); auto bad=jacobianMetrics(j);
 if(!near(good.sigma_min,1)||!near(good.manipulability,1)||!near(bad.sigma_min,0)||!std::isinf(bad.condition_number)) return 1;
 if(!near(singularityMarginCost(0.05,0.1),0.25)||!near(singularityMarginCost(0.2,0.1),0)) return 1;
 Eigen::VectorXd q(1),lo(1),hi(1);lo<<-1;hi<<1;q<<0.95;
 if(!near(jointLimitCost(q,lo,hi,0.1),0.25)) return 1;
 Eigen::Quaterniond a=Eigen::Quaterniond::Identity(),r(Eigen::AngleAxisd(0.2,Eigen::Vector3d::UnitZ()));
 if(!near(orientationCost(a,r),0.04)) return 1;
 Eigen::Quaterniond neg=a;neg.coeffs()*=-1;
 if(!near(orientationCost(a,neg),0)) return 1;
 if(!near(wristSingularityCost(-0.1,0.2,false),0.25)||!near(wristSingularityCost(0.3,0.2,true),0)) return 1;
 std::cout<<"PASS: surfacing cost functions\n";return 0;
}
