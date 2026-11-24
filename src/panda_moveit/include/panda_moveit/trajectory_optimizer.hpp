#pragma once
#include <Eigen/Core>
#include <functional>
#include <vector>
namespace panda_moveit {
using Configuration=Eigen::VectorXd;
using Candidates=std::vector<Configuration>;
using EdgeValid=std::function<bool(const Configuration&,const Configuration&)>;
using StateCost=std::function<double(const Configuration&)>;
// Callers must generate pose-valid candidates and implement collision/edge checks.
Configuration weightedIK(const Candidates&,const Configuration&,const Configuration&,const EdgeValid&,const StateCost&);
std::vector<Configuration> optimizeTrajectory(const std::vector<Candidates>&,const Configuration&,const Configuration&,const EdgeValid&,const StateCost&);
}
