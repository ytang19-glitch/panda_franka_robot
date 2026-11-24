#pragma once
#include <Eigen/Core>
namespace panda_moveit {
double jointCost(const Eigen::VectorXd& q, const Eigen::VectorXd& ref, const Eigen::VectorXd& weights);
double singularityCost(const Eigen::MatrixXd& jacobian, double epsilon=1e-6);
double collisionCost(double distance, double safe_distance);
double smoothnessCost(const Eigen::MatrixXd& path);
}
