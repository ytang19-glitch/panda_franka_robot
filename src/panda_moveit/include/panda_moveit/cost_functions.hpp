#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace panda_moveit {
double jointCost(const Eigen::VectorXd& q, const Eigen::VectorXd& ref, const Eigen::VectorXd& weights);
double singularityCost(const Eigen::MatrixXd& jacobian, double epsilon=1e-6);
double collisionCost(double distance, double safe_distance);
double smoothnessCost(const Eigen::MatrixXd& path);
struct JacobianMetrics { double sigma_min; double sigma_max; double manipulability; double condition_number; };
JacobianMetrics jacobianMetrics(const Eigen::MatrixXd& jacobian);
double singularityMarginCost(double sigma_min, double desired);
double jointLimitCost(const Eigen::VectorXd& q, const Eigen::VectorXd& lower, const Eigen::VectorXd& upper, double margin);
double orientationError(const Eigen::Quaterniond& actual, const Eigen::Quaterniond& desired);
double orientationCost(const Eigen::Quaterniond& actual, const Eigen::Quaterniond& desired);
double wristSingularityCost(double angle, double desired, bool positive_branch);
}
