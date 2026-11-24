#include "panda_moveit/cost_functions.hpp"
#include <stdexcept>
namespace panda_moveit {
double jointCost(const Eigen::VectorXd& q, const Eigen::VectorXd& ref, const Eigen::VectorXd& w) {
  if (q.size()!=ref.size() || q.size()!=w.size() || !q.allFinite() || !ref.allFinite() || !w.allFinite() || (w.array()<0).any()) throw std::invalid_argument("Invalid joint cost input");
  return (w.array()*(q-ref).array().square()).sum();
}
}
