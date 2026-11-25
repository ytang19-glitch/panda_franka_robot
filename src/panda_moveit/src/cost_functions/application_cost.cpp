#include "panda_moveit/application_cost.hpp"
#include <cmath>
#include <stdexcept>
namespace panda_moveit {
double combinedCost(const CostTerms& t,const CostTerms& w) {
  double result=0;
  for (std::size_t i=0;i<t.size();++i) {
    if(!std::isfinite(t[i]) || t[i]<0 || !std::isfinite(w[i]) || w[i]<0)
      throw std::invalid_argument("Cost terms and weights must be finite and nonnegative");
    result+=t[i]*w[i];
  }
  if(!std::isfinite(result)) throw std::overflow_error("Nonfinite total cost");
  return result;
}
}
