#pragma once
#include <array>
namespace panda_moveit {
using CostTerms = std::array<double,4>;  // joint, singularity, clearance, smoothness
double combinedCost(const CostTerms& normalized_terms, const CostTerms& weights);
}
