#pragma once
#include <array>
namespace panda_moveit {
using CostTerms = std::array<double,7>;  // joint, singularity, clearance, smoothness, joint margin, orientation, wrist
double combinedCost(const CostTerms& normalized_terms, const CostTerms& weights);
}
