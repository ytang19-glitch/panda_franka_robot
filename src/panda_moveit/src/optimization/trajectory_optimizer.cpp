#include "panda_moveit/trajectory_optimizer.hpp"
#include "panda_moveit/cost_functions.hpp"
#include <limits>
#include <cmath>
#include <stdexcept>
#include <algorithm>
namespace panda_moveit {
// Dynamic programming: optimal additive cost over the supplied finite candidate graph,
// not a continuous trajectory optimizer. Second-difference costs require expanded state.
std::vector<Configuration> optimizeTrajectory(const std::vector<Candidates>& layers,const Configuration& start,const Configuration& w,const EdgeValid& valid,const StateCost& cost) {
  if(layers.empty()) throw std::invalid_argument("Empty candidate graph");
  const double inf=std::numeric_limits<double>::infinity();
  Candidates prev{start}; std::vector<double> scores{0.0};
  std::vector<std::vector<size_t>> parents;
  for(const auto& layer:layers){
    if(layer.empty()) throw std::runtime_error("Empty IK layer");
    std::vector<double> next(layer.size(),inf);
    std::vector<size_t> parent(layer.size(),0);
    for(size_t j=0;j<layer.size();++j){
      const double local=cost(layer[j]);
      if(!std::isfinite(local)) continue;
      for(size_t i=0;i<prev.size();++i){
        if(!std::isfinite(scores[i])||!valid(prev[i],layer[j])) continue;
        const double c=scores[i]+jointCost(layer[j],prev[i],w)+local;
        if(c<next[j]){next[j]=c;parent[j]=i;}
      }
    }
    parents.push_back(parent); scores=next;prev=layer;
  }
  auto best=std::min_element(scores.begin(),scores.end());
  if(!std::isfinite(*best)) throw std::runtime_error("No feasible candidate path");
  size_t j=static_cast<size_t>(best-scores.begin());
  std::vector<Configuration> out(layers.size());
  for(size_t k=layers.size();k-- >0;){out[k]=layers[k][j];j=parents[k][j];}
  return out;
}
}
