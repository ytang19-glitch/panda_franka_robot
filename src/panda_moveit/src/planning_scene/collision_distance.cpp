#include "panda_moveit/collision_distance.hpp"
#include <stdexcept>
#include <cmath>
namespace panda_moveit {
Clearance measureClearance(const planning_scene::PlanningScene& scene, moveit::core::RobotState& state) {
  state.update();
  const bool colliding = scene.isStateColliding(state);
  const double distance = scene.distanceToCollision(state);
  if (!std::isfinite(distance)) throw std::runtime_error("Nonfinite world clearance");
  return {distance, colliding};
}
}
