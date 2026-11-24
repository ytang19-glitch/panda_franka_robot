#pragma once
#include <moveit/planning_scene/planning_scene.hpp>
namespace panda_moveit {
struct Clearance { double distance; bool colliding; };
Clearance measureClearance(const planning_scene::PlanningScene& scene, moveit::core::RobotState& state);
}
