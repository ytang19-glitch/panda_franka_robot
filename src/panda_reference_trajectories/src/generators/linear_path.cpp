#include "panda_reference_trajectories/generators/path_generator.hpp"

#include <algorithm>
#include <cmath>

namespace panda_reference_trajectories
{

PoseSequence generateLinearPath(
  const Pose & start,
  const Pose & goal,
  double step_size)
{
  PoseSequence waypoints;

  // A positive interpolation step is required.
  if (step_size <= 0.0) {
    return waypoints;
  }

  const double dx =
    goal.position.x - start.position.x;

  const double dy =
    goal.position.y - start.position.y;

  const double dz =
    goal.position.z - start.position.z;

  const double distance = std::sqrt(
    dx * dx +
    dy * dy +
    dz * dz);

  // If both positions are equal, return one pose.
  if (distance < 1.0e-9) {
    waypoints.push_back(start);
    return waypoints;
  }

  const int samples = std::max(
    1,
    static_cast<int>(
      std::ceil(distance / step_size)));

  for (int i = 0; i <= samples; ++i) {
    const double s =
      static_cast<double>(i) /
      static_cast<double>(samples);

    Pose pose;

    pose.position.x =
      start.position.x + s * dx;

    pose.position.y =
      start.position.y + s * dy;

    pose.position.z =
      start.position.z + s * dz;

    /*
     * Keep the tool orientation constant in this first version.
     * Quaternion interpolation can be added later.
     */
    pose.orientation = start.orientation;

    waypoints.push_back(pose);
  }

  return waypoints;
}

}  // namespace panda_reference_trajectories