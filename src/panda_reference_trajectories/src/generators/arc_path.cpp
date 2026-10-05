#include "panda_reference_trajectories/generators/path_generator.hpp"

#include <algorithm>
#include <cmath>

namespace panda_reference_trajectories
{

PoseSequence generateArcPath(
  const Pose & center,
  double radius,
  double start_angle,
  double end_angle,
  double step_size)
{
  PoseSequence waypoints;

  // Reject invalid parameters.
  if (radius <= 0.0 || step_size <= 0.0) {
    return waypoints;
  }

  const double angle_difference = end_angle - start_angle;

  // Arc length: L = radius × angle
  const double arc_length =
    radius * std::abs(angle_difference);

  // Calculate how many points are required.
  const int samples = std::max(
    1,
    static_cast<int>(
      std::ceil(arc_length / step_size)));

  for (int i = 0; i <= samples; ++i) {
    const double ratio =
      static_cast<double>(i) /
      static_cast<double>(samples);

    const double angle =
      start_angle + ratio * angle_difference;

    Pose pose;

    // Generate an arc in the XY plane.
    pose.position.x =
      center.position.x + radius * std::cos(angle);

    pose.position.y =
      center.position.y + radius * std::sin(angle);

    pose.position.z = center.position.z;

    // Maintain a constant tool orientation.
    pose.orientation = center.orientation;

    waypoints.push_back(pose);
  }

  return waypoints;
}

}  // namespace panda_reference_trajectories