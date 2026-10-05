#include "panda_reference_trajectories/generators/path_generator.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cstddef>

namespace panda_reference_trajectories
{

namespace
{

// Calculate one coordinate of a Catmull-Rom spline.
double catmullRom(
  double p0,
  double p1,
  double p2,
  double p3,
  double t)
{
  const double t2 = t * t;
  const double t3 = t2 * t;

  return 0.5 * (
    2.0 * p1 +
    (-p0 + p2) * t +
    (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
    (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
}


// Smoothly interpolate orientation from pose p1 to pose p2.
geometry_msgs::msg::Quaternion interpolateOrientation(
  const geometry_msgs::msg::Quaternion & start,
  const geometry_msgs::msg::Quaternion & goal,
  double t)
{
  tf2::Quaternion q_start;
  tf2::Quaternion q_goal;

  tf2::fromMsg(start, q_start);
  tf2::fromMsg(goal, q_goal);

  q_start.normalize();
  q_goal.normalize();

  tf2::Quaternion q_interpolated = q_start.slerp(q_goal, t);
  q_interpolated.normalize();

  return tf2::toMsg(q_interpolated);
}

}  // namespace


PoseSequence generateSplinePath(
  const PoseSequence & control_points,
  unsigned int samples_per_segment)
{
  PoseSequence waypoints;

  if (control_points.empty()) {
    return waypoints;
  }

  if (control_points.size() == 1) {
    waypoints.push_back(control_points.front());
    return waypoints;
  }

  // Prevent zero samples per segment.
  samples_per_segment = std::max(1U, samples_per_segment);

  /*
   * Each spline segment travels from p1 to p2.
   *
   * p0 and p3 determine the shape and tangent direction around the
   * segment. At the beginning and end of the path, the nearest endpoint
   * is repeated because an additional neighbouring point does not exist.
   */
  for (std::size_t segment = 0;
    segment + 1 < control_points.size();
    ++segment)
  {
    const std::size_t index_p0 =
      (segment == 0) ? 0 : segment - 1;

    const std::size_t index_p1 = segment;
    const std::size_t index_p2 = segment + 1;

    const std::size_t index_p3 =
      std::min(segment + 2, control_points.size() - 1);

    const Pose & p0 = control_points[index_p0];
    const Pose & p1 = control_points[index_p1];
    const Pose & p2 = control_points[index_p2];
    const Pose & p3 = control_points[index_p3];

    for (unsigned int sample = 0;
      sample <= samples_per_segment;
      ++sample)
    {
      // Avoid adding a control point twice between adjacent segments.
      if (segment > 0 && sample == 0) {
        continue;
      }

      const double t =
        static_cast<double>(sample) /
        static_cast<double>(samples_per_segment);

      Pose pose;

      pose.position.x = catmullRom(
        p0.position.x,
        p1.position.x,
        p2.position.x,
        p3.position.x,
        t);

      pose.position.y = catmullRom(
        p0.position.y,
        p1.position.y,
        p2.position.y,
        p3.position.y,
        t);

      pose.position.z = catmullRom(
        p0.position.z,
        p1.position.z,
        p2.position.z,
        p3.position.z,
        t);

      // SLERP avoids invalid quaternion interpolation.
      pose.orientation = interpolateOrientation(
        p1.orientation,
        p2.orientation,
        t);

      waypoints.push_back(pose);
    }
  }

  return waypoints;
}

}  // namespace panda_reference_trajectories