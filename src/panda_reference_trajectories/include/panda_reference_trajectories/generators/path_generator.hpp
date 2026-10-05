#ifndef PANDA_REFERENCE_TRAJECTORIES__GENERATORS__PATH_GENERATOR_HPP_
#define PANDA_REFERENCE_TRAJECTORIES__GENERATORS__PATH_GENERATOR_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace panda_reference_trajectories
{

// Make the function declarations shorter and easier to read.
using Pose = geometry_msgs::msg::Pose;
using PoseSequence = std::vector<Pose>;


/**
 * @brief Generate a point-to-point motion request.
 *
 * This function provides the start and goal poses. MoveIt determines
 * the collision-free path between them.
 *
 * @param start Initial end-effector pose.
 * @param goal Desired end-effector pose.
 * @return Sequence containing the start and goal poses.
 */
PoseSequence generatePointToPoint(
  const Pose & start,
  const Pose & goal);


/**
 * @brief Generate a straight Cartesian path.
 *
 * @param start Initial end-effector pose.
 * @param goal Final end-effector pose.
 * @param step_size Distance between consecutive points, in metres.
 * @return Interpolated Cartesian poses.
 */
PoseSequence generateLinearPath(
  const Pose & start,
  const Pose & goal,
  double step_size);


/**
 * @brief Generate a circular arc in the XY plane.
 *
 * @param center Pose defining the center and orientation of the arc.
 * @param radius Arc radius, in metres.
 * @param start_angle Initial angle, in radians.
 * @param end_angle Final angle, in radians.
 * @param step_size Approximate distance between consecutive points.
 * @return Cartesian poses along the arc.
 */
PoseSequence generateArcPath(
  const Pose & center,
  double radius,
  double start_angle,
  double end_angle,
  double step_size);


/**
 * @brief Generate a smooth Catmull-Rom spline.
 *
 * @param control_points Poses through which the spline should pass.
 * @param samples_per_segment Number of generated samples between
 * consecutive control points.
 * @return Smooth Cartesian pose sequence.
 */
PoseSequence generateSplinePath(
  const PoseSequence & control_points,
  unsigned int samples_per_segment);


/**
 * @brief Generate a raster or lawnmower-style surface path.
 *
 * The path consists of alternating parallel linear segments.
 *
 * @param start Pose at one corner of the surface.
 * @param width Surface width, in metres.
 * @param height Surface height, in metres.
 * @param line_spacing Distance between parallel rows, in metres.
 * @param step_size Distance between consecutive points, in metres.
 * @return Cartesian poses covering the surface.
 */
PoseSequence generateRasterPath(
  const Pose & start,
  double width,
  double height,
  double line_spacing,
  double step_size);


/**
 * @brief Generate a safe linear approach toward a target.
 *
 * @param target Final working pose.
 * @param approach_distance Distance before the target, in metres.
 * @param step_size Distance between consecutive points, in metres.
 * @return Cartesian approach path.
 */
PoseSequence generateApproachPath(
  const Pose & target,
  double approach_distance,
  double step_size);


/**
 * @brief Generate a safe linear retraction away from a target.
 *
 * @param target Initial working pose.
 * @param retract_distance Retraction distance, in metres.
 * @param step_size Distance between consecutive points, in metres.
 * @return Cartesian retraction path.
 */
PoseSequence generateRetractPath(
  const Pose & target,
  double retract_distance,
  double step_size);

}  // namespace panda_reference_trajectories

#endif  // PANDA_REFERENCE_TRAJECTORIES__GENERATORS__PATH_GENERATOR_HPP_