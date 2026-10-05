#include "panda_reference_trajectories/generators/path_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace panda_reference_trajectories
{

namespace
{

// Append a new segment while avoiding a duplicated connection point.
void appendSegment(
  PoseSequence & complete_path,
  const PoseSequence & segment)
{
  if (segment.empty()) {
    return;
  }

  std::size_t first_index = 0;

  // The first point of a connected segment is normally already
  // the final point of the previous segment.
  if (!complete_path.empty()) {
    first_index = 1;
  }

  complete_path.insert(
    complete_path.end(),
    segment.begin() + first_index,
    segment.end());
}

}  // namespace


PoseSequence generateRasterPath(
  const Pose & start,
  double width,
  double height,
  double line_spacing,
  double step_size)
{
  PoseSequence waypoints;

  // Reject invalid dimensions.
  if (
    width <= 0.0 ||
    height < 0.0 ||
    line_spacing <= 0.0 ||
    step_size <= 0.0)
  {
    return waypoints;
  }

  /*
   * Add one because both the first row at y = 0 and the
   * final row at y = height should be included.
   */
  const int number_of_rows =
    static_cast<int>(
      std::ceil(height / line_spacing)) + 1;

  for (int row = 0; row < number_of_rows; ++row) {
    Pose row_start = start;
    Pose row_end = start;

    // Prevent the final row from extending beyond the requested height.
    const double y_offset = std::min(
      static_cast<double>(row) * line_spacing,
      height);

    row_start.position.y += y_offset;
    row_end.position.y += y_offset;

    /*
     * Even rows move in the positive X direction.
     *
     * Odd rows move in the negative X direction.
     */
    if (row % 2 == 0) {
      row_end.position.x += width;
    } else {
      row_start.position.x += width;
    }

    /*
     * Connect the previous row to the beginning of this row.
     * This produces the short vertical part of the raster pattern.
     */
    if (!waypoints.empty()) {
      const Pose & previous_end = waypoints.back();

      PoseSequence connector = generateLinearPath(
        previous_end,
        row_start,
        step_size);

      appendSegment(waypoints, connector);
    }

    // Generate the horizontal row.
    PoseSequence line = generateLinearPath(
      row_start,
      row_end,
      step_size);

    appendSegment(waypoints, line);
  }

  return waypoints;
}

}  // namespace panda_reference_trajectories