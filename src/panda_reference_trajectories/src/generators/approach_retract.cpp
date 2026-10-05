#include "panda_reference_trajectories/generators/path_generator.hpp"

namespace panda_reference_trajectories
{

PoseSequence generateApproachPath(
  const Pose & target,
  double approach_distance,
  double step_size)
{
  Pose approach = target;

  // Initial version: approach from above along the world Z-axis.
  approach.position.z += approach_distance;

  return generateLinearPath(approach, target, step_size);
}

PoseSequence generateRetractPath(
  const Pose & target,
  double retract_distance,
  double step_size)
{
  Pose retract = target;
  retract.position.z += retract_distance;

  return generateLinearPath(target, retract, step_size);
}

}  // namespace panda_reference_trajectories