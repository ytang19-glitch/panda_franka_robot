#include "panda_reference_trajectories/generators/path_generator.hpp"

namespace panda_reference_trajectories
{

PoseSequence generatePointToPoint(
  const Pose & start,
  const Pose & goal)
{
  return {start, goal};
}

}  // namespace panda_reference_trajectories