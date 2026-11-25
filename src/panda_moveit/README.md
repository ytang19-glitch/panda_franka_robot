# Panda MoveIt: reference planning extension

Existing MoveIt launch, RViz, SRDF, controller and planner settings remain unchanged. The separate reference_tracker executable is a planning-only derivative of the existing Cartesian executor. It receives nav_msgs/Path, plans an approach, computes a complete collision-checked Cartesian path, retimes, checks timing and previews in RViz. It scores weighted joint displacement, geometric smoothness, scaled Jacobian singularity and world clearance from a frozen planning scene; weights DO NOT yet alter MoveIt's solution.

## Modules

| File | Responsibility |
|---|---|
| config/cost_weights.yaml | Per-joint and default scalar scoring weights; application presets override scalar weights |
| config/profiles/*.yaml | Robot group, TCP, topic and clock selection |
| include/panda_moveit/reference_tracker.hpp | Trajectory helper interfaces |
| include/panda_moveit/cost_functions.hpp | Cost interfaces |
| include/panda_moveit/trajectory_optimizer.hpp | Candidate graph interfaces |
| src/reference_tracker.cpp | Baseline coordinator, preview only |
| src/cost_functions/joint_cost.cpp | Weighted squared displacement |
| src/cost_functions/singularity_cost.cpp | Inverse squared minimum singular value |
| src/cost_functions/collision_cost.cpp | Clearance hinge penalty |
| src/cost_functions/smoothness_cost.cpp | Squared second differences |
| src/optimization/weighted_ik.cpp | Select lowest-cost feasible supplied candidate |
| src/optimization/trajectory_optimizer.cpp | Dynamic programming over supplied IK candidate layers |
| src/trajectory/trajectory_validator.cpp | Timing, finite values and dimensions only |
| src/trajectory/time_parameterization.cpp | MoveIt TOTG wrapper |
| launch/reference_tracking.launch.py | Select profile and start client |
| launch/gazebo_tracking.launch.py | Panda simulation shortcut |
| launch/real_robot_tracking.launch.py | FR3 planning-only shortcut |
| CMakeLists.txt / package.xml | Build and dependencies |

## Build and preview

```bash
cd ~/panda_franka_robot
colcon build --symlink-install --packages-select panda_moveit
source install/setup.bash
# With Panda Gazebo, controllers and original MoveIt already running:
ros2 launch panda_moveit reference_tracking.launch.py profile:=panda_gazebo
# In another sourced terminal, publish the reference:
ros2 run panda_reference_trajectories reference_generator
```

For FR3 Gazebo or hardware, start the matching official robot description, MoveIt and state publisher first; use profile:=fr3_gazebo or profile:=fr3_real and publish the matching frame and topic. All nodes must use the correct ROS_DOMAIN_ID. The client relies on the running MoveIt system making robot_description and robot_description_semantic available; supply matching local parameters if discovery fails. Verify group and TCP against the loaded model. No robot bringup is performed by these wrappers.

## Status and limitations

- No execution or controller command publication is enabled in the new tracker. The old executor is untouched.
- The weighted IK selector and candidate graph optimizer are implemented as independent helpers but NOT wired into the tracker. Generate pose-valid IK candidate layers, use a synchronized planning scene, supply hard state/edge validity checks, then integrate the optimizer. The candidate graph helper supports a state-cost callback, but the live tracker only scores baseline trajectories. Smoothness is not integrated into the graph solver.
- Timing validation does not establish collision safety. Retimed path collision, Cartesian accuracy, clearance, joint limits and continuity need independent verification before any future execution/export. The baseline preview can deviate from the exact path during retiming.
- Build and runtime have not been tested against ROS 2 Jazzy. Verify in Gazebo first. The same algorithms can be reused with FR3 profiles, not the Panda SRDF or controller config.

## Collision-cost experiment

See [GAZEBO_COLLISION_COST_GUIDE.md](GAZEBO_COLLISION_COST_GUIDE.md) for the numerical unit test and read-only MoveIt scene experiment. The new test uses a local virtual box sweep and exports measured clearance/cost to CSV; the original planner and execution interfaces remain unchanged.

## Application profiles

See [GAZEBO_APPLICATION_GUIDE.md](GAZEBO_APPLICATION_GUIDE.md). Select `application:=baseline`, `welding`, `deburring`, `polishing`, `clamp_avoidance` or `pick_and_place` separately from the robot profile. The four terms are evaluated and logged; `cost_csv_path` optionally exports them. These presets are scoring-only and illustrative. A synthetic graph unit test checks that a supplied cost can change the independent optimizer's selection.
