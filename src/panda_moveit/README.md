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


## Industrial surfacing: problem-to-solution roadmap

See [cost function mathematics and robot figures](src/cost_functions/README.md).

| Industrial problem | Proposed objective / constraint | Current implementation |
|---|---|---|
| Wrist singularity | Robot-specific wrist margin; for a suitable spherical wrist, enforce absolute J5 >= required margin | General Jacobian score only; dedicated wrist constraint pending |
| Shoulder singularity | Minimum scaled Jacobian singular value and robot-specific geometry checks | General singularity evaluator implemented; hard threshold pending |
| Elbow singularity | Minimum scaled Jacobian singular value and robot-specific geometry checks | General singularity evaluator implemented; hard threshold pending |
| Poor manipulability / high condition number | Maximize minimum singular value; optionally report manipulability and condition number | Minimum singular value cost implemented; separate metrics pending |
| Insufficient joint limit margin | Normalized distance from joint bounds; enforce physical bounds | Weighted reference displacement exists, but is not a joint-limit margin cost |
| Insufficient collision clearance | Clearance hinge cost plus hard state and edge collision checks | World clearance scoring exists; optimized-path validation pending |
| Torch angle deviation | Rotation error cost within approved process tolerance | Dedicated orientation cost and tolerance sampling pending |

### Proposed optimization formulation

For candidate configurations at each TCP waypoint, minimize

$$\sum_k \left(w_s C_s(q_k)+w_c C_c(q_k)+w_l C_l(q_k)+w_o C_o(q_k)+w_w C_w(q_k)\right)+\sum_{k>0}\|q_k-q_{k-1}\|_W^2.$$

The existing singularity objective is $C_s=1/(\sigma_{\min}^2+\epsilon)$, evaluated on a consistently scaled Jacobian. The existing candidate graph solver minimizes additive state and transition costs over supplied candidate layers, not over all continuous configurations. The other terms above are proposed extensions. A fixed exact TCP orientation makes orientation cost constant across exact IK branches; sample allowed orientation variations only when the manufacturing process permits them.

### Integration sequence

1. Generate multiple pose-valid IK candidates for every waypoint, using multiple seeds or a compatible IK plugin. Reject invalid states using the synchronized planning scene.
2. Evaluate scaled Jacobian singular values, robot-specific margins, joint bounds, clearance and torch pose. Reject states violating mandatory thresholds instead of merely penalizing them.
3. Supply the remaining candidates and additive state scores to the existing dynamic-programming helper in [trajectory_optimizer.cpp](src/optimization/trajectory_optimizer.cpp). Validate each transition for collisions, singularity margins, joint continuity and Cartesian path deviation; reject unreachable layers.
4. Time-parameterize the selected sequence and independently validate the resulting interpolated path, joint velocity/acceleration limits and required TCP process speed. TOTG alone does not guarantee constant welding speed.
5. Compare baseline and optimized plans in Gazebo before any supervised hardware dry run. Keep execution disabled until robot-specific safety checks and controller integration are established.

### Baseline versus optimized validation

Use identical start states, toolpaths, scene geometry and process tolerances. Record minimum scaled singular value, maximum condition number, minimum joint-limit margin, minimum environmental and self clearance, maximum torch angular error, maximum joint velocity, TCP speed error, planning time and successful path completion. Report infeasible cases instead of relaxing mandatory constraints.

**Robot portability:** the J5-near-zero criterion is specific to appropriate six-axis spherical-wrist robots and must not be assumed for Panda/FR3. Use the actual robot model, calibrated TCP and fixtures, and separately validated thresholds. MoveIt is a planning backend, not a safety-rated controller. No real-robot feasibility or singularity avoidance is claimed by this roadmap.


## Industrial surfacing costs

The surfacing application adds joint-limit margin and constant nominal torch orientation scoring, a threshold-based singularity cost and manipulability/condition-number diagnostics. Optional J5 scoring stays disabled for Panda/FR3. See [cost function documentation](src/cost_functions/README.md). Run `ros2 run panda_moveit test_surfacing_cost` after building with testing enabled. Select `application:=surfacing` for planning-only evaluation. No live optimization or new hard process constraints are enabled.
