# Application profiles: Gazebo validation

Robot profile selects group, TCP, clock and topic. Application profile selects four cost weights. Existing MoveIt launch and original Cartesian executor are unchanged. The new tracker is strictly preview-only.

## Build
```bash
cd ~/panda_franka_robot
git status
git pull --ff-only origin main
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/panda_moveit --ignore-src -r -y
colcon build --symlink-install --packages-select panda_moveit panda_reference_trajectories --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run panda_moveit test_collision_cost
ros2 run panda_moveit test_application_cost
```

## Start simulation (Terminal 1)
Use the same simulation-only domain in all terminals. Do not connect real hardware. If Gazebo and MoveIt are already running, do not duplicate them.
```bash
export ROS_DOMAIN_ID=10
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.xml
```
Check /joint_states, /clock and /get_planning_scene. Ensure the arm is stationary and collision free.

## Reference (Terminal 2)
In every new terminal export ROS_DOMAIN_ID=10 and source both environments as above.
```bash
ros2 run panda_reference_trajectories reference_generator
```
The default path uses panda_link0 and /panda_reference/cartesian_path. If unreachable, adjust reference position and orientation; do not disable collision checks.

## Preview and score (Terminal 3)
```bash
ros2 launch panda_moveit reference_tracking.launch.py profile:=panda_gazebo application:=baseline cost_csv_path:=/tmp/baseline.csv
ros2 launch panda_moveit reference_tracking.launch.py profile:=panda_gazebo application:=welding cost_csv_path:=/tmp/welding.csv
ros2 launch panda_moveit reference_tracking.launch.py profile:=panda_gazebo application:=clamp_avoidance cost_csv_path:=/tmp/clamp.csv
```
Expected log: Cartesian path completion 100%, application SCORING ONLY, four normalized terms, weights, total, minimum sampled world clearance and invalid sample count. The preview is published to RViz only if no sampled invalid state is found. Inspect CSV with cat /tmp/baseline.csv. If the world is empty, the distance backend may report a very large finite sentinel, meaning no nearby world geometry, not a physical measurement.

The score uses normalized per-sample terms on the retimed process segment (not the approach): mean squared weighted joint increments divided by joint_step_scale squared; mean inverse squared minimum singular value of the scaled Jacobian times singularity_scale squared; mean squared clearance shortfall divided by desired_clearance squared; mean squared second differences divided by joint_step_scale squared. These depend on sampling and are not calibrated physical quantities. Weights are illustrative. The singularity metric scales angular Jacobian rows by characteristic_length.

## Independent collision geometry test
```bash
ros2 launch panda_moveit collision_cost_test.launch.py
cat /tmp/panda_collision_cost.csv
```
See GAZEBO_COLLISION_COST_GUIDE.md. The virtual box sweep is local to the test and does not affect the reference tracker or live RViz scene. To evaluate clearance of a path near an obstacle, add a collision object to the live MoveIt planning scene using RViz and confirm it is present before planning; keep the robot stationary.

## Interpret correctly
Changing application weights changes scoring of a path, not MoveIt's planner objective. Repeated plans can differ due to randomized planning; compare weighted totals for identical raw terms or recompute all presets from a single CSV row. Do not compare differently weighted totals as though they were the same metric. The synthetic candidate graph unit test verifies that a supplied state cost can change the independent optimizer's selection. Live candidate generation and optimizer integration remain pending. The tracker checks sampled states only, not continuous edges, exact Cartesian tracking, or real hardware safety. No execution is enabled. ROS Jazzy build/runtime not yet verified here.
