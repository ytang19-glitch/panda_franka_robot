# Gazebo collision-cost validation (no motion)

## 1. Build
```bash
cd ~/panda_franka_robot
git pull --ff-only origin main
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/panda_moveit --ignore-src -r -y
colcon build --symlink-install --packages-select panda_moveit --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 run panda_moveit test_collision_cost
```
Expected costs for distances 0, .02, .05, .10, .15 m: .01, .0064, .0025, 0, 0. PASS confirms only numerical formula.

## 2. Start the existing simulated Panda
Use a simulation-only ROS domain (e.g. 10) in every terminal; do not connect hardware. If already running, do not launch duplicate servers. Existing full bringup:
```bash
export ROS_DOMAIN_ID=10
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.xml
```
Do not launch the commander or execute plans. In another terminal with same domain and sourced workspace:
```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 service list | grep get_planning_scene
```
The simulated joint state broadcaster and MoveIt service must be available. Keep arm stationary and initially collision-free.

## 3. Run geometry experiment
```bash
ros2 launch panda_moveit collision_cost_test.launch.py
```
The node reads a full MoveIt scene snapshot and live arm state, then sweeps a virtual 10 cm box toward the selected link origin along model X. The box is local to the test scene, not visible in Gazebo or RViz, and is never sent to the live planning scene. No commands are sent to the robot. The model frame defaults to world due to the Panda virtual joint; if the model reports panda_link0, pass base_frame:=panda_link0. The reported distance is minimum robot-to-world distance, not necessarily distance to the probe. Self collision is checked separately via isStateColliding. Existing world obstacles may dominate; a far sample may already have positive cost. Adjust offsets in config/obstacles.yaml if coverage is incomplete. Return code 2 means incomplete coverage, not necessarily a mathematical error.

```bash
cat /tmp/panda_collision_cost.csv
ros2 launch panda_moveit collision_cost_test.launch.py collision_weight:=0.0 csv_path:=/tmp/collision_w0.csv
ros2 launch panda_moveit collision_cost_test.launch.py collision_weight:=100.0 csv_path:=/tmp/collision_w100.csv
```
With stationary state and identical scene, distances and raw costs should match; weighted cost is zero in first run and 100 times raw cost in second. This does NOT alter planning. PASS requires observing zero cost, positive cost and a detected collision. Offsets are not measured clearance and need not produce monotonic clearance.

## 4. Inspect results
```bash
python3 - <<'PY'
import csv
with open('/tmp/panda_collision_cost.csv') as f:
    for r in csv.DictReader(f):
        d=float(r['distance_m'])
        expected=max(0.0,0.1-d)**2
        assert abs(float(r['cost'])-expected)<1e-9
        print(r)
print('PASS: measured-distance cost formula')
PY
```
No trajectory optimization is tested yet. Next: integrate candidate generation and hard state/edge validation, then compare clearance of baseline and optimized reference paths. Never infer hardware safety from this experiment. Build/runtime not yet verified on ROS Jazzy.
