# panda_nmpc

ROS 2 Panda reference-tracking scaffold. It reads measured joint positions and
a `trajectory_msgs/msg/JointTrajectory` reference, validates the reference,
and logs the position error. **It does not solve NMPC or send robot commands.**

## Layout

- `panda_nmpc/nmpc_node.py`: ROS subscriptions and read-only diagnostics.
- `panda_nmpc/reference_trajectory.py`: joint mapping and interpolation.
- `panda_nmpc/safety.py`: measured-state validation and error metric.
- `panda_nmpc/robot_model.py`: boundary for validated robot dynamics.
- `panda_nmpc/optimizer.py`: boundary for the future constrained solver.
- `config/nmpc.yaml`: ROS parameters.
- `launch/nmpc_sim.launch.py`: diagnostics launch file.

Build with `colcon build --packages-select panda_nmpc`, source the workspace,
then run `ros2 launch panda_nmpc nmpc_sim.launch.py`. Publish a MoveIt
`RobotTrajectory.joint_trajectory` to
`/panda_nmpc/reference_trajectory` to inspect tracking error. A trajectory
is a *reference* here; this node does not execute it. For a future controller,
implement dynamics, constraints, solver failure handling, and a separately
validated command interface, then test in simulation before hardware.


Run the current tracking diagnostic
Use the same ROS environment and ROS_DOMAIN_ID in every terminal.
Terminal A — start the Panda simulation and MoveIt:
```bash
cd /home/yujietang
git clone https://github.com/ytang19-glitch/panda_franka_robot.git
cd panda_franka_robot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix panda_nmpc
```
That repository launch starts Gazebo, the controllers, MoveIt, and RViz. Source: bringup launch

Terminal B — start the diagnostic node:\
```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 launch panda_nmpc nmpc_sim.launch.py
```
Terminal C — verify its inputs:
```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 topic echo /joint_states --once
ros2 topic info /panda_nmpc/reference_trajectory
```
The node will wait until you publish a trajectory_msgs/msg/JointTrajectory on /panda_nmpc/reference_trajectory. Once received, Terminal B prints joint position error=... rad.
