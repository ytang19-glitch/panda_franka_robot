# panda_nmpc

This package currently **measures joint tracking error only**. It reads `/joint_states` and a `trajectory_msgs/msg/JointTrajectory` reference on `/panda_nmpc/reference_trajectory`. It does not run an NMPC solver or command the robot.

## Run the simulation (three terminals)

Build once from `~/panda_franka_robot` with `colcon build --symlink-install`. In **each** new terminal, source ROS and the workspace, and use the same free `ROS_DOMAIN_ID`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
export ROS_DOMAIN_ID=71  # example; use a free domain for all three terminals
```

1. **Terminal 1 — Gazebo, controllers, MoveIt, RViz, and vision:**
   ```bash
   ros2 launch panda_bringup pick_and_place.launch.xml
   ```
2. **Terminal 2 — check readiness, then run the read-only diagnostic:**
   ```bash
   ros2 control list_controllers -c /controller_manager
   ros2 topic echo /joint_states --once
   ros2 run panda_nmpc nmpc_node --ros-args -p use_sim_time:=true
   ```
   Wait for `joint_state_broadcaster` and `arm_controller` to be active before starting the pick-and-place task.
3. **Terminal 3 — run the existing MoveIt pick-and-place task:**
   ```bash
   ros2 launch panda_bringup pick_and_place_commander.launch.xml target_color:=R
   ```

**Expected now:** The robot can follow the MoveIt plan, but the diagnostic may print no error: the commander does not yet publish its planned trajectory to `/panda_nmpc/reference_trajectory`. Check with `ros2 topic info /panda_nmpc/reference_trajectory -v`. The three terminals must share a ROS domain; avoid launching duplicate Panda or real FR3 control stacks into that domain.

## Which files to focus on

| Order | File | Why |
| --- | --- | --- |
| 1 | [`../panda_commander/src/panda_commander.cpp`](../panda_commander/src/panda_commander.cpp) | MoveIt plans and executes the task here. Publish each planned arm `JointTrajectory` here to feed the diagnostic. |
| 2 | [`panda_nmpc/nmpc_node.py`](panda_nmpc/nmpc_node.py) | Subscribes to measured joints and the reference; logs tracking error. Start here inside this package. |
| 3 | [`panda_nmpc/reference_trajectory.py`](panda_nmpc/reference_trajectory.py) and [`panda_nmpc/safety.py`](panda_nmpc/safety.py) | Map/interpolate the seven joints and validate the measurements. |
| 4 | [`panda_nmpc/robot_model.py`](panda_nmpc/robot_model.py) and [`panda_nmpc/optimizer.py`](panda_nmpc/optimizer.py) | Future dynamics model and NMPC solver; both are unfinished. |
| 5 | [`config/nmpc.yaml`](config/nmpc.yaml) and [`launch/nmpc_sim.launch.py`](launch/nmpc_sim.launch.py) | Topic names, diagnostic rate, and launch settings. |

**Next implementation step:** Publish the MoveIt arm trajectory from the commander, then verify the diagnostic prints `joint position error=... rad` while the robot moves. Only after that, implement and test the model and optimizer in simulation. The current diagnostic uses receipt time to sample the reference, so its error is an approximate tracking measurement.

For process cleanup and ROS-domain debugging, see [the troubleshooting guide](../../docs/Panda_ROS_Domain_and_NMPC_Debugging_2026-09-27.md).
