# Panda MoveIt RViz drag handle and execution troubleshooting

This guide records the September 27, 2026 Panda test. The end-effector marker became draggable and a planned trajectory executed after the running nodes stopped waiting for a missing ROS simulation clock.

## Confirmed result

The test used `ros2 launch panda_moveit moveit.launch.py is_sim:=true`. At one point `/rviz2`, `/move_group`, and `/controller_manager` reported `use_sim_time: True`, while `ros2 topic info /clock` reported **Publisher count: 0** (and 16 subscribers). The arm joint states arrived at about 100 Hz, and `arm_controller`, `joint_state_broadcaster`, and `gripper_controller` were active. Changing the running nodes to `use_sim_time: False` restored dragging and trajectory execution.

The decisive execution log was:

```text
Motion plan was computed successfully.
sending trajectory to arm_controller
Controller 'arm_controller' successfully finished
Completed trajectory execution with status SUCCEEDED
Execution completed: SUCCEEDED
```

This confirms that MoveIt planned and the arm controller reported successful completion. It does not, by itself, establish that a Gazebo physics world was running. RViz can visualize and execute motion through suitable virtual `ros2_control` hardware without Gazebo.

## Choose the correct clock

| Setup | ROS `/clock` publisher | `use_sim_time` for RViz, MoveIt, and state-producing nodes |
| --- | --- | --- |
| MoveIt/RViz with a virtual controller but no ROS simulation clock | None | `false` |
| Gazebo with the clock bridged to ROS | Present and advancing | `true` |
| Gazebo is intended, but the ROS clock bridge is absent or broken | None | Fix Gazebo/bridge first; do not mix clocks |

`is_sim` is a **custom launch argument**, not a ROS-wide switch that starts Gazebo. In the GitHub version of `src/panda_moveit/launch/moveit.launch.py` inspected for this guide, it is used to set `use_sim_time` on `move_group`. The user's local file also sets RViz's `use_sim_time` from `is_sim`; it has local changes not yet reflected in that GitHub file. This MoveIt launch by itself does not start Gazebo or the controller manager. Check the actual launch files and hardware configuration before treating a controller as virtual or sending an Execute command.

The composed `src/panda_bringup/launch/pick_and_place.launch.xml` includes Gazebo, controllers, and MoveIt. Its Gazebo launch includes `robot_state_publisher` with simulation time, and `src/panda_description/config/parameter_bridge.yaml` declares a Gazebo-to-ROS `/clock` bridge. A declared bridge is not proof that a ROS `/clock` publisher is currently alive.

### Inspect the running system

Run these in a second terminal sourced with the same workspace as the launch:

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 node list
ros2 topic info /clock
timeout 5s ros2 topic echo /clock --once
ros2 param get /rviz2 use_sim_time
ros2 param get /move_group use_sim_time
ros2 param get /controller_manager use_sim_time
```

`Publisher count: 0` on `/clock` means no ROS node currently publishes it. `ros2 topic echo` timing out is consistent with that observation. If a node does not exist, `ros2 param get` reports `Node not found`; do not set a parameter on a guessed node name. For example, `/robot_state_publisher` did not exist in the standalone launch observed here. Use `ros2 node list` to find the actual names.

### Temporary recovery for a run without a ROS `/clock`

If you have confirmed this is a virtual setup and no ROS clock is being published, change the *running* nodes:

```bash
ros2 param set /rviz2 use_sim_time false
ros2 param set /move_group use_sim_time false
ros2 param set /controller_manager use_sim_time false
```

Run only the commands for nodes that exist, then verify each with `ros2 param get`. These changes do **not** persist across a restart. A live clock change can produce a time jump; restart the stack cleanly after fixing the launch configuration. Do not click Execute until you have confirmed the controller is connected to virtual hardware.

For an intentional Gazebo run, keep simulation time enabled and make the ROS `/clock` bridge publish and advance. Inspect `src/panda_description/launch/gazebo.launch.xml` and `src/panda_description/config/parameter_bridge.yaml` if `/clock` has no publisher.

### Permanent configuration

Inspect the *source* files (not only `install/`):

```bash
grep -RInE 'use_sim_time|is_sim' ~/panda_franka_robot/src/panda_moveit ~/panda_franka_robot/src/panda_bringup
```

The local search during this incident found `{"use_sim_time": is_sim}` at lines 45 and 67 of `src/panda_moveit/launch/moveit.launch.py`, plus `is_sim=True` in the composed bringup. This directly couples the custom `is_sim` argument to the node clock. For a standalone run **without** a ROS `/clock`, set the node clocks to `false` in the launch configuration. For a Gazebo run, ensure its ROS clock works before setting them to `true`. If supporting both modes, a separate `use_sim_time` launch argument is clearer than assuming `is_sim` proves that a clock exists. Apply it consistently to RViz, MoveIt, and any controller/state publisher started by the chosen bringup.

Because the local launch file contains changes absent from GitHub `main`, review or push that file separately rather than copying an older parameter block over it. Rebuild and restart after source edits:

```bash
cd ~/panda_franka_robot
colcon build --symlink-install --packages-select panda_moveit
source install/setup.bash
```

Stop the old launch with **Ctrl+C**, then start the intended configuration and repeat the clock checks. `source install/setup.bash` changes the shell environment for subsequent commands; it does not change the parameters of already-running nodes.

## Fixed Frame, IK, and the drag marker

These are separate from the clock issue:

- In RViz, **Global Options → Fixed Frame** controls data transforms. `panda_link0` was the working base frame when `world` was absent. Use `world` only when TF connects it to the Panda base. **Views → Current View → Target Frame** controls the camera view and is a different setting.
- Use **Interact**, select planning group `arm`, and enable **MotionPlanning → Planning Request → Query Goal State**. Drag a colored arrow to translate or a ring to rotate the orange goal marker.
- RViz needs the `arm` IK configuration in `robot_description_kinematics`. Earlier in this incident that RViz parameter was initially absent, even though `src/panda_moveit/config/kinematics.yaml` contained the KDL entry. Check with:

```bash
ros2 param get /rviz2 robot_description_kinematics.arm.kinematics_solver
```

The expected solver for this configuration is `kdl_kinematics_plugin/KDLKinematicsPlugin`. The user's later local launch passed `moveit_config.robot_description_kinematics` to RViz. A visible marker and green MotionPlanning status alone do not prove that a goal can be dragged or planned.

Save RViz view changes with **File → Save Config** to `src/panda_moveit/rviz/moveit.rviz` if they should persist. Do not publish a duplicate `world → panda_link0` transform; use an identity transform only if the Panda base is actually at the world origin.

## Isolate planning from execution

1. In RViz, move the goal a small amount and click **Plan**. If planning fails or Execute stays disabled, try a small goal in the **Joints** tab to distinguish marker/IK problems from planning problems. Read the `move_group` log at the moment of failure.
2. If Plan succeeds but Execute does not move the virtual robot, check the controller and action endpoints:

```bash
ros2 control list_controllers
ros2 action list -t
timeout 5s ros2 topic echo /joint_states --once
```

Look for `/execute_trajectory [moveit_msgs/action/ExecuteTrajectory]` and `/arm_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]`. Compare the actual action and joint names with `src/panda_moveit/config/moveit_controllers.yaml`. An `active` controller alone does not prove that MoveIt can address it. Check whether `/joint_states` changes after a virtual execution.

3. A successful execution should show `sending trajectory to arm_controller`, `Controller 'arm_controller' successfully finished`, and `status SUCCEEDED` in the launch terminal. The orange robot may remain visible because it is the **goal state** visualization; it is not, by itself, evidence of failed execution.

The `RTPS_TRANSPORT_SHM Error` messages observed here did not prevent the roughly 100 Hz joint-state stream or the later successful execution. Diagnose them separately if communication is missing. Press **Ctrl+C** to stop `ros2 topic hz`; **Ctrl+Z** suspends the process and leaves a shell job. `rg` was not installed on the user's machine, so use `grep` or run `ros2 action list -t` without a pipeline. A missing pipe command can cause Python's `BrokenPipeError`, which does not indicate a MoveIt action failure.

## References

- [MoveIt: Quickstart in RViz](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)
- [MoveIt: Low Level Controllers](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html)
- [ROS 2 Jazzy: Understanding parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
