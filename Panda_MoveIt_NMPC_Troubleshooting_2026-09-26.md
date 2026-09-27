# Panda MoveIt and `panda_nmpc`: Troubleshooting Notes

Date: September 26, 2026 (Edmonton)  
Workspace: `/home/yujietang/panda_franka_robot`  
Environment: ROS 2 Jazzy, Gazebo, MoveIt 2

## Current status

The latest screenshot shows **MotionPlanning → Status: Ok** in RViz, with planning group `arm` and planning scene topic `monitored_planning_scene`. The previous **“Requesting initial scene failed”** warning is gone. This does not yet verify that `arm_controller` is active, that a MoveIt trajectory has executed, or that NMPC has commanded the robot.

## Issues, evidence, and actions

| Issue | Evidence | Action or result |
| --- | --- | --- |
| `nmpc_sim.launch.py` was not found | The local package had not yet been updated and built with that launch file. | Update the new workspace, run `colcon build --symlink-install --packages-select panda_nmpc`, and source its install directory. The node subsequently started and reported `Read-only tracking node started; no commands are sent`. |
| `/joint_states` was unavailable and the reference topic was unknown | The simulation and diagnostic node were not both running at that point. | Start `panda_bringup pick_and_place.launch.xml` and `panda_nmpc nmpc_sim.launch.py` in separate terminals. Later, joint states were received and the reference topic had one subscriber. |
| Position error was nearly zero but the Panda did not move | The published `JointTrajectory` was a stationary zero reference; the current node only measures position error. | This verified reference reception and position comparison. It was not an NMPC control test. |
| RViz reported `PlanningScene: Requesting initial scene failed` | `/move_group` and `/get_planning_scene` appeared in the ROS graph, but one service call remained at `requester: making request...`. Visible endpoints alone did not prove a successful reply. | Inspect the bringup log and workspace environment. After the environment was cleaned and the stack relaunched, RViz showed `Status: Ok`. The available evidence does not establish a single definitive cause for the service delay. |
| Old workspace paths and mixed Panda/FR3 data appeared | `AMENT_PREFIX_PATH` contained both `panda_franka_robot` and `Franka-Panda-Robot-Project`. Logs also showed duplicate `/rviz2` names and an unconnected `fr3_link0` transform. | Stop other simulation and MoveIt instances. Clear inherited overlay variables in a fresh shell, then source Jazzy and only the new workspace's `local_setup.bash`. The subsequent path check no longer showed the old workspace. |
| Controller initialization failed in the old launch | A standalone `ros2_control_node` could not load `gz_ros2_control/GazeboSimSystem`; `joint_state_broadcaster` failed to configure and `arm_controller` waited. The new source `panda_controller/launch/controller.launch.xml` says it uses Gazebo's controller manager instead. | Compare the installed launch file with the new source and run the new workspace alone. `gz_ros2_control` itself was found under `/opt/ros/jazzy`. The old log does not prove the same failure persists in the new launch. |
| `rg` was not installed | The shell returned `rg: command not found`. | Use `grep -RInE` for these source searches. |

## Start with one clean workspace

Stop the previous Panda and FR3 launch processes with `Ctrl+C`. In a new terminal:

```bash
bash --noprofile --norc
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source /home/yujietang/panda_franka_robot/install/local_setup.bash

# No output means the old workspace is absent from AMENT_PREFIX_PATH.
printf '%s\n' "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep 'Franka-Panda-Robot-Project'

ros2 pkg prefix panda_bringup
ros2 pkg prefix panda_moveit
ros2 pkg prefix panda_controller
ros2 pkg prefix panda_description
ros2 pkg prefix panda_nmpc
ros2 pkg prefix gz_ros2_control
```

The Panda packages should resolve under `/home/yujietang/panda_franka_robot/install/...`; `gz_ros2_control` should resolve under `/opt/ros/jazzy`. During this session, the new paths were confirmed for `panda_bringup`, `panda_moveit`, and `panda_description`, and the Jazzy path for `gz_ros2_control`.

Compare the controller launch source with the installed copy:

```bash
cd /home/yujietang/panda_franka_robot
diff -u \
  src/panda_controller/launch/controller.launch.xml \
  install/panda_controller/share/panda_controller/launch/controller.launch.xml
```

If they differ, rebuild in the clean environment:

```bash
colcon build --symlink-install --packages-select panda_controller panda_bringup
source install/local_setup.bash
```

## Next experiment

1. Launch only one simulation stack: `ros2 launch panda_bringup pick_and_place.launch.xml`. Check that the new log does not start an extra standalone `ros2_control_node`.
2. In a terminal with the same ROS environment, run `ros2 control list_controllers`. Verify that `joint_state_broadcaster` and `arm_controller` are `active`, and that `/joint_states` keeps updating.
3. Set a small goal in RViz and click **Plan** first. Confirm the planned trajectory contains seven Panda joints and multiple timed points.
4. Publish the MoveIt result's `RobotTrajectory.joint_trajectory` to `/panda_nmpc/reference_trajectory`. The current node can then observe reference versus measured position.
5. For actual NMPC tracking, implement the optimizer, valid velocity state, constraints, solver failure handling, and a simulation command interface. Do not let MoveIt execution and NMPC command the same arm at the same time.

Earlier `/joint_states` messages contained `.nan` velocities. This did not affect the current position-only diagnostic. An NMPC state `x=[q, dq]` needs valid velocity measurements or an estimator.
