# Panda ROS 2 Domain and NMPC Debugging (2026-09-27)

Date: 2026-09-27 (Edmonton)  
Repository: `panda_franka_robot`  
Environment: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2

This guide records the observed Panda simulation problems while FR3 hardware processes were also running. The key debugging rule is to establish **which ROS domain, process, and controller manager** a command is inspecting before interpreting its output.

## Quick ROS 2 debugging checklist

Run these commands in a **new terminal** configured for Panda. Use the same `ROS_DOMAIN_ID` and source the same workspace in every Panda terminal, including the launch, CLI, commander, and `panda_nmpc` terminals.

```bash
export ROS_DOMAIN_ID=71
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 daemon stop
echo "$ROS_DOMAIN_ID"
ros2 node list
```

`71` is an example. Before starting Panda, `ros2 node list` should ideally be empty. If it is not, identify the nodes or choose an unused domain. A separate domain prevents ROS 2 discovery across domains; it does **not** stop the FR3 programs running elsewhere.

After starting **one** Panda simulation, run:

```bash
ros2 node list | sort | uniq -d
ros2 control list_controllers -c /controller_manager
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo panda_link0 camera_link
ros2 param get /move_group use_sim_time
ros2 param get /rviz2 use_sim_time
ros2 topic info /panda_nmpc/reference_trajectory -v
```

| Command | What to check |
| --- | --- |
| `echo "$ROS_DOMAIN_ID"` | Every Panda terminal has the same domain value. |
| `ros2 node list` | The current domain contains the expected Panda nodes; check for unexpected FR3 nodes. |
| `ros2 node list \| sort \| uniq -d` | Duplicate node names can make CLI results ambiguous. |
| `ros2 control list_controllers -c /controller_manager` | `arm_controller`, `gripper_controller`, and `joint_state_broadcaster` should be active in the isolated simulation. |
| `ros2 topic echo /joint_states --once` | Inspect `name` and `position` together; joint positions cannot be assigned by array index alone. |
| `ros2 run tf2_ros tf2_echo panda_link0 camera_link` | Confirm that the camera transform becomes available after startup. Stop the continuously running command with `Ctrl+C`. |
| `ros2 param get /move_group use_sim_time` and `ros2 param get /rviz2 use_sim_time` | Both should report `true` for Gazebo simulation. |
| `ros2 topic info /panda_nmpc/reference_trajectory -v` | Check whether a trajectory publisher actually exists and whether `panda_nmpc` subscribes. |

The commands above are **diagnostics**, not evidence that NMPC is controlling the robot.

## 1. What was observed

| Observation | Evidence | Interpretation and limit |
| --- | --- | --- |
| Duplicate `/rviz2` and `/controller_manager` nodes | `ros2 node list` warned about duplicate names. `pgrep -af` showed four FR3 MoveIt launch processes and one `panda_bringup` process. FR3 launch arguments included `use_fake_hardware:=false` and `robot_ip:=172.16.0.2`. | Multiple systems were running. Their individual domains were not all verified, so isolate Panda before trusting ROS graph results. |
| An old Panda simulation remained running | `pgrep -af` found Panda launch PID `2522909` and its Gazebo processes. | The previous launch had not fully exited. These PIDs were only a snapshot; never reuse them without checking the current process list. |
| One controller query did not show `arm_controller` | `ros2 control list_controllers -c /controller_manager` showed active `joint_state_broadcaster` and `gripper_controller` only. | The queried manager did not show an active arm controller at that time. Duplicate managers prevent a definite conclusion about the isolated Gazebo manager. |
| The detector initially reported disconnected TF trees | Logs said `panda_link0` and `camera_link` were not connected. Later `tf2_echo` repeatedly returned translation `[0.600, 0.000, 1.000]`, identity rotation, and time `0.0`. | The static camera transform eventually became available. A brief startup `Invalid frame ID` warning is not proof of a permanent TF failure; time zero is normal for a static transform. Recheck in one isolated simulation. |
| Eight joint positions were near zero | The pasted `/joint_states` values did not include the corresponding `name` array. | Near-zero positions match the repository's initial configuration, but the joint names must be checked before interpreting individual values. |
| `panda_nmpc` printed only its startup line | `Read-only tracking node started; no commands are sent`. | It had not received a reference trajectory. The current node does not solve NMPC or send commands. |
| Fast DDS reported shared-memory errors | `RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed` appeared, while `/joint_states` and TF were also received. | Shared-memory transport had a problem, but the logs alone do not prove all ROS communication failed or explain the TF/controller symptoms. |
| MoveIt reported execution success and RViz later crashed | The log said `arm_controller successfully finished` and `Completed trajectory execution with status SUCCEEDED`; subsequently `rviz2` exited with `-11`. | Successful trajectory execution and an RViz crash are separate observations. The `class_loader` unload warning has not been proven to cause the crash. |

## 2. Find and stop only the old Panda launch

Prefer `Ctrl+C` in the terminal that launched Panda. Do not use broad commands such as `pkill -f move_group`, `pkill -f rviz2`, or `killall ros2` while FR3 hardware processes are running. Do not send motion commands into a mixed ROS graph.

If the original Panda terminal is gone, inspect current processes:

```bash
pgrep -af 'ros2 launch panda_bringup|panda_franka_robot/install/panda_description'
ps -o pid,ppid,pgid,args -p 2522909,2522950,2522952
```

The `ps` PIDs above illustrate the recorded session. **Replace them with PIDs verified by the current `pgrep` output.** Only after confirming that a PID is still the Panda parent launch, send it `SIGINT` and check again:

```bash
kill -INT <verified_panda_launch_pid>
pgrep -af 'ros2 launch panda_bringup|panda_franka_robot/install/panda_description'
```

Replace the angle-bracket placeholder with the verified numeric PID; do not paste the placeholder literally. Do not terminate a process solely because its name is `gz sim server`: this machine had more than one Gazebo server. If `pgrep` prints its help text, correct the command syntax and rerun `pgrep -af '...'`; help output is not a process list.

## 3. Isolate Panda with `ROS_DOMAIN_ID`

In **every** Panda terminal:

```bash
export ROS_DOMAIN_ID=71
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 daemon stop
echo "$ROS_DOMAIN_ID"
ros2 node list
```

Use one free domain for this experiment. Setting the variable in a terminal affects processes started from that terminal; it does not retroactively move existing nodes. Restart Panda processes after configuring their terminal environments. The ROS 2 CLI also needs the same domain to discover them. The [ROS 2 domain documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html) explains discovery isolation.

## 4. Verify which installed launch file ROS 2 uses

The repository version of `src/panda_bringup/launch/pick_and_place_commander.launch.xml` launches only the commander and no longer activates controllers a second time. A screenshot nevertheless showed `ros2 control set_controller_state ... active`. Compare source with the installed package actually found by ROS:

```bash
cd ~/panda_franka_robot
ros2 pkg prefix --share panda_bringup
diff -u src/panda_bringup/launch/pick_and_place_commander.launch.xml \
  "$(ros2 pkg prefix --share panda_bringup)/launch/pick_and_place_commander.launch.xml"
```

If the files differ, rebuild and source the workspace again:

```bash
colcon build --symlink-install --packages-select panda_bringup
source install/setup.bash
ros2 pkg prefix --share panda_bringup
```

If the **source** file itself still contains a second activation command, inspect `git status --short` and local changes before editing it.

## 5. Start one Panda simulation and inspect the controller

In the configured Panda terminal:

```bash
ros2 launch panda_bringup pick_and_place.launch.xml
```

This launch starts Gazebo, controller spawners, MoveIt/RViz, and color detection. It sets the included MoveIt launch's `is_sim` to `True`; appending `is_sim:=false` to this outer command does not override that inner value and is inappropriate for this Gazebo test.

In a **second** Panda terminal, repeat the domain export and both `source` commands, then inspect:

```bash
ros2 node list | sort | uniq -d
ros2 control list_controllers -c /controller_manager
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo panda_link0 camera_link
ros2 param get /move_group use_sim_time
ros2 param get /rviz2 use_sim_time
```

Expected results: the three controllers are active; there is no unexpected duplicate `/controller_manager`; joint states and the camera transform arrive; MoveIt and RViz use simulation time. A transient `Invalid frame ID` when `tf2_echo` starts can resolve as static transforms arrive.

**Only after confirming there is one controller manager in this domain**, if `arm_controller` is still absent, try spawning it and inspect the full error if it fails:

```bash
ros2 run controller_manager spawner arm_controller \
  -c /controller_manager --controller-manager-timeout 120
ros2 control list_controllers -c /controller_manager
```

See the [ros2_control controller manager and spawner documentation](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html).

## 6. Check RViz time and the displayed robot state

In the current repository version, `panda_moveit/launch/moveit.launch.py` passes `use_sim_time` to `move_group`, but the RViz node's `parameters` list is missing it. Add the following entry to `rviz_node` parameters:

```python
{"use_sim_time": is_sim},
```

Then rebuild, source, and restart the simulation:

```bash
cd ~/panda_franka_robot
colcon build --symlink-install --packages-select panda_moveit
source install/setup.bash
```

Verify with `ros2 param get /rviz2 use_sim_time`. The time mismatch is a confirmed configuration inconsistency; it has **not** been proven to cause RViz exit code `-11`. If RViz still crashes after the clocks agree, investigate RViz and the graphics stack separately.

The SRDF `home` state and `panda_moveit/config/initial_positions.yaml` set zero joint positions; the latter describes fake ros2_control initialization. RViz may show both current and goal states, so overlapping colors alone do not mean two robots were loaded. Compare the `name` and `position` fields from `ros2 topic echo /joint_states --once`, then inspect RViz Start State and Goal State.

## 7. Debug the MoveIt-to-`panda_nmpc` reference path

The current `panda_nmpc/panda_nmpc/nmpc_node.py` subscribes to `/joint_states` and `/panda_nmpc/reference_trajectory`. It interpolates desired positions with `reference_trajectory.py` and reports the norm of the seven joint position errors. `optimizer.py` and `robot_model.py` are placeholders.

First plan and execute a small reachable `arm` motion in RViz to verify simulated controller execution. Pressing **Plan/Execute** in RViz does **not** publish a message to the `panda_nmpc` reference topic.

For a commander-driven experiment, publish `plan.trajectory_.joint_trajectory` in `panda_commander.cpp` from the successful `arm_` planning branch to `/panda_nmpc/reference_trajectory`, then execute the plan. Add `trajectory_msgs` to `panda_commander`'s `package.xml` and `CMakeLists.txt`. Ensure the published trajectory has the expected seven arm joint names.

Start the current **read-only** tracking node in the same Panda domain:

```bash
ros2 run panda_nmpc nmpc_node --ros-args -p use_sim_time:=true
```

In another Panda terminal, inspect the topic and its messages:

```bash
ros2 topic info /panda_nmpc/reference_trajectory -v
ros2 topic echo /panda_nmpc/reference_trajectory --once
ros2 topic echo /joint_states --once
```

`ros2 topic info -v` shows publishers, subscribers, and QoS. The `topic echo --once` call waits for a message; stop it with `Ctrl+C` if no reference is being published. With no reference, the node only reports `Read-only tracking node started; no commands are sent`. After a valid seven-joint reference arrives, look for `Accepted reference trajectory` and `joint position error=... rad`.

The current tracking code uses `time.monotonic()` beginning when it receives the trajectory. This is not precisely aligned with when the controller begins executing in simulation, so experiment-quality desired-versus-measured plots require a common execution time base.

A future NMPC controller must validate the prediction model in `robot_model.py`, then implement dynamics, constraints, and a solver in `optimizer.py`. When switching to NMPC execution, avoid simultaneously calling MoveIt `execute(plan)` and commanding the same arm controller from NMPC. Panda position-control simulation results do not establish FR3 hardware torque-control performance.

## 8. Still unverified

- No controller list from a clean, isolated Panda domain has yet confirmed that `arm_controller` is fixed.
- The root cause of Fast DDS shared-memory errors is unknown. Do not blindly clear `/dev/shm` while hardware nodes are running.
- The cause of the RViz `-11` crash is unknown, despite MoveIt reporting `SUCCEEDED` for one trajectory.
- No MoveIt trajectory was observed on the NMPC reference topic, and no NMPC solver controlled Panda in this session.
