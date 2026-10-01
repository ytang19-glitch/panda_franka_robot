# panda_nmpc — terminal guide

This package connects measured Panda joints to a planned joint trajectory. **The current CasADi optimizer is read-only: it computes proposed velocities and tracking error, but sends no robot commands.**

## What is implemented?

| Executable | Behavior | Needs CasADi? |
| --- | --- | --- |
| `reference_bridge` | Forwards a validated MoveIt arm plan to `/panda_nmpc/reference_trajectory` | No |
| `nmpc_node` | Reads measured joints and the reference; solves joint-space MPC and logs its proposed velocity | Yes |

The current repository includes the CasADi implementation in
[`panda_nmpc/nmpc_node.py`](panda_nmpc/nmpc_node.py). Its model is
`q[k+1] = q[k] + dt * u[k]`, where `u` is joint velocity: this is **linear
kinematic MPC**, not yet nonlinear robot-dynamics NMPC.

The bridge and optimizer are separate ROS nodes. Starting the bridge does not
replace the optimizer or change its Terminal 3 command.

## Terminal responsibilities

Start Terminal 1, then Terminal 3, then Terminal 2. Use Terminal 4 to check readiness before making a new RViz plan.

| Terminal | Run | Leave running? |
| --- | --- | --- |
| 1 | Gazebo + controllers + MoveIt + RViz + color detection | Yes |
| 2 | Planned-trajectory reference bridge | Yes |
| 3 | Read-only CasADi optimizer | Yes |
| 4 | Clock, topic, node, and controller checks | Available for commands |

**Do not separately launch `panda_moveit moveit.launch.py`.** [`pick_and_place.launch.xml`](../panda_bringup/launch/pick_and_place.launch.xml) already includes it. A second launch creates duplicate `/move_group` and `/rviz2` nodes.

## Prepare once

For a new checkout, build the workspace in a terminal with the ROS environment loaded:

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

After editing only the `panda_nmpc` source, rebuild that package and restart its node:

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
colcon build --packages-select panda_nmpc --symlink-install
source install/setup.bash
```

Every terminal must use the same `ROS_DOMAIN_ID` and compatible middleware settings. Check the domain with:

```bash
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
```

Keep the domain already used by your running simulation. If you change it, restart the whole stack with that domain in every terminal.

### CasADi environment — required for the optimizer

The current `nmpc_node` imports CasADi; the bridge does not. Install CasADi once using a virtual environment built from Ubuntu's system Python:

```bash
sudo apt install python3-venv
/usr/bin/python3 -m venv --system-site-packages ~/venvs/panda_nmpc
~/venvs/panda_nmpc/bin/python -m pip install casadi

source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
~/venvs/panda_nmpc/bin/python -c \
"import casadi, rclpy; print('CasADi:', casadi.__version__); print('ROS import OK')"
```

`--system-site-packages` makes system Python packages available, and sourcing ROS exposes ROS packages. Calling the virtual environment's Python explicitly avoids relying on the interpreter recorded in a previously built ROS executable.

## Terminal 1 — start the complete simulation

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch panda_bringup pick_and_place.launch.xml
```

This launch already starts Gazebo, the robot controllers, MoveIt, RViz, and the color detector. The included MoveIt launch receives `is_sim=True`; the top-level XML currently does not declare an `is_sim` launch argument.

Leave this terminal running. Ensure Gazebo is unpaused. In Terminal 4, wait until `joint_state_broadcaster` and `arm_controller` are active and `/joint_states` is arriving.

## Terminal 3 — start ONE optimizer

Run this command once and leave it running:

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash

~/venvs/panda_nmpc/bin/python \
install/panda_nmpc/lib/panda_nmpc/nmpc_node \
--ros-args -p use_sim_time:=true
```

Use the virtual environment's Python explicitly so this executable can import
CasADi. The bridge's executable in Terminal 2 does not require this environment.
Do not start a second `nmpc_node` alongside this one.

Expected startup:

```text
Joint-space optimizer started in read-only mode; it does not send robot commands
```

The node can be quiet until a reference arrives. It uses ROS time to advance the
reference when `use_sim_time=true`, and monotonic wall time to measure feedback
age and solver duration.

**After `Reference trajectory finished`, do not restart the optimizer.** That
message means the current reference ended. The ROS node stays alive and waits
for another reference from the bridge. Click **Plan** again in RViz to start the
next input test.

If your shell prompt returns without pressing Ctrl+C, the process has exited.
Inspect any preceding traceback and check `ros2 node list` in Terminal 4 before
restarting it. Rebuilding or changing the source also requires a node restart.

## Terminal 2 — connect MoveIt plans to the reference topic

The installed `reference_bridge` executable extracts a Panda arm
`JointTrajectory` from a new MoveIt `DisplayTrajectory` and publishes it on
`/panda_nmpc/reference_trajectory`. Its source is
[`panda_nmpc/bridge.py`](panda_nmpc/bridge.py).

After updating this package, build it once to register the executable:

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select panda_nmpc --symlink-install
source install/setup.bash
ros2 pkg executables panda_nmpc
```

The executable list should include `nmpc_node` and `reference_bridge`.

Then start the bridge in Terminal 2:

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run panda_nmpc reference_bridge
```

Leave it running. Stop the earlier temporary Python bridge before starting
this executable, so only one bridge publishes references. CasADi is not required
for this bridge.

Expected startup:

```text
Bridge ready: /display_planned_path -> /panda_nmpc/reference_trajectory.
Create a NEW plan in RViz; no robot commands are sent.
```

The bridge validates the seven Panda arm joint names, finite positions, and
strictly increasing nonnegative trajectory times. Empty, multi-segment,
gripper-only, and malformed trajectories are rejected. This bridge supports one
arm trajectory per display message; it does not concatenate segments or
synchronize execution.

If your planned-path topic is namespaced, find its full name with
`ros2 topic list -t`, then set the topic parameter:

```bash
ros2 run panda_nmpc reference_bridge --ros-args \
-p display_topic:=/YOUR_NAMESPACE/display_planned_path
```

The output topic is also configurable with `-p reference_topic:=...`.
Match it to the tracker's `reference_topic` parameter.

## Terminal 4 — check readiness and data

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 control list_controllers -c /controller_manager
ros2 topic info /clock
ros2 topic echo /joint_states --once
ros2 node list
ros2 node info /panda_nmpc

ros2 topic info /display_planned_path
ros2 topic info /panda_nmpc/reference_trajectory
```

Expected:

- `joint_state_broadcaster` and `arm_controller` are active.
- `/clock` has a publisher and Gazebo time advances.
- `/joint_states` includes `panda_joint1` through `panda_joint7`.
- There is one `/panda_nmpc` node and one instance of MoveIt/RViz.
- `/display_planned_path` has type `moveit_msgs/msg/DisplayTrajectory`.
- With bridge and tracker running, the reference topic normally has one publisher and one subscriber.

The current repository's RViz launch does not explicitly set `use_sim_time`. Check and set it for this simulation session:

```bash
ros2 param get /rviz2 use_sim_time
ros2 param set /rviz2 use_sim_time true
ros2 param get /move_group use_sim_time
ros2 param get /panda_nmpc use_sim_time
```

The RViz setting above is a runtime change and must be repeated after restarting RViz unless its launch file is updated.

## Make a new plan and inspect the result

1. Keep Terminals 1, 2, and 3 running.
2. In RViz, select the arm planning group and set the start state to **Current**.
3. Move the goal marker to a nearby reachable pose.
4. Click **Plan** after the bridge has started.
5. Watch Terminal 2 for `Forwarded ... trajectory points`.
6. Watch Terminal 3 for `Accepted reference trajectory (...)` and tracking logs.

For the current CasADi optimizer, expect:

```text
Joint-space optimizer started in read-only mode; it does not send robot commands
Accepted reference trajectory (... s)
t=... s | error vector [rad]=... | error norm=... rad |
optimized first velocity [rad/s]=... | solve=... ms
Reference trajectory finished
```

At the reference duration, the optimizer stops processing that reference and waits for the next one. Leave both the bridge and optimizer running between plans.

**This is a plan-to-optimizer input test.** The reference timer starts on receipt of the plan, before robot execution. If the robot stays still while the reference advances, measured error can increase. Clicking **Execute** later uses the existing MoveIt/controller execution path; it does not apply MPC output or fix the timer alignment. These logs do not establish closed-loop MPC tracking performance.

### What the successful local test showed

The 2026-09-29 test forwarded 19 points with a 1.71 s duration. Reported solver times were 9.3–38.9 ms. Proposed velocities reached the configured `±0.4 rad/s` bounds. This confirms reference delivery and optimizer operation; it does not demonstrate 1 kHz control or that the robot followed MPC commands.

## Troubleshooting

| Symptom | Check / action |
| --- | --- |
| `No module named 'casadi'` | Complete the CasADi setup and start the optimizer with the virtual environment's Python. A successful colcon build does not prove runtime imports are available. |
| Reference publisher count is zero | Start Terminal 2. Planning in RViz alone does not populate the custom reference topic. |
| Bridge is ready but forwards nothing | Check the topic name/type with `ros2 topic list -t`; set `display_topic` if needed and click **Plan** again after bridge startup. |
| Bridge forwards but tracker stays quiet | Check Terminal 3 for rejection/solver errors; verify `/joint_states`, ROS domain, and advancing `/clock` for the optimizer. |
| Duplicate `/move_group` or `/rviz2` | Stop the extra standalone MoveIt launch with Ctrl+C in its terminal. Keep Terminal 1 running. |
| Two `/joint_states` publishers | Run `ros2 topic info -v /joint_states` to identify them. Check whether their messages conflict; the count alone does not identify the cause. |
| `RTPS_TRANSPORT_SHM ... open_and_lock_file failed` | Check whether feedback and references still arrive. They did in the successful test. If communication fails, investigate middleware/process configuration; do not assume these messages caused solver failure. |

For feedback-rate checks, run the following and press Ctrl+C to finish:

```bash
ros2 topic hz /joint_states
```

## Which files to focus on

| File | Purpose |
| --- | --- |
| [`panda_nmpc/bridge.py`](panda_nmpc/bridge.py) | Installed `reference_bridge` executable: MoveIt display plan to validated tracker reference; no commands. |
| [`panda_nmpc/nmpc_node.py`](panda_nmpc/nmpc_node.py) | Feedback/reference subscriptions, joint-space MPC solver, and read-only tracking loop. |
| [`panda_nmpc/reference_trajectory.py`](panda_nmpc/reference_trajectory.py) | Validates exactly seven Panda joints and interpolates timed positions. |
| [`panda_nmpc/safety.py`](panda_nmpc/safety.py) | Joint ordering and error calculation. |
| [`../panda_commander/src/panda_commander.cpp`](../panda_commander/src/panda_commander.cpp) | Existing MoveIt planning/execution; future place to publish a reference aligned with execution. |
| [`panda_nmpc/robot_model.py`](panda_nmpc/robot_model.py), [`panda_nmpc/optimizer.py`](panda_nmpc/optimizer.py) | Future dynamics model and NMPC implementation. |
| [`config/nmpc.yaml`](config/nmpc.yaml), [`launch/nmpc_sim.launch.py`](launch/nmpc_sim.launch.py) | Package settings and alternate launch; do not start it alongside another tracker instance. |
| [`../panda_controller/config/panda_controllers.yaml`](../panda_controller/config/panda_controllers.yaml) | Existing arm controller configuration: position command interface. |

## Next development step

First align reference timing with actual execution. Then implement a simulation-only MPC command path with one owner of arm commands. The configured arm controller accepts positions, while the optimizer returns velocities: a bounded position target can be formed as `q_cmd = q_measured + dt * u0_star`, but requires a properly timed controller command path, joint-limit checks, and measured feedback before claiming closed-loop control.

Verify the running interfaces before implementing that path:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 param get /arm_controller command_interfaces
ros2 topic info /arm_controller/joint_trajectory
```

For true dynamics-based NMPC, replace the simple position integrator with a robot state/dynamics model and the intended physical control input (for example, torque). Keep model prediction, controller interfaces, and experiment timing consistent.

## Stop the session

Press Ctrl+C in Terminals 2 and 3, then in Terminal 1. Do not start a second full stack while the first is still running.

See also [ROS-domain and process debugging](../../docs/Panda_ROS_Domain_and_NMPC_Debugging_2026-09-27.md).
