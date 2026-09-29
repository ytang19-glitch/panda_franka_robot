# panda_nmpc — terminal guide

This package connects measured Panda joints to a planned joint trajectory. **The tracker and the local optimizer described below are read-only: neither sends robot commands.**

## What is implemented?

| Version | Behavior | Expected startup message |
| --- | --- | --- |
| Current repository `nmpc_node.py` | Interpolates a reference and logs measured joint-position error; no solver | `Read-only tracking node started; no commands are sent` |
| Local CasADi prototype tested on 2026-09-29 | Also solves a finite-horizon joint-space MPC problem and logs proposed joint velocities | `Joint-space optimizer started in read-only mode; it does not send robot commands` |

The local CasADi implementation is not included by this README update. Your installed source determines which version runs. The local model is `q[k+1] = q[k] + dt * u[k]`, where `u` is joint velocity: this is **linear kinematic MPC**, not yet nonlinear robot-dynamics NMPC.

## Terminal responsibilities

Start Terminal 1, then Terminal 3, then Terminal 2. Use Terminal 4 to check readiness before making a new RViz plan.

| Terminal | Run | Leave running? |
| --- | --- | --- |
| 1 | Gazebo + controllers + MoveIt + RViz + color detection | Yes |
| 2 | Planned-trajectory reference bridge | Yes |
| 3 | Read-only tracker or local CasADi optimizer | Yes |
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

### CasADi environment — only for the local optimizer

The repository's diagnostics-only node does not require CasADi. For the local optimizer, install it once using a virtual environment built from Ubuntu's system Python:

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

## Terminal 3 — start ONE tracker/optimizer

Source the environment:

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**For your local CasADi optimizer**, run:

```bash
~/venvs/panda_nmpc/bin/python \
install/panda_nmpc/lib/panda_nmpc/nmpc_node \
--ros-args -p use_sim_time:=true
```

**For the current repository's diagnostics-only node**, use this instead:

```bash
ros2 run panda_nmpc nmpc_node --ros-args -p use_sim_time:=true
```

Run only one of these commands. Leave it running. After the startup message, the node may be quiet until a reference arrives; that is expected.

The current repository node samples its reference using Python's monotonic wall clock. The tested local optimizer uses ROS time for reference progression. Setting `use_sim_time` alone does not change explicit `time.monotonic()` calculations in the older code.

## Terminal 2 — connect MoveIt plans to the reference topic

The bridge below extracts a Panda arm `JointTrajectory` from a MoveIt `DisplayTrajectory` and publishes it on `/panda_nmpc/reference_trajectory`.

Paste the entire block into Terminal 2. It runs directly; no new source file or rebuild is needed. The system Python is sufficient for this bridge because it does not use CasADi.

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash

/usr/bin/python3 - <<'PY'
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory


class ReferenceBridge(Node):
    def __init__(self):
        super().__init__("moveit_reference_bridge")
        self.required_joints = {
            f"panda_joint{i}" for i in range(1, 8)
        }
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/panda_nmpc/reference_trajectory",
            10,
        )
        # Reliable, volatile subscription: receive NEW plans after startup.
        self.subscription = self.create_subscription(
            DisplayTrajectory,
            "/display_planned_path",
            self.forward_reference,
            10,
        )
        self.get_logger().info(
            "Bridge ready. Create a NEW plan in RViz."
        )

    def forward_reference(self, msg):
        for robot_trajectory in msg.trajectory:
            trajectory = robot_trajectory.joint_trajectory
            if (
                len(trajectory.joint_names) != 7
                or set(trajectory.joint_names) != self.required_joints
            ):
                continue
            if len(trajectory.points) < 2:
                continue
            end = trajectory.points[-1].time_from_start
            if end.sec + end.nanosec * 1e-9 <= 0.0:
                self.get_logger().warning(
                    "Skipping trajectory without positive duration"
                )
                continue
            self.publisher.publish(trajectory)
            self.get_logger().info(
                f"Forwarded {len(trajectory.points)} trajectory points"
            )
            return
        self.get_logger().warning(
            "No suitable timed seven-joint Panda arm trajectory found"
        )


rclpy.init()
node = ReferenceBridge()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
```

Leave it running. This test bridge forwards the first suitable arm trajectory from each display message; it does not concatenate multiple trajectory segments or synchronize execution.

If your planned-path topic is namespaced, find its full name with `ros2 topic list -t` and replace `/display_planned_path` in the script.

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

For the repository tracker, expect:

```text
Read-only tracking node started; no commands are sent
Accepted reference trajectory (... s)
t=... s, joint position error=... rad
```

For the local CasADi optimizer, expect:

```text
Joint-space optimizer started in read-only mode; it does not send robot commands
Accepted reference trajectory (... s)
t=... s | error vector [rad]=... | error norm=... rad |
optimized first velocity [rad/s]=... | solve=... ms
Reference trajectory finished
```

The tested local optimizer stops sampling at the reference duration. The older repository tracker continues reporting the final reference error after that duration.

**This is a plan-to-optimizer input test.** The reference timer starts on receipt of the plan, before robot execution. If the robot stays still while the reference advances, measured error can increase. Clicking **Execute** later uses the existing MoveIt/controller execution path; it does not apply MPC output or fix the timer alignment. These logs do not establish closed-loop MPC tracking performance.

### What the successful local test showed

The 2026-09-29 test forwarded 19 points with a 1.71 s duration. Reported solver times were 9.3–38.9 ms. Proposed velocities reached the configured `±0.4 rad/s` bounds. This confirms reference delivery and optimizer operation; it does not demonstrate 1 kHz control or that the robot followed MPC commands.

## Troubleshooting

| Symptom | Check / action |
| --- | --- |
| `No module named 'casadi'` | Complete the optional CasADi setup and start the local optimizer with the virtual environment's Python. A successful colcon build does not prove runtime imports are available. |
| Reference publisher count is zero | Start Terminal 2. Planning in RViz alone does not populate the custom reference topic. |
| Bridge is ready but forwards nothing | Check the topic name/type with `ros2 topic list -t`; click **Plan** again after bridge startup. |
| Bridge forwards but tracker stays quiet | Check Terminal 3 for rejection/solver errors; verify `/joint_states`, ROS domain, and advancing `/clock` for the local optimizer. |
| Duplicate `/move_group` or `/rviz2` | Stop the extra standalone MoveIt launch with Ctrl+C in its terminal. Keep Terminal 1 running. |
| Two `/joint_states` publishers | Run `ros2 topic info -v /joint_states` to identify them. Check whether their messages conflict; the count alone does not identify the cause. |
| `RTPS_TRANSPORT_SHM ... open_and_lock_file failed` | Check whether feedback and references still arrive. They did in the successful test. If communication fails, investigate middleware/process configuration; do not assume these messages caused solver failure. |
| `AttributeError` from `first_velocity.full()` | In the local solver use `return ca.DM(first_velocity).full().flatten().tolist()` after `solution.value(...)`. Rebuild and restart after the source edit. |

For feedback-rate checks, run the following and press Ctrl+C to finish:

```bash
ros2 topic hz /joint_states
```

## Which files to focus on

| File | Purpose |
| --- | --- |
| [`panda_nmpc/nmpc_node.py`](panda_nmpc/nmpc_node.py) | Feedback/reference subscriptions and tracking loop; the local prototype also implements the optimizer here. |
| [`panda_nmpc/reference_trajectory.py`](panda_nmpc/reference_trajectory.py) | Validates exactly seven Panda joints and interpolates timed positions. |
| [`panda_nmpc/safety.py`](panda_nmpc/safety.py) | Joint ordering and error calculation. |
| [`../panda_commander/src/panda_commander.cpp`](../panda_commander/src/panda_commander.cpp) | Existing MoveIt planning/execution; future place to publish a reference aligned with execution. |
| [`panda_nmpc/robot_model.py`](panda_nmpc/robot_model.py), [`panda_nmpc/optimizer.py`](panda_nmpc/optimizer.py) | Future dynamics model and NMPC implementation. |
| [`config/nmpc.yaml`](config/nmpc.yaml), [`launch/nmpc_sim.launch.py`](launch/nmpc_sim.launch.py) | Package settings and alternate launch; do not start it alongside another tracker instance. |
| [`../panda_controller/config/panda_controllers.yaml`](../panda_controller/config/panda_controllers.yaml) | Existing arm controller configuration: position command interface. |

## Next development step

First align reference timing with actual execution. Then implement a simulation-only MPC command path with one owner of arm commands. The configured arm controller accepts positions, while the local optimizer returns velocities: a bounded position target can be formed as `q_cmd = q_measured + dt * u0_star`, but requires a properly timed controller command path, joint-limit checks, and measured feedback before claiming closed-loop control.

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
