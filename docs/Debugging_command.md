# ROS 2 / Gazebo Process Debugging

## Check whether old simulation processes are still running

Before restarting the Panda simulation, check whether old ROS 2, Gazebo, MoveIt, or RViz processes are still alive:

```bash
pgrep -af 'gz sim|rviz2|move_group|ros2 launch|robot_state_publisher'
```

### What this command does

- `pgrep` searches the currently running processes.
- `-a` prints the full command line.
- `-f` matches against the full command line instead of only the executable name.
- The expression after it searches for several common robotics processes:
  - `gz sim` — Gazebo simulation
  - `rviz2` — RViz
  - `move_group` — MoveIt planning server
  - `ros2 launch` — ROS 2 launch processes
  - `robot_state_publisher` — publishes the robot TF tree

Example output:

```text
12345 gz sim world.sdf
12390 /opt/ros/jazzy/lib/rviz2/rviz2
12410 /opt/ros/jazzy/lib/moveit_ros_move_group/move_group
```

If this command prints processes after you thought the simulation was closed, some old processes are still running.

If it prints nothing, the listed processes are no longer running.

---

## Stop stale processes

First try stopping the parent launch process cleanly with:

```bash
Ctrl+C
```

Then check again:

```bash
pgrep -af 'gz sim|rviz2|move_group|ros2 launch|robot_state_publisher'
```

If stale processes remain, inspect their PID and terminate them:

```bash
kill -INT <PID>
```

Example:

```bash
kill -INT 12345
```

If a process still refuses to exit, use:

```bash
kill -TERM <PID>
```

Use `kill -9` only as a last resort because it does not allow ROS nodes to shut down cleanly.

You can also check specific processes:

```bash
pgrep -af gz
pgrep -af rviz2
pgrep -af move_group
pgrep -af robot_state_publisher
pgrep -af controller_manager
```

A useful rule is:

```text
Stop simulation
      ↓
Check processes
      ↓
Kill stale processes
      ↓
Check again
      ↓
Start a clean simulation
```

---

## Why old processes can block the next simulation

A previous simulation may still own resources used by the new simulation, for example:

- ROS 2 node names
- controller manager instances
- DDS participants
- Gazebo resources
- TF publishers
- simulation clock publishers
- ROS topics and services

This can produce confusing symptoms such as:

- duplicated nodes
- RViz connecting to an old robot
- MoveIt using stale robot state
- controllers failing to start
- `/clock` behaving unexpectedly
- duplicated TF frames
- topics having unexpected publishers
- Gazebo appearing frozen or failing to launch correctly

---

## Why multiple ROS environments can cause problems

Another common problem is launching different terminals from different sourced environments.

For example, one terminal may use:

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
```

while another terminal may additionally source an older workspace:

```bash
source ~/old_robot_ws/install/setup.bash
```

ROS 2 uses environment variables such as:

```bash
AMENT_PREFIX_PATH
CMAKE_PREFIX_PATH
LD_LIBRARY_PATH
PYTHONPATH
ROS_DISTRO
ROS_DOMAIN_ID
RMW_IMPLEMENTATION
```

If different terminals contain different workspace overlays, ROS may load different versions of:

- packages
- launch files
- message definitions
- shared libraries
- controller plugins
- robot descriptions

This can make one terminal behave differently from another even when the command looks identical.

For simulation debugging, try to use the same environment in every terminal.

Example:

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
```

Then verify important environment variables:

```bash
echo $ROS_DISTRO
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
echo $AMENT_PREFIX_PATH
```

You can also check which package installation ROS is finding:

```bash
ros2 pkg prefix panda_bringup
ros2 pkg prefix panda_moveit
ros2 pkg prefix panda_nmpc
```

If these commands point to an unexpected workspace, the wrong environment is being sourced.

---

## Recommended clean restart procedure

When the simulation behaves strangely:

```bash
# 1. Stop launch terminals with Ctrl+C

# 2. Check for remaining processes
pgrep -af 'gz sim|rviz2|move_group|ros2 launch|robot_state_publisher|controller_manager'

# 3. Kill remaining stale processes if necessary
kill -INT <PID>

# 4. Confirm they are gone
pgrep -af 'gz sim|rviz2|move_group|ros2 launch|robot_state_publisher|controller_manager'

# 5. Open fresh terminals and source the SAME environment
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash

# 6. Verify package paths
ros2 pkg prefix panda_bringup
ros2 pkg prefix panda_moveit
ros2 pkg prefix panda_nmpc

# 7. Start the simulation again
```

The important idea is:

```text
Old processes + mixed ROS environments
                ↓
      duplicated/stale ROS state
                ↓
 simulation, MoveIt, TF, or controllers
       may behave inconsistently
```
