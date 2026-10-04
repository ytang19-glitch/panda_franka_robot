# RViz Cartesian path troubleshooting

This page explains why the Cartesian path in RViz can **swing between two positions**, why it can **disappear after `pkill`**, and how to cleanly restart the reference generator.

## Symptom: the green line swings up and down

The most common cause is that **more than one `reference_generator` process is publishing to the same topic**:

```text
/panda_reference/cartesian_path
```

For example, an old process may still publish the default path at `z = 0.35`, while a new process publishes a path at `z = 0.55`. RViz receives messages from both publishers and alternates between the two paths, so the line appears to swing.

A second possible cause is inconsistent frame or time data. Every `nav_msgs/msg/Path` message and every pose should use the same valid frame, normally `panda_link0`, and all simulation nodes should use `use_sim_time:=true`.

Check how many publishers exist:

```bash
ros2 topic info /panda_reference/cartesian_path
ros2 topic info /panda_reference/cartesian_path --verbose
```

The desired result is:

```text
Publisher count: 1
```

List all matching operating-system processes:

```bash
pgrep -af reference_generator
```

List matching ROS nodes:

```bash
ros2 node list | grep reference
```

If the same node name appears more than once, or the topic has more than one publisher, an older generator is probably still running.

## Why the path disappears after `pkill`

This command sends SIGINT to every matching generator process:

```bash
pkill -INT -f 'install/panda_reference_trajectories/lib/panda_reference_trajectories/reference_generator'
```

That process is the publisher of `/panda_reference/cartesian_path`. After it stops:

- the topic has no active publisher;
- no new `nav_msgs/msg/Path` message is produced;
- the normal volatile ROS 2 QoS does not save the last path for a new subscriber;
- RViz can clear the display when it resets, reconnects, changes topic, or receives an empty replacement path.

Therefore, disappearance after `pkill` is expected. The command does not damage the path code; it only stops the node that supplies the visualization.

RViz may sometimes keep an already received line on screen until the display is reset. Do not use the visible line alone to decide whether the generator is running—check the topic publisher count.

## Clean only old reference-generator processes

Open a terminal and source ROS 2 and the workspace:

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Inspect the current processes before stopping anything:

```bash
pgrep -af reference_generator
ros2 node list | grep reference
ros2 topic info /panda_reference/cartesian_path
```

Stop all old instances of this generator:

```bash
pkill -INT -f 'install/panda_reference_trajectories/lib/panda_reference_trajectories/reference_generator'
```

Confirm that they stopped:

```bash
pgrep -af reference_generator || echo "No reference_generator process is running"
ros2 topic info /panda_reference/cartesian_path
```

It is normal for the last command to report `Unknown topic` after the only publisher has stopped.

If a process ignores SIGINT, find its PID and terminate only that PID:

```bash
pgrep -af reference_generator
kill -TERM <PID>
```

Use `kill -KILL <PID>` only as a last resort.

## Restart exactly one generator

Run one generator and keep this terminal open:

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run panda_reference_trajectories reference_generator \
  --ros-args \
  -r __node:=cartesian_reference_generator \
  -p use_sim_time:=true \
  -p frame_id:=panda_link0 \
  -p start_x:=0.45 \
  -p start_y:=-0.25 \
  -p start_z:=0.55 \
  -p goal_x:=0.45 \
  -p goal_y:=0.25 \
  -p goal_z:=0.55 \
  -p step_size:=0.005
```

Do not run this command again in another terminal while the first copy is active.

## Verify the restarted path

In a new terminal:

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic info /panda_reference/cartesian_path
ros2 topic echo /panda_reference/cartesian_path --once
```

Verify all of the following:

- `Publisher count: 1`;
- `header.frame_id` is `panda_link0`;
- the path uses `x = 0.45` and `z = 0.55`;
- the Y coordinate progresses from approximately `-0.25` to `0.25`;
- the path does not alternate between two sets of coordinates.

Check the active parameters:

```bash
ros2 param get /cartesian_reference_generator frame_id
ros2 param get /cartesian_reference_generator start_z
ros2 param get /cartesian_reference_generator goal_z
ros2 param get /cartesian_reference_generator use_sim_time
```

Expected values are `panda_link0`, `0.55`, `0.55`, and `True`.

## RViz settings

In RViz:

1. Set **Fixed Frame** to `panda_link0`.
2. Add a **Path** display.
3. Select `/panda_reference/cartesian_path`.
4. Use `Lines` and a line width such as `0.02`.
5. Temporarily hide the MoveIt MotionPlanning path if its green trajectory makes the two displays difficult to distinguish.

The path display is only a Cartesian reference visualization. It does not move the Panda robot. Robot motion requires a separate MoveIt trajectory builder/executor or a controller command publisher.

## Optional ROS graph refresh

Use this only if stopped nodes remain visible in ROS 2 CLI results:

```bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

Restarting the ROS daemon refreshes CLI graph discovery; it does not stop Gazebo, RViz, MoveIt, or the robot controllers.

## Relevant cleanup commands

| Purpose | Command |
|---|---|
| Find generator processes | `pgrep -af reference_generator` |
| Find reference ROS nodes | `ros2 node list \| grep reference` |
| Count path publishers | `ros2 topic info /panda_reference/cartesian_path` |
| Show publisher details | `ros2 topic info /panda_reference/cartesian_path --verbose` |
| Stop installed generators cleanly | `pkill -INT -f 'install/panda_reference_trajectories/lib/panda_reference_trajectories/reference_generator'` |
| Stop one known process | `kill -TERM <PID>` |
| Refresh ROS CLI discovery | `ros2 daemon stop && ros2 daemon start` |
| Verify one path message | `ros2 topic echo /panda_reference/cartesian_path --once` |

The normal clean-start rule is: **stop old generators, confirm zero publishers, start one generator, then confirm exactly one publisher**.
