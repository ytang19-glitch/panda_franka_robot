# Panda MoveIt RViz drag handle troubleshooting

## Current result

In the latest RViz screenshot, **Global Status: Ok**, **MotionPlanning Status: Ok**, and the drag handle is visible. The working setting under **Global Options → Fixed Frame** is `panda_link0`.

The **Views → Current View → Target Frame: `world`** field at the bottom of the screenshot controls the camera view. It is different from **Fixed Frame**, which RViz uses to transform and draw data.

## Which Fixed Frame should I use?

| Goal | Fixed Frame | Requirement |
| --- | --- | --- |
| Work with the Panda arm relative to its base | `panda_link0` | The base frame is present in TF. This is the working choice shown in your screenshot. |
| Display a Gazebo world or objects defined in world coordinates | `world` | TF must connect `world` and `panda_link0`. |

The earlier **Fixed Frame: Frame [world] does not exist** error meant RViz could not use `world` at that point. Selecting `panda_link0` removed that display error. You can select `world` when a valid transform to the base is available. An identity `world → panda_link0` transform is appropriate only if the base really is at the world origin; do not publish duplicate transforms for the same child frame.

To keep the choice across restarts, save the RViz configuration loaded by your launch: `src/panda_moveit/rviz/moveit.rviz`. In RViz, use **File → Save Config** and make sure the source file is updated, rather than only a copy under `install/`.

## What caused the missing handle?

These were separate issues observed during debugging:

1. **RViz initially lacked the arm kinematics solver.** The `arm` KDL entry was present in `src/panda_moveit/config/kinematics.yaml`, but RViz had not received `robot_description_kinematics`. Its parameter initially returned an empty string. After passing `moveit_config.robot_description_kinematics` to RViz, the parameter returned `kdl_kinematics_plugin/KDLKinematicsPlugin`, and the RViz log reported arm joint weights.
2. **The initial Fixed Frame, `world`, was missing.** This caused the red Fixed Frame status. The latest screenshot shows green status and a visible handle with Fixed Frame `panda_link0`.
3. **RViz reported `use_sim_time: False` while `move_group` used `is_sim:=true`.** When Gazebo publishes `/clock`, different clocks can cause timestamp and TF trouble. However, the available evidence does **not** establish that this mismatch caused the handle to disappear: the latest screenshot shows the handle. Aligning the clocks is still appropriate when running Gazebo.
4. **The marker topic had one publisher and one subscriber.** Duplicate marker publishers were unlikely on the observed topic. This count alone cannot prove a usable marker was created.

## Which file should change?

| Purpose | File or UI | Action |
| --- | --- | --- |
| Set RViz Fixed Frame | **Global Options → Fixed Frame**, stored in `src/panda_moveit/rviz/moveit.rviz` | Use `panda_link0` for the working base frame view; use `world` only with valid TF. |
| Pass kinematics and simulation time to RViz | `src/panda_moveit/launch/moveit.launch.py` | Include the four RViz parameters below. |
| Define the arm IK solver | `src/panda_moveit/config/kinematics.yaml` | Keep the `arm` KDL entry. A non-chain `gripper` does not need a KDL chain solver. |

In `src/panda_moveit/launch/moveit.launch.py`, set the **RViz node's** parameter list to:

```python
parameters=[
    moveit_config.robot_description,
    moveit_config.robot_description_semantic,
    moveit_config.robot_description_kinematics,
    {"use_sim_time": is_sim},
],
```

`is_sim` is already declared in the launch file and defaults to `true`. Use `is_sim:=true` when Gazebo publishes `/clock`. For a real robot without a simulation clock, use `is_sim:=false` and check the real robot's TF frames; the Panda simulation frames do not automatically apply to the FR3.

## Rebuild and verify

Run each command separately and press Enter after each one:

```bash
cd ~/panda_franka_robot
```

```bash
colcon build --symlink-install --packages-select panda_moveit
```

```bash
source install/setup.bash
```

Stop the previous Panda launch and restart it with Gazebo running:

```bash
ros2 launch panda_moveit moveit.launch.py is_sim:=true
```

In another terminal, check each parameter:

```bash
ros2 param get /rviz2 use_sim_time
```

Expected with Gazebo: `Boolean value is: True`.

```bash
ros2 param get /rviz2 robot_description_kinematics.arm.kinematics_solver
```

Expected: `String value is: kdl_kinematics_plugin/KDLKinematicsPlugin`.

```bash
ros2 topic info /clock
```

If `use_sim_time` is `True`, check that `/clock` has a publisher. In RViz, check **Global Status: Ok**, select **Interact**, choose planning group **arm**, and enable **MotionPlanning → Planning Request → Query Goal State**. The goal drag handle should appear at the arm's end effector.

## References

- [MoveIt: Quickstart in RViz](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html): Fixed Frame, Query Goal State, and Interact.
- [ROS 2: Understanding parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html): node parameters, including `use_sim_time`.
- [ROS 2: Webots simulation supervisor](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Simulation-Supervisor.html): an example of a simulator publishing `/clock` for nodes using simulation time.
