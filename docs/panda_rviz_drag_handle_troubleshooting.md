# Panda MoveIt RViz drag handle: fixed frame, IK, and simulation time

## Current result / 当前结果

In the latest RViz screenshot, **Global Status: Ok**, **MotionPlanning Status: Ok**, and the colored drag handle is visible. The working **Global Options → Fixed Frame** is `panda_link0`.

最新 RViz 截图中，**Global Status: Ok**、**MotionPlanning Status: Ok**，并且已经能看到彩色拖动手柄。当前正常工作的 **Global Options → Fixed Frame** 是 `panda_link0`。

Do not confuse this with **Views → Current View → Target Frame: `world`** at the bottom of the screenshot. Target Frame controls the camera view; Fixed Frame is the coordinate frame RViz uses to draw the scene.

不要把截图底部 **Views → Current View → Target Frame: `world`** 当成 Fixed Frame。Target Frame 用于相机视角；Fixed Frame 用于 RViz 场景的坐标变换。

## Which Fixed Frame? / 选哪个 Fixed Frame？

| Situation / 场景 | RViz Fixed Frame / 固定坐标系 | Requirement / 条件 |
| --- | --- | --- |
| Work with the Panda arm in its base frame / 仅以 Panda 底座为参考 | `panda_link0` | It exists in TF. This is the working choice in the screenshot. / TF 中存在；最新截图已经验证。 |
| Show a Gazebo world or objects in world coordinates / 需要显示 Gazebo 世界和物体 | `world` | A valid TF connection between `world` and `panda_link0` must exist. / `world` 与 `panda_link0` 必须通过 TF 连通。 |

If `world` is selected without the required TF frame or connection, RViz can show **Fixed Frame: Frame [world] does not exist**. Selecting `panda_link0` resolves that particular display error when the Panda base TF exists. A `world → panda_link0` identity transform is correct only if the robot base really is at the world origin; avoid publishing a second transform for the same child frame.

如果选择了 `world`，但 TF 中没有所需的坐标系或连接，RViz 会显示 **Frame [world] does not exist**。当 Panda 底座 TF 存在时，改为 `panda_link0` 可解决这一显示错误。只有机器人底座确实在世界原点时，`world → panda_link0` 才应是单位变换；不要给同一个子坐标系重复发布 TF。

To keep the GUI choice for the next RViz launch, save the RViz configuration to the file loaded by the launch: `src/panda_moveit/rviz/moveit.rviz` (**File → Save Config**). If you changed the installed copy under `install/`, make the change in the source configuration instead.

若要下次启动仍保留 GUI 选择，把 RViz 配置保存至 launch 加载的 `src/panda_moveit/rviz/moveit.rviz`（**File → Save Config**）。如果修改的是 `install/` 下的副本，应把设置保存回源码配置。

## Why did the handle disappear? / 手柄为什么消失？

There were several separate conditions during debugging:

1. **RViz initially lacked arm IK configuration.** `ros2 param get /rviz2 robot_description_kinematics.arm.kinematics_solver` was empty. The `arm` KDL entry existed in `config/kinematics.yaml`, but the RViz node was not passed that parameter. Passing `moveit_config.robot_description_kinematics` to RViz fixed this: the parameter later returned `kdl_kinematics_plugin/KDLKinematicsPlugin` and the RViz log showed arm joint weights.
2. **`world` was initially missing as RViz's Fixed Frame.** This was a separate TF/display error. The current screenshot with Fixed Frame `panda_link0` shows green status and the handle.
3. **RViz reported `use_sim_time: False`, while `move_group` was launched with `is_sim:=true`.** When Gazebo publishes `/clock`, this makes the nodes use different notions of time and can cause timestamp/TF trouble. **We have not established that this mismatch alone made the handle disappear**: the latest screenshot shows the handle despite the earlier `False` reading. Aligning the clocks is still the correct setup for a Gazebo simulation.
4. **One marker publisher and one subscriber were present.** That result rules against duplicate marker publishers in the observed topic; it does not by itself prove that a usable handle was created.

调试过程中出现了几种**不同**的问题：

1. **RViz 一开始没有拿到机械臂逆运动学配置。** 虽然 `config/kinematics.yaml` 中有 `arm` 的 KDL 配置，但 RViz 节点没有接收该参数。把 `moveit_config.robot_description_kinematics` 传给 RViz 后，参数和日志都确认加载成功。
2. **RViz 最初把不存在的 `world` 设为 Fixed Frame。** 这是独立的 TF/显示问题。最新截图使用 `panda_link0`，状态为绿色，手柄也可见。
3. **RViz 的 `use_sim_time` 曾为 `False`，而 `move_group` 使用 `is_sim:=true`。** Gazebo 发布 `/clock` 时，两者时钟不一致可能带来时间戳与 TF 问题。**目前没有证据证明这就是手柄消失的唯一原因**；尽管此前检测到 `False`，最新截图仍显示手柄。仿真时保持时钟一致依然是正确配置。
4. **手柄话题有一个发布者、一个订阅者。** 这使重复发布者问题不太可能，但不代表手柄一定已正确生成。

## Which file should change? / 具体修改哪个文件？

| Goal / 目的 | File or UI / 文件或界面 | Change / 修改 |
| --- | --- | --- |
| Set RViz Fixed Frame / 设置固定坐标系 | RViz **Global Options → Fixed Frame**, saved in `src/panda_moveit/rviz/moveit.rviz` | Set `panda_link0` for the working base-frame view; use `world` only with valid TF. / 当前可用值为 `panda_link0`。 |
| Pass IK and simulated time to RViz / 给 RViz 传入 IK 与仿真时间 | `src/panda_moveit/launch/moveit.launch.py` | Include the four RViz parameters shown below. / 加入下面四项参数。 |
| Define the arm IK solver / 定义机械臂 IK | `src/panda_moveit/config/kinematics.yaml` | Keep the `arm` KDL entry. A non-chain `gripper` does not need a KDL chain solver. / 保留 `arm` 的 KDL；非链式夹爪不需要该求解器。 |

In `src/panda_moveit/launch/moveit.launch.py`, set the **RViz node's** parameter list to:

```python
parameters=[
    moveit_config.robot_description,
    moveit_config.robot_description_semantic,
    moveit_config.robot_description_kinematics,
    {"use_sim_time": is_sim},
],
```

`is_sim` is already a `LaunchConfiguration` in this launch file, and its argument defaults to `true`. Use simulation time only when a simulation clock is actually being published. For a real FR3 without Gazebo, use `is_sim:=false` and do not assume the Panda simulation's frame or launch configuration applies unchanged.

本 launch 文件中 `is_sim` 已定义为 `LaunchConfiguration`，默认 `true`。只有仿真时钟实际在发布时才应使用仿真时间。实机 FR3 不运行 Gazebo 时使用 `is_sim:=false`，不要直接套用 Panda 仿真的坐标系和启动配置。

## Rebuild and verify / 重新编译与检查

Run each command separately; press Enter after each one. / 每条命令单独执行，并分别按回车。

```bash
cd ~/panda_franka_robot
```

```bash
colcon build --symlink-install --packages-select panda_moveit
```

```bash
source install/setup.bash
```

Stop the previous Panda launch and restart it while Gazebo is running:

```bash
ros2 launch panda_moveit moveit.launch.py is_sim:=true
```

In another terminal, check each parameter separately:

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

Confirm Gazebo has a `/clock` publisher if `use_sim_time` is `True`. In RViz, check **Global Status: Ok**, choose **Interact**, select planning group **arm**, and enable **MotionPlanning → Planning Request → Query Goal State**. The goal marker should appear at the arm's end effector.

若 `use_sim_time=True`，确认 `/clock` 有 Gazebo 发布者。RViz 中检查 **Global Status: Ok**，点击 **Interact**，选择 **arm** 规划组，并勾选 **MotionPlanning → Planning Request → Query Goal State**。目标手柄应显示在机械臂末端。

## References / 参考资料

- [MoveIt: Quickstart in RViz](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) — Fixed Frame, Query Goal State, and Interact.
- [ROS 2: Understanding parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html) — node parameters, including `use_sim_time`.
- [ROS 2: Webots simulation supervisor](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Simulation-Supervisor.html) — example of a simulation publishing `/clock` and nodes using simulation time.
