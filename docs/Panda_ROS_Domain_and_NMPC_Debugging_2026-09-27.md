# Panda 仿真与 FR3 实机同时运行：MoveIt、TF、控制器和 NMPC 排查记录

日期：2026-09-27（Edmonton）  
仓库：`panda_franka_robot`  
环境：Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、MoveIt 2

## 一、这次实际观察到了什么

| 现象 | 证据 | 当前判断 |
| --- | --- | --- |
| 重复的 `/rviz2`、`/controller_manager` 等节点 | `ros2 node list` 报同名节点警告；`pgrep -af` 显示四套 `franka_fr3_moveit_config` 实机 MoveIt 启动进程，以及一套 `panda_bringup`。FR3 命令包含 `use_fake_hardware:=false` 和 `robot_ip:=172.16.0.2`。 | 多套 ROS 系统同时运行。需要先用不同 `ROS_DOMAIN_ID` 隔离 Panda 实验；同名节点和服务使未隔离的检查结果难以解释。尚未逐一核实所有现有进程的 domain。 |
| Panda 旧仿真仍在运行 | `pgrep -af` 仍列出 PID `2522909` 的 `panda_bringup pick_and_place.launch.xml is_sim:=false`，及其 Gazebo 启动命令。 | 之前的启动没有完全退出。PID 只是本次记录，不能在以后的会话中照抄。 |
| 控制器列表缺少 `arm_controller` | 一次 `ros2 control list_controllers -c /controller_manager` 只显示 `joint_state_broadcaster`、`gripper_controller` 为 active。 | 当时查询到的 controller manager 尚未显示可执行手臂轨迹的 arm controller。重复 manager 存在时，不能断言唯一的 Gazebo manager 状态。 |
| `color_detector` 曾报告两个 TF 树不相连 | 日志反复提示 `panda_link0` 与 `camera_link` 不在同一棵树。之后 `tf2_echo panda_link0 camera_link` 先等待，随后持续给出平移 `[0.600, 0.000, 1.000]`、单位旋转，时间为 `0.0`。 | 固定相机变换最终可用；启动早期的 Invalid frame ID 不等于永久缺少 TF。固定变换显示时间 0.0 是正常现象。需在隔离后的单一仿真中复查。 |
| `/joint_states` 的八个位置几乎为零 | 用户贴出的八个位置接近 0，但没有同时贴出 `name` 数组。 | 与仓库的零位初始配置相符。必须按 `name` 映射后才能把数值归给具体关节；不要仅凭数组顺序判断。 |
| `panda_nmpc` 只显示启动日志 | `Read-only tracking node started; no commands are sent`。 | 节点已启动，但还没有收到 `/panda_nmpc/reference_trajectory`，因此没有误差日志。它当前不求解 NMPC，也不发送机器人命令。 |
| Fast DDS SHM 报错 | 多次出现 `RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed`；同一会话也收到 `/joint_states` 和 TF。 | 共享内存传输存在问题，但不能仅凭这些日志断言所有 ROS 通信失败，也不能把它当作 TF 或控制器异常的已证实根因。 |
| MoveIt 曾报告成功，随后 RViz 退出 | 一次日志为 `arm_controller successfully finished`、`Completed trajectory execution with status SUCCEEDED`，之后 `rviz2` 退出码 `-11`。 | 该次机械臂轨迹执行成功与 RViz 崩溃是两个观察结果；目前不能证明 `class_loader` 卸载警告是崩溃根因。 |

## 二、先只结束 Panda 的旧仿真，不批量结束 FR3

四套 FR3 启动命令带有 `use_fake_hardware:=false`。不要运行 `pkill -f move_group`、`pkill -f rviz2`、`killall ros2` 等按通用名字批量关闭的命令，以免影响实机进程。也不要在混合节点图中发送运动命令。

优先到原来启动 Panda 的终端按 `Ctrl+C`。若原终端已找不到，先重新确认 PID 和命令行：

```bash
pgrep -af 'ros2 launch panda_bringup|panda_franka_robot/install/panda_description'
ps -o pid,ppid,pgid,args -p 2522909,2522950,2522952
```

**仅当 PID 仍与本次 Panda 主 launch 匹配**，再向它发 SIGINT：

```bash
kill -INT 2522909
pgrep -af 'ros2 launch panda_bringup|panda_franka_robot/install/panda_description'
```

不要根据单独的 `gz sim server` 名字结束进程；此次机器上不止一个 Gazebo server。`pgrep` 偶尔打印帮助文本只说明某次输入不正确；后续 `pgrep -af ...` 的实际进程列表才是检查依据。

## 三、给 Panda 使用独立的 ROS domain

在**每一个用于 Panda 的终端**设置同一个 domain，并且只 source Jazzy 与 Panda 工作空间：

```bash
export ROS_DOMAIN_ID=71
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 daemon stop
ros2 node list
```

这里的 `71` 是本次 Panda 实验的示例值。启动前，`ros2 node list` 应为空；如果已有节点，先确认它们属于谁，另选一个空闲 domain。不同 domain 的 ROS 2 节点不会互相发现，但**不会停止**其他 domain 中的 FR3 实机程序。不要只在一个终端设置变量：仿真、CLI、commander 与 `panda_nmpc` 必须使用相同值。

ROS 2 官方说明：[The ROS_DOMAIN_ID](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html)。

## 四、核对实际安装的 launch 文件

GitHub 当前的 `src/panda_bringup/launch/pick_and_place_commander.launch.xml` 只启动 commander，不再重复激活控制器。截图却出现 `ros2 control set_controller_state ... active`，因此先比较 source 与 ROS 实际找到的安装副本：

```bash
cd ~/panda_franka_robot
ros2 pkg prefix --share panda_bringup
diff -u src/panda_bringup/launch/pick_and_place_commander.launch.xml \
  "$(ros2 pkg prefix --share panda_bringup)/launch/pick_and_place_commander.launch.xml"
```

若文件不同，在当前工作空间重建并重新 source，然后再检查一次：

```bash
colcon build --symlink-install --packages-select panda_bringup
source install/setup.bash
```

如果 source 本身也包含重复激活命令，先查看 `git status --short` 和本地文件内容；不要盲目覆盖尚未提交的改动。

## 五、只启动一套 Panda 仿真并验证

在已设置 `ROS_DOMAIN_ID=71` 的终端启动：

```bash
ros2 launch panda_bringup pick_and_place.launch.xml
```

此文件启动 Gazebo、controller spawners、MoveIt/RViz 和颜色检测。它把包含的 MoveIt 的 `is_sim` 固定为 `True`；在外层命令行追加 `is_sim:=false` 不会覆盖这个内层值，也不适合当前 Gazebo 测试。

在另一个同 domain、同工作空间的终端检查：

```bash
ros2 control list_controllers -c /controller_manager
ros2 node list | sort | uniq -d
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo panda_link0 camera_link
ros2 param get /move_group use_sim_time
ros2 param get /rviz2 use_sim_time
```

目标：`arm_controller`、`gripper_controller`、`joint_state_broadcaster` 均为 active；没有意外重复的 `/controller_manager`；相机 TF 可以输出；MoveIt 与 RViz 均使用仿真时间。`tf2_echo` 启动时短暂出现 Invalid frame ID 后转为稳定输出，不必把首条提示视为最终结果。

如果已确认**只有一个** controller manager，但 arm controller 仍未加载，可单独执行：

```bash
ros2 run controller_manager spawner arm_controller \
  -c /controller_manager --controller-manager-timeout 120
ros2 control list_controllers -c /controller_manager
```

若 spawner 失败，保留其完整错误和主 launch 日志，检查加载失败原因。ros2_control 官方说明：[Controller Manager / spawner](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html)。

## 六、RViz 时间与初始姿态

当前 GitHub 版本的 `panda_moveit/launch/moveit.launch.py` 把 `use_sim_time` 传给 `move_group`，但 RViz 的 `parameters` 中缺少同一设置。需要在 `rviz_node` 的参数列表中加入：

```python
{"use_sim_time": is_sim},
```

修改后运行 `colcon build --symlink-install --packages-select panda_moveit`，重新 source 并重启整个仿真。不要把这项确定的配置不一致直接宣称为 RViz 退出码 `-11` 的已证实原因；如果时间统一后仍崩溃，需要另查 RViz/图形栈。

SRDF 的 `home` 和 `panda_moveit/config/initial_positions.yaml` 将关节设为零；后者注明用于 fake ros2_control。RViz 可同时显示当前状态和规划目标，因此重叠的两种颜色不必然表示加载了两台机器人。按 `/joint_states.name` 对应的 `position` 判断真实仿真姿态，再检查 RViz 的 Start State/Goal State。

## 七、MoveIt 到 `panda_nmpc` 的下一步

当前 `panda_nmpc/panda_nmpc/nmpc_node.py` 订阅 `/joint_states` 和 `/panda_nmpc/reference_trajectory`，通过 `reference_trajectory.py` 插值目标位置，并计算七个关节位置误差的二范数。`optimizer.py`、`robot_model.py` 仍是占位接口。

先在 RViz 对 `arm` 规划并执行一段小幅度可达运动，确认模拟控制器工作。**RViz 中按 Plan/Execute 不会自动向 `panda_nmpc` 的 reference topic 发布消息。** 要记录 commander 所执行的 MoveIt 轨迹，需要在 `panda_commander.cpp` 的成功规划分支，把 `plan.trajectory_.joint_trajectory` 发布到 `/panda_nmpc/reference_trajectory`，并只对七轴 `arm_` 发布；然后再执行该计划。相应地在 `panda_commander` 的 `package.xml` 和 `CMakeLists.txt` 增加 `trajectory_msgs` 依赖。

在同 domain 终端启动只读诊断：

```bash
ros2 run panda_nmpc nmpc_node --ros-args -p use_sim_time:=true
ros2 topic info /panda_nmpc/reference_trajectory -v
```

若没有参考轨迹，节点只会显示 `Read-only tracking node started; no commands are sent`；这不是优化器运行成功，也不是报错。收到七关节轨迹后才会显示 `Accepted reference trajectory` 和 `joint position error=... rad`。当前代码以 `time.monotonic()` 从消息接收时刻开始计时，尚未精确对齐控制器真正开始执行的仿真时间，实验级误差曲线需要修正这个时间基准。

后续真正的 NMPC 阶段才是在 `robot_model.py` 中建立并验证预测模型，在 `optimizer.py` 中加入动态、约束和求解器，以当前测量状态和未来 MoveIt 参考点滚动优化。切换到 NMPC 执行时，不要让 MoveIt `execute(plan)` 和 NMPC 同时向同一个 arm controller 下发竞争命令。Panda 仿真中的位置控制验证也不能直接视为 FR3 实机扭矩 NMPC 验证。

## 八、本次尚未证实的事项

- 还没有一份隔离到独立 domain 后的 controller 列表，因此不能宣布 arm controller 已修复。
- Fast DDS SHM 报错的根因未查明；不要在仍有实机节点运行时盲目清空 `/dev/shm`。
- RViz 的 `-11` 崩溃根因未查明，虽然同次 MoveIt 轨迹报告 `SUCCEEDED`。
- 没有收到来自 MoveIt 的 reference topic，也没有运行 NMPC 求解器或由它控制 Panda。
