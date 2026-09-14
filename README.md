# panda_franka_robot

Franka Panda 视觉分拣仿真学习与二次开发项目，使用 ROS 2 Jazzy、Gazebo Harmonic、MoveIt 2 和 OpenCV。

## Reference / 来源与致谢

本项目基于 **heimizhou1314 / zjs** 的 [Franka-Panda-Robot-Project](https://github.com/heimizhou1314/Franka-Panda-Robot-Project)。原始机器人模型、控制器配置、视觉检测、MoveIt 配置和抓取程序来自该项目，并非本仓库从零实现。

- 上游基线提交：`1eb59f4272fc6f98a4f828ec6a2c1f5b117edec4`
- 原始说明：[UPSTREAM_README.md](UPSTREAM_README.md)，保留原作者开发过程和演示链接；运行步骤以本 README 为准。
- 保留原始 [Apache 2.0 LICENSE](LICENSE) 及源码署名。上游对 Franka 模型来源的说明也保留在原始文档中。
- 本版本修改：重写安装步骤；移除 Gazebo 仿真中额外的独立 controller manager；移除任务启动时重复激活控制器的命令。修改文件内带有修改说明。

## 1. First: git clone / 首先下载项目

在 Ubuntu 终端执行：

```bash
cd ~
git clone https://github.com/ytang19-glitch/panda_franka_robot.git
cd ~/panda_franka_robot
```

## 2. Copy the six packages / 复制六个功能包

```bash
mkdir -p ~/panda_robot_ws/src
cd ~/panda_franka_robot
cp -a src/{panda_description,panda_controller,panda_moveit,panda_vision,panda_commander,panda_bringup} ~/panda_robot_ws/src/
ls ~/panda_robot_ws/src
```

六个包应直接放在 `~/panda_robot_ws/src/` 下，不要把它们放进另一个 `panda_bringup` 文件夹。`cp -a` 会覆盖目标中同名的文件；如有自己的修改，请先备份。避免同时保留另一套同名包，否则 `colcon` 可能报重复包名。

| Package | 功能 |
| --- | --- |
| `panda_description` | URDF/Xacro、模型、相机和 Gazebo 场景 |
| `panda_controller` | 控制器配置与 spawner |
| `panda_moveit` | MoveIt 2 规划配置与 RViz |
| `panda_vision` | OpenCV 色块检测 |
| `panda_commander` | 抓取与放置任务程序 |
| `panda_bringup` | 整套系统和任务启动文件 |

源码仓库与运行工作区是两份副本：修改仓库后，需要再次复制相关包，再在工作区编译。

## 3. Dependencies and build / 依赖与编译

前提：已安装 Ubuntu 24.04 和 ROS 2 Jazzy，且有可用的图形界面。以下命令在运行 ROS 的同一个环境执行；Docker 用户应在容器内使用实际工作区路径，例如 `/panda_ws`。

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-cv-bridge \
  python3-opencv \
  python3-colcon-common-extensions \
  python3-rosdep

source /opt/ros/jazzy/setup.bash
```

仅第一次配置 rosdep 时运行 `sudo rosdep init`；已初始化则跳过。

```bash
rosdep update
cd ~/panda_robot_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

## 4. Start the simulation / 启动仿真

终端 A：

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_robot_ws/install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.xml
```

这个启动文件依次声明 Gazebo、控制器 spawner、MoveIt/RViz 和颜色检测节点。声明顺序不代表它们会等待前一个组件初始化完毕；spawner 会等待 Gazebo 中的 controller manager。

```xml
<launch>
    <!-- Gazebo 仿真环境 -->
    <include file="$(find-pkg-share panda_description)/launch/gazebo.launch.xml" />

    <!-- 机器人控制器 -->
    <include file="$(find-pkg-share panda_controller)/launch/controller.launch.xml" />

    <!-- MoveIt 运动规划 -->
    <include file="$(find-pkg-share panda_moveit)/launch/moveit.launch.py">
        <arg name="is_sim" value="True"/>
    </include>

    <!-- 颜色检测视觉节点 -->
    <node pkg="panda_vision" exec="color_detector" name="color_detector" output="screen"/>
</launch>
```

Gazebo 的 `gz_ros2_control` 插件负责创建仿真的 controller manager；`controller.launch.xml` 只启动三个 spawner，不再另启 `ros2_control_node`。

## 5. Check controllers, then run the task / 检查后运行任务

终端 B：

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_robot_ws/install/setup.bash
ros2 node list
ros2 control list_controllers -c /controller_manager
```

预期只有一个 `/controller_manager`，并且 `joint_state_broadcaster`、`arm_controller`、`gripper_controller` 都是 `active`。如果控制器没有准备好，先排查 Gazebo 插件和启动日志。

```bash
ros2 launch panda_bringup pick_and_place_commander.launch.xml target_color:=R
```

`R`、`G`、`B` 分别对应红、绿、蓝。任务 launch 不再重复调用 `set_controller_state ... active`；请在控制器就绪后手动启动任务。一次任务结束后按 `Ctrl+C`，再启动下一次。

## Troubleshooting / 常见问题

- **`Waiting for data on robot_description` 来自独立的 `ros2_control_node`**：确认运行的是本版本的 `controller.launch.xml`，并停止旧的本项目 launch 后重启。该修复移除了多余节点；若 Gazebo 本身仍缺模型，继续检查 `robot_state_publisher`、模型生成和 Gazebo 插件日志。
- **`cannot activate ... from its current state active`**：控制器已激活，避免再次发激活命令。本版本的任务 launch 已移除这些命令。
- **`list_controllers` 超时**：检查 `ros2 node list` 是否有同名管理器，Gazebo 是否已加载机器人，以及终端是否属于相同的容器/ROS 环境。
- **改源码后没有变化**：将修改从仓库复制到工作区，在工作区重新编译并 source。可用 `ros2 pkg prefix panda_controller` 确认当前加载的安装位置。

## Validation status / 验证状态

本版本已进行 XML 解析和启动结构静态检查。本准备环境没有运行 ROS 2/Gazebo，因此尚未验证编译、实际控制器激活或完整抓取效果。上游演示不代表本版本已通过运行验证。
