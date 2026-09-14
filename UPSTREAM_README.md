# Franka Panda 色块分拣机械臂项目

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros) ![Gazebo Sim](https://img.shields.io/badge/Gazebo_Sim-Harmonic-blue?logo=ros&logoColor=white) ![MoveIt](https://img.shields.io/badge/MoveIt-2-green?logo=robotics) ![Python](https://img.shields.io/badge/Python-3.12-blue?logo=python) ![C++](https://img.shields.io/badge/C%2B%2B-17-blue?logo=cplusplus)

一个基于 ROS 2 的 Franka Panda 机械臂色块分拣项目：结合 OpenCV 视觉识别、MoveIt 2 运动规划与 Gazebo 仿真环境，使 Panda 机器人能够检测桌面上的彩色色块（红、绿、蓝），并自动完成「抓取 → 投放至垃圾桶」的全流程操作。

## 项目概述

本项目在 **ROS 2 Jazzy** 上演示了一条完整的机器人颜色分拣流水线。Gazebo 仿真场景中的相机将图像发布到 ROS 话题，`panda_vision` 节点使用 OpenCV 检测红、绿、蓝色块的位置，`panda_commander` 节点基于 MoveIt 2 规划运动轨迹，控制 Panda 机械臂将目标颜色的色块抓取并投放到垃圾桶中。

**核心特性：**
- Franka Panda 七自由度机械臂 + 二指夹爪仿真（URDF/Xacro 描述）。
- OpenCV HSV 颜色检测（红、绿、蓝），支持坐标经 TF2 变换到机械臂基座标系。
- MoveIt 2 运动规划与执行（OMPL RRTConnect / RRTstar）。
- Gazebo 仿真场景（色块、垃圾桶、咖啡桌、相机传感器）与 RViz 可视化。
- ros2_control 关节控制器（机械臂 + 夹爪 + 状态广播器），附带 GUI 测试节点。
- C++ 命令节点（`panda_commander`），自动完成移动、抓取、确认、投放、回原点全流程。
- bringup 启动文件，一键启动所有组件。

**关键词：** 机器人、ROS 2、Franka Panda、MoveIt 2、Gazebo、OpenCV、颜色分拣、抓取-放置

## 📂 项目结构

```
panda_robot_ws/
├── src/
│   ├── panda_description/          # URDF/Xacro 模型与 Gazebo 仿真资源
│   │   ├── urdf/                   # Panda URDF 文件（arm、sensors、gazebo、ros2_control）
│   │   ├── meshes/                 # 机器人网格模型（visual / collision）
│   │   ├── models/                 # Gazebo 场景模型（咖啡桌、垃圾桶）
│   │   ├── world/                  # 仿真世界文件（scene.world：色块、桌子、垃圾桶）
│   │   ├── config/                 # ros_gz_bridge 桥接配置
│   │   ├── rviz/                   # RViz 显示配置
│   │   └── launch/                 # 启动文件（display.launch.xml、gazebo.launch.xml）
│   ├── panda_controller/           # ros2_control 控制器配置与测试节点
│   │   ├── config/                 # 控制器 YAML（机械臂、夹爪、状态广播器）
│   │   ├── test_panda_controller/  # 测试节点（test_controller.cpp）
│   │   └── launch/                 # 控制器启动文件（controller、test_controller）
│   ├── panda_moveit/               # MoveIt 2 运动规划配置
│   │   ├── config/                 # SRDF、运动学、规划器、关节限位等配置
│   │   ├── launch/                 # moveit.launch.py
│   │   └── rviz/                   # MoveIt RViz 配置
│   ├── panda_vision/               # OpenCV 颜色检测节点（Python）
│   │   └── panda_vision/           # 颜色检测节点源码（color_detector.py）
│   ├── panda_commander/            # C++ 抓取指挥官节点（pick-and-place）
│   │   └── src/                    # panda_commander.cpp
│   ├── panda_bringup/              # 整套系统统一启动
│   │   └── launch/                 # pick_and_place、pick_and_place_commander 启动文件
└── README.md                       # 本文档
```

## 🛠️ 安装

### 环境依赖

- **操作系统**：Ubuntu 24.04（对应 ROS 2 Jazzy）。
- **ROS 2**：Jazzy Jalisco。
- **主要依赖**：
  ```bash
  sudo apt install \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-cv-bridge \
    python3-opencv \
    python3-colcon-common-extensions
  ```
  > 其余依赖（如 `tf_transformations`、`robot_state_publisher` 等）由下文的 `rosdep install` 自动补全。

### 安装步骤

1. **创建工作区并放入源码**：
   ```bash
   mkdir -p ~/panda_robot_ws/src
   ```
   将本项目工作区 `src/` 目录下的六个功能包（`panda_description`、`panda_controller`、`panda_moveit`、`panda_vision`、`panda_commander`、`panda_bringup`）复制到 `~/panda_robot_ws/src/` 中。

2. **安装 ROS 依赖**：
   ```bash
   cd ~/panda_robot_ws
   sudo apt install python3-rosdep -y
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **编译工作区**：
   ```bash
   cd ~/panda_robot_ws
   colcon build
   source install/setup.bash
   ```

## 🚀 使用

### 启动系统

1. **一键启动整套系统**（Gazebo 仿真 + 控制器 + MoveIt + RViz + 视觉检测）：
   ```bash
   ros2 launch panda_bringup pick_and_place.launch.xml
   ```
   - 启动 Gazebo 仿真世界：桌面上摆放红、绿、蓝三个色块，另有垃圾桶与咖啡桌；
   - 加载并激活机械臂（`arm_controller`）与夹爪（`gripper_controller`）控制器；
   - 启动 MoveIt 2 运动规划节点与 RViz；
   - 启动颜色检测节点，并弹出 OpenCV 显示窗口（实时显示识别到的色块）。

2. **运行抓取命令节点，指定目标颜色（R、G 或 B）**：
   - 在另一个终端中执行：
   ```bash
   source ~/panda_robot_ws/install/setup.bash
   ros2 launch panda_bringup pick_and_place_commander.launch.xml target_color:=R
   ```
   - 将 `R` 替换为 `G` 或 `B` 可更换目标颜色（target_color参数默认为 `R`）；
   - 单次任务完成后，按 `Ctrl+C` 停止节点，换一个颜色重新运行即可。

3. **运行效果展示**：
   - 见b站视频链接：【基于 ROS2 的 Franka Panda机械臂视觉分拣系统】 https://www.bilibili.com/video/BV1jR496JEYZ/?share_source=copy_web&vd_source=7afa3d85eb826d11f0b248d616bcde00

### 系统流程

1. Gazebo 中的相机将桌面图像发布到话题 `/camera/image_raw`。
2. 视觉节点（`color_detector.py`）通过 HSV 阈值识别红、绿、蓝色块，将像素坐标经 TF2 变换到 `panda_link0` 基座标系，并发布到话题 `/color_coordinates`。
3. 抓取（`panda_commander`）订阅目标颜色的坐标和机械臂关节状态（`/joint_states`）以及垃圾桶位姿（`/model/Trash_01_001/pose`），使用 MoveIt 2 规划运动。
4. Panda 机械臂依次完成：移动到色块上方 → 张开夹爪 → 直线下降 → 闭合抓取 → 确认抓稳 → 抬升 → 移动到垃圾桶上方 → 张开投放 → 回到 `home` 位姿并闭合夹爪。
5. 单次分拣任务结束。

## 🧩 开发过程

本节按功能模块梳理了本项目的搭建思路，便于二次开发与扩展。

### 第一步：机器人描述（panda_description）

- 创建 `panda_description` 功能包：
  ```bash
  cd ~/panda_robot_ws/src
  ros2 pkg create --build-type ament_cmake panda_description
  ```
- 从 [franka_ros2 仓库](https://github.com/frankaemika/franka_ros2) 复制 Panda 的 URDF 与网格模型；
- 补充相机传感器（`sensors.xacro`）、Gazebo 插件（`gazebo.xacro`）、ros2_control 硬件接口（`ros2control.xacro`）；
- 在 RViz 中查看模型：
  ```bash
  ros2 launch panda_description display.launch.xml
  ```
- 在 Gazebo 中查看仿真场景：
  ```bash
  ros2 launch panda_description gazebo.launch.xml
  ```

### 第二步：控制器配置（panda_controller）

- 创建 `panda_controller` 功能包：
  ```bash
  ros2 pkg create --build-type ament_cmake panda_controller
  ```
- 添加控制器配置 `panda_controllers.yaml`（`arm_controller`、`gripper_controller`、`joint_state_broadcaster`）；
- 编写 C++ 控制器测试节点（`test_controller.cpp`）：订阅关节状态 GUI 的 `/joint_commands`，分别转发给机械臂与夹爪控制器；
- 测试控制器：
  ```bash
  ros2 launch panda_description gazebo.launch.xml
  ros2 launch panda_controller controller.launch.xml
  ros2 launch panda_controller test_controller.launch.xml
  ```
  实验现象：可通过调节关节状态 GUI 中的各关节值，在Gazebo仿真场景中观察机械臂与夹爪的运动。

### 第三步：MoveIt 2 配置（panda_moveit）

- 启动 MoveIt Setup Assistant：
  ```bash
  ros2 run moveit_setup_assistant moveit_setup_assistant
  ```
- 配置规划组 `arm`（panda_link0 → panda_link7）与 `gripper`（夹爪），定义末端执行器及命名位姿 `home` / `open` / `close`；
- 生成 `panda_moveit` 功能包，并将配置放入 `panda_moveit/config/`；
- 测试运动规划：
  ```bash
  ros2 launch panda_description gazebo.launch.xml
  ros2 launch panda_controller controller.launch.xml
  ros2 launch panda_moveit moveit.launch.py
  ```
  实验现象：通过在Moveit2界面中设置机械臂的起始和终点状态，在Gazebo仿真场景中观察机械臂与夹爪的运动。

### 第四步：OpenCV 颜色检测（panda_vision）

- 创建 `panda_vision` 功能包：
  ```bash
  ros2 pkg create --build-type ament_python panda_vision
  ```
- 实现 `color_detector.py`：HSV 颜色分割 → 形态学去噪 → 轮廓检测 → 质心提取 → TF2 坐标变换到 `panda_link0`；
- 将识别结果（颜色 ID + 坐标）发布到 `/color_coordinates` 话题。

### 第五步：抓取命令（panda_commander）

- 创建 `panda_commander` 功能包：
  ```bash
  ros2 pkg create --build-type ament_cmake panda_commander
  ```
- 基于 `MoveGroupInterface` 实现完整 pick-and-place 流程：移动到色块上方 → 张开夹爪 → 直线下降 → 闭合抓取 → 确认抓稳 → 抬升 → 投放至垃圾桶 → 回 `home` 位姿。

### 第六步：系统集成（panda_bringup）

- 创建 `panda_bringup` 功能包：
  ```bash
  ros2 pkg create --build-type ament_cmake panda_bringup
  ```
- 编写 `pick_and_place.launch.xml`，一键启动仿真、控制器、MoveIt 与视觉节点；
- 编写 `pick_and_place_commander.launch.xml`，运行抓取命令。
  ```bash
  ros2 launch panda_bringup pick_and_place.launch.xml
  ros2 launch panda_bringup pick_and_place_commander.launch.xml target_color:=R
  ```

## 📦 功能包汇总

| 功能包            | 描述                                       |
|--------------------|--------------------------------------------|
| `panda_description` | URDF/Xacro 模型、网格、Gazebo 仿真场景与相机 |
| `panda_controller`  | ros2_control 控制器配置与控制器测试节点       |
| `panda_moveit`      | MoveIt 2 运动规划配置                      |
| `panda_vision`      | OpenCV 颜色检测与坐标发布                  |
| `panda_commander`   | C++ 抓取-投放命令节点                    |
| `panda_bringup`     | 整套系统统一启动                           |

## 🧠 关键技术

| 组件             | 用途                             | 徽标 |
|------------------|----------------------------------|------|
| **ROS 2 Jazzy**  | 机器人中间件与节点间通信         | ![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros) |
| **Gazebo Sim**   | 机器人仿真与相机图像生成         | ![Gazebo](https://img.shields.io/badge/Gazebo-Sim-blue?logo=ros) |
| **MoveIt 2**     | 运动规划与轨迹执行               | ![MoveIt](https://img.shields.io/badge/MoveIt-2-green?logo=robotics) |
| **OpenCV**       | 实时颜色检测与目标定位           | ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-blue) |
| **TF2**          | 相机与基座之间的坐标系变换       | ![ROS 2](https://img.shields.io/badge/TF2-ROS_2-blue?logo=ros) |
| **ros2_control** | 关节控制器与硬件接口             | ![ROS 2](https://img.shields.io/badge/ros2_control-Jazzy-blue?logo=ros) |
| **C++ / Python** | 节点实现（C++17 / Python 3.12）  | ![C++](https://img.shields.io/badge/C%2B%2B-17-blue?logo=cplusplus) |

## 🔧 故障排查

- **Gazebo 无法启动**：确认已安装 `ros-jazzy-ros-gz` 与 `ros-jazzy-gz-ros2-control`，并检查 `GZ_SIM_RESOURCE_PATH` 是否正确指向 `panda_description` 的 `models` 目录。
- **控制器未激活**：手动执行 `ros2 control set_controller_state arm_controller active`（夹爪同理）；启动 `pick_and_place_commander.launch.xml` 时会再次激活三个控制器。
- **MoveIt 规划失败**：检查 `panda_moveit/config/` 中的配置是否正确，并确认 `arm_controller` / `gripper_controller` 已激活。
- **颜色识别不准**：根据实际光照在 `color_detector.py` 中调整 HSV 阈值与最小轮廓面积；红、绿、蓝的补偿量也可按需微调。
- **没有相机图像**：确认 `gazebo.launch.xml` 中的 `ros_gz_image` 图像桥接节点已启动，话题 `/camera/image_raw` 有数据。
- **抓取失败**：检查 `/color_coordinates` 与 `/model/Trash_01_001/pose` 话题是否有数据、TF 变换是否正常；可调整 `panda_commander.cpp` 中 `EnsureGraspBox` 的判定阈值与超时时间。

## 🤝 贡献

欢迎对本项目提出建议或贡献代码：

- 视觉检测与运动规划的改进；
- 仿真或控制问题的修复；
- 新功能（例如多色块连续分拣、真实机器人支持等）。

## 📜 许可证

本项目采用 [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) 开源协议。

## 📧 联系方式

如有问题或建议，欢迎通过 [GitHub Issues](https://github.com/heimizhou1314/Franka-Panda-Robot-Project/issues) 或 邮件 (208979620@qq.com or chuxinqingmu@gmail.com)。
