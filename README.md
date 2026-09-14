# panda_franka_robot — Franka Panda Vision-Guided Sorting

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros) ![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-blue) ![MoveIt](https://img.shields.io/badge/MoveIt-2-green) ![Python](https://img.shields.io/badge/Python-3.12-blue?logo=python) ![C++](https://img.shields.io/badge/C%2B%2B-17-blue?logo=cplusplus)

A ROS 2 simulation project for learning robotic manipulation with a Franka Panda arm: detect colored objects with OpenCV, plan movements with MoveIt 2, and execute pick-and-place tasks in Gazebo.

Maintained by **Yujie Tang** as a learning and further-development project.

## Reference and Acknowledgments

**This project is based on [Franka-Panda-Robot-Project](https://github.com/heimizhou1314/Franka-Panda-Robot-Project) by heimizhou1314 / zjs. That repository is my primary reference and the source of the initial six-package structure and implementation.**

The original robot description, simulation resources, controller configuration, MoveIt configuration, vision detector, and pick-and-place program come from the reference project. This README follows its organization and explains the adapted repository in English.

- **Upstream baseline:** `1eb59f4272fc6f98a4f828ec6a2c1f5b117edec4`.
- **Original documentation:** [UPSTREAM_README.md](UPSTREAM_README.md), including the author's development notes and demonstration links.
- **Attribution:** the original [Apache 2.0 license](LICENSE) and source attribution are preserved. The upstream documentation also credits [franka_ros2](https://github.com/frankaemika/franka_ros2) for the Panda models.
- **Changes in this version:** setup documentation; removal of the extra standalone controller manager in Gazebo; removal of redundant controller activation commands from the task launch; and declaration of the vision node's `tf_transformations` runtime dependency.

The inherited sorting system is credited to the original author. Further work in this repository should document its changes and validation separately.

## Project Overview

The intended workflow is to detect a red, green, or blue block in the simulated camera image, estimate its position in the robot base frame, and use MoveIt 2 to command the arm and gripper to pick it up and release it over a bin.

The inherited implementation includes:

- A seven-joint Franka Panda arm and two-finger gripper described with URDF/Xacro.
- A Gazebo scene with colored blocks, a table, a bin, and a camera.
- OpenCV HSV color segmentation and TF2 coordinate transformation.
- MoveIt 2 planning configuration and RViz visualization.
- `ros2_control` arm and gripper controllers and a joint-state broadcaster.
- A C++ pick-and-place commander and separate system/task launch files.

**Current localization limitation:** the detector uses hard-coded camera intrinsics, a fixed `Z = -0.9`, a scene-specific axis mapping, and small color-dependent offsets. It does not measure object depth. This is a simulation baseline to investigate and improve before adapting the pipeline to another camera or a real robot.

## Project Structure

The repository itself is the ROS 2 workspace. Build and run from `~/panda_franka_robot`.

| Path | Contents |
| --- | --- |
| [src/panda_description/](src/panda_description/) | Robot description and simulation resources: `urdf/`, `meshes/`, `models/`, `world/`, `config/`, `rviz/`, and `launch/` |
| [src/panda_controller/](src/panda_controller/) | Controller YAML configuration, launch files, and controller test code |
| [src/panda_moveit/](src/panda_moveit/) | MoveIt planning configuration, launch files, and RViz configuration |
| [src/panda_vision/](src/panda_vision/) | Python package containing `panda_vision/color_detector.py` |
| [src/panda_commander/](src/panda_commander/) | C++ task implementation in `src/panda_commander.cpp` |
| [src/panda_bringup/](src/panda_bringup/) | System and commander launch files |
| [UPSTREAM_README.md](UPSTREAM_README.md) | Preserved original documentation |
| [LICENSE](LICENSE) | Original Apache 2.0 license |


```bash
panda_robot_ws/
├── src/
│   ├── panda_description/          # URDF/Xacro robot model and Gazebo simulation resources
│   │   ├── urdf/                   # Panda URDF files (arm, sensors, Gazebo, ros2_control)
│   │   ├── meshes/                 # Robot meshes (visual / collision)
│   │   ├── models/                 # Gazebo scene models (coffee table, bins)
│   │   ├── world/                  # Simulation worlds (scene.world: colored blocks, table, bins)
│   │   ├── config/                 # ros_gz_bridge configuration
│   │   ├── rviz/                   # RViz display configuration
│   │   └── launch/                 # Launch files (display.launch.xml, gazebo.launch.xml)
│   ├── panda_controller/           # ros2_control controller configuration and test nodes
│   │   ├── config/                 # Controller YAML files (arm, gripper, joint state broadcaster)
│   │   ├── test_panda_controller/  # Test nodes (test_controller.cpp)
│   │   └── launch/                 # Controller launch files (controller, test_controller)
│   ├── panda_moveit/               # MoveIt 2 motion planning configuration
│   │   ├── config/                 # SRDF, kinematics, planners, joint limits, and other settings
│   │   ├── launch/                 # moveit.launch.py
│   │   └── rviz/                   # MoveIt RViz configuration
│   ├── panda_vision/               # OpenCV color detection node (Python)
│   │   └── panda_vision/           # Color detection source code (color_detector.py)
│   ├── panda_commander/            # C++ pick-and-place coordinator node
│   │   └── src/                    # panda_commander.cpp
│   └── panda_bringup/              # Unified launch package for the complete system
│       └── launch/                 # pick_and_place and pick_and_place_commander launch files
└── README.md                       # This document
```


## Installation

### Environment

- Ubuntu 24.04 with ROS 2 Jazzy installed.
- Gazebo Harmonic and MoveIt 2.
- Python 3.12 and a C++ build toolchain.
- A working graphical session for Gazebo, RViz, and the OpenCV window.

Run all commands in the environment that runs ROS. If using Docker, run them inside the container and substitute its actual workspace path, for example `/panda_ws`.

### 1. Understand the Original Clone-and-Copy Workflow

The starting workflow is **clone the reference → copy its six packages → reproduce the baseline → develop your own version**.

To study the original source in a separate workspace:

```bash
cd ~
git clone https://github.com/heimizhou1314/Franka-Panda-Robot-Project.git
mkdir -p ~/panda_upstream_ws/src
cp -a ~/Franka-Panda-Robot-Project/src/. ~/panda_upstream_ws/src/
```

`git clone` downloads the reference repository; `cp -a` copies the packages into a ROS 2 workspace. The clone command retrieves the latest upstream version by default; the baseline used for this repository is recorded above.

Use the [original instructions](https://github.com/heimizhou1314/Franka-Panda-Robot-Project#readme) when studying that workspace. Keep it separate from the adapted version below so that copying the original source does not overwrite this repository's fixes.

### 2. Clone This Repository

The six packages are already included here, so no copying is required to run this version.

```bash
cd ~
git clone https://github.com/ytang19-glitch/panda_franka_robot.git
cd ~/panda_franka_robot
```

If it is already cloned:

```bash
cd ~/panda_franka_robot
git pull --ff-only origin main
```

### 3. Install Dependencies

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
  ros-jazzy-tf-transformations \
  python3-opencv \
  python3-colcon-common-extensions \
  python3-rosdep

source /opt/ros/jazzy/setup.bash
```

Run `sudo rosdep init` only if rosdep has not previously been initialized. Then:

```bash
rosdep update
cd ~/panda_franka_robot
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

### 4. Build the Workspace

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Start the System

In Terminal A:

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.xml
```

This launch starts Gazebo, controller spawners, MoveIt/RViz, and the color detector. Its structure is:

```xml
<launch>
    <!-- Gazebo simulation environment -->
    <include file="$(find-pkg-share panda_description)/launch/gazebo.launch.xml" />

    <!-- Robot controller spawners -->
    <include file="$(find-pkg-share panda_controller)/launch/controller.launch.xml" />

    <!-- MoveIt motion planning -->
    <include file="$(find-pkg-share panda_moveit)/launch/moveit.launch.py">
        <arg name="is_sim" value="True"/>
    </include>

    <!-- Color detection node -->
    <node pkg="panda_vision" exec="color_detector" name="color_detector" output="screen"/>
</launch>
```

Gazebo's `gz_ros2_control` plugin creates the simulation controller manager. The controller launch starts three spawners, which wait for that manager. XML declaration order does not guarantee that each component finishes initialization before the next starts.

### Check Readiness and Run a Task

In Terminal B:

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 node list
ros2 control list_controllers -c /controller_manager
ros2 topic info /color_coordinates
```

Expect a single `/controller_manager` and these controllers in the `active` state:

- `joint_state_broadcaster`
- `arm_controller`
- `gripper_controller`

Confirm that the vision topic has a publisher and is producing coordinates before starting the task:

```bash
ros2 topic echo /color_coordinates
```

Stop the topic echo with `Ctrl+C`, then run:

```bash
ros2 launch panda_bringup pick_and_place_commander.launch.xml target_color:=R
```

Use `R`, `G`, or `B` to select red, green, or blue. The task launch does not repeat controller activation commands. Press `Ctrl+C` after the task finishes before starting another run.

Use the same `ROS_DOMAIN_ID` in all terminals. If your simulation uses domain 42, run `export ROS_DOMAIN_ID=42` in each terminal before ROS commands.

### System Workflow

1. The Gazebo camera image is bridged to `/camera/image_raw`.
2. `color_detector.py` segments colors, extracts bounding-box centers, and estimates camera-frame positions using its fixed scene assumptions.
3. TF2 supplies the transform from `camera_link` to `panda_link0`. The detector publishes strings in the form `R,x,y,z` on `/color_coordinates`.
4. The commander uses target coordinates, joint states, and the bin pose to plan and execute the task through MoveIt 2.
5. The intended task sequence is approach, open, descend, grasp, check the grasp, lift, move over the bin, release, and return home.

**Reference demonstration:** see the original author's [sorting video](https://www.bilibili.com/video/BV1jR496JEYZ/). This demonstrates the upstream project, not independent runtime validation of this version.

## Development Process

The six stages below follow the reference project's organization. They describe the inherited modules and where to study or extend them; they are not a claim that these modules were originally written by this repository's maintainer.

### Step 1: Robot Description — `panda_description`

Study the URDF/Xacro model, meshes, camera configuration, Gazebo plugins, and scene resources. Understand how robot links and joints establish the TF tree.

To inspect the robot description separately:

```bash
ros2 launch panda_description display.launch.xml
```

To inspect the Gazebo scene separately:

```bash
ros2 launch panda_description gazebo.launch.xml
```

Stop standalone launches before starting the complete system.

### Step 2: Controller Configuration — `panda_controller`

Study the arm and gripper controller configuration and joint-state broadcaster. Understand how planned joint trajectories reach the simulated robot.

This version uses Gazebo's controller manager and starts controller spawners through `controller.launch.xml`. It removes the extra standalone `ros2_control_node` that could create a duplicate manager.

### Step 3: Motion Planning — `panda_moveit`

Study the planning groups, named poses, kinematics, joint limits, planner configuration, and controller connections. Use RViz to inspect planned motion and compare it with execution in Gazebo.

### Step 4: Color Detection and Localization — `panda_vision`

Study the detector pipeline: HSV thresholding, erosion/dilation, contour filtering, bounding-box centers, camera-coordinate estimation, and TF2 transformation.

The current implementation already contains TF2 lookup and point transformation. A missing `tf_transformations` Python dependency prevents the node from starting; it does not mean the source lacks transformation code. Improving depth estimation and calibration is a separate development task.

### Step 5: Pick-and-Place Commands — `panda_commander`

Study the C++ task sequence and MoveIt `MoveGroupInterface` calls. Identify where approach poses, grasp checks, release behavior, and return motion are defined before changing task logic.

### Step 6: System Integration — `panda_bringup`

Study how `pick_and_place.launch.xml` brings up the system and how `pick_and_place_commander.launch.xml` starts a selected-color task. Keep system readiness checks separate from task execution.

### Make and Record Your Own Changes

Edit the source directly in this repository and rebuild here. For example:

```bash
cd ~/panda_franka_robot
git switch -c feature/vision-improvements
# Edit src/panda_vision/panda_vision/color_detector.py.
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select panda_vision
source install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.xml
```

After stopping the previous simulation, run the updated version and record the result. Commit only after making and reviewing actual changes:

```bash
cd ~/panda_franka_robot
git diff
git add src/panda_vision/panda_vision/color_detector.py
git commit -m "Improve color detection"
git push -u origin feature/vision-improvements
```

Preserve upstream attribution and include change notices in modified upstream files.

## Package Summary

| Package | Responsibility |
| --- | --- |
| `panda_description` | Robot model, simulated scene, and camera |
| `panda_controller` | Joint controller configuration and spawners |
| `panda_moveit` | Motion planning and execution configuration |
| `panda_vision` | Color detection, coordinate estimation, and publication |
| `panda_commander` | Pick-and-place task logic |
| `panda_bringup` | System and task launch integration |

## Key Technologies

| Technology | Role in This Project |
| --- | --- |
| ROS 2 Jazzy | Nodes, topics, parameters, and launch |
| Gazebo Harmonic | Robot physics, scene, and simulated camera |
| MoveIt 2 | Motion planning and trajectory execution |
| OpenCV | Image processing and color segmentation |
| TF2 | Transform lookup between camera and robot frames |
| ros2_control | Controller management and simulated joint interfaces |
| Python / C++ | Vision processing and task implementation |

## Troubleshooting

### Vision node fails: missing tf_transformations

If the detector exits with `ModuleNotFoundError: No module named 'tf_transformations'`, install the missing ROS package:

```bash
sudo apt update
sudo apt install ros-jazzy-tf-transformations
source /opt/ros/jazzy/setup.bash
python3 -c "import tf_transformations; print('tf_transformations import OK')"
```

The Python module name is `tf_transformations`; its Ubuntu ROS Jazzy package name is `ros-jazzy-tf-transformations`. It is now declared as a runtime dependency in `src/panda_vision/package.xml`, so `rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy` can install it from the updated source.

This missing import prevents the vision node from starting, which leaves `/color_coordinates` without a publisher. Installing the dependency does not require recompiling the already-built Python node. Restart the failed detector or restart the main launch; avoid running both copies.

To diagnose the detector directly, source your actual built workspace first, then run:

```bash
ros2 run panda_vision color_detector --ros-args -p use_sim_time:=true
```

Use the same `ROS_DOMAIN_ID` as the simulation in every terminal (for example, `export ROS_DOMAIN_ID=42` if the simulation uses 42). Check `ros2 topic info /color_coordinates` from another terminal. If the detector starts but reports TF lookup failures, inspect `robot_state_publisher` and robot transforms separately; installing this Python module does not create missing TF frames.

- **An independent `ros2_control_node` reports `Waiting for data on robot_description`:** confirm that you are running this version's `controller.launch.xml`. Stop the previous project launch and restart. This version removes the extra node; if Gazebo itself is missing the model, inspect `robot_state_publisher`, model spawning, and Gazebo plugin logs.
- **`cannot activate ... from its current state active`:** the controller is already active. Avoid sending another activation command. This version removes those commands from the task launch.
- **`list_controllers` times out:** check for duplicate controller managers in `ros2 node list`, confirm that Gazebo has loaded the robot, and ensure the terminals use the same container and ROS environment.
- **Source changes have no effect:** rebuild in `~/panda_franka_robot` and source `~/panda_franka_robot/install/setup.bash` in each terminal. Use `ros2 pkg prefix panda_controller` to confirm which installation is being loaded.

## Further Development

The following items are planned directions, not completed features:

- Reproduce the baseline and record grasp success, localization error, and task duration.
- Improve camera calibration, depth estimation, and grasp-pose generation.
- Add failure detection, retries, and clearer task-state reporting.
- Explore visual servoing or trajectory optimization after establishing a stable baseline.
- Investigate NMPC as a later control extension with a defined model, objective, and constraints.



### Development 1:

Python is where you write the controller. CasADi helps you express and solve the optimization problem inside it. Let’s start with a small working example before connecting to Gazebo.

We will control a simple simulated joint using:

$$ \ddot q=u-\sin(q)-0.1\dot q $$

Here, \(q\) is joint angle, \(\dot q\) is velocity, and \(u\) is the control input. This is a simplified nonlinear teaching model—not your Panda’s dynamics.


#### 1. Install CasADi

Run in an Ubuntu terminal:
```bash
python3 -m venv ~/venvs/nmpc
source ~/venvs/nmpc/bin/activate
pip install casadi numpy matplotlib
```
#### 2. Understand the main CasADi commands

Python code

| Python code            | Meaning                                  |
| ---------------------- | ---------------------------------------- |
| `opti = ca.Opti()`     | Create an optimization problem           |
| `opti.variable(...)`   | Create quantities the solver must choose |
| `opti.parameter(...)`  | Create inputs you supply before solving  |
| `opti.minimize(cost)`  | Define what “best” means                 |
| `opti.subject_to(...)` | Define equations and limits              |
| `opti.solve()`         | Calculate the optimal solution           |







## Development 2: Create a Python ROS 2 Node for NMPC

This tutorial establishes the feedback connection to the simulated Panda. The example below **only reads joint angles**: it does not solve an NMPC problem or command motion. Follow these steps locally to create the package; this documentation update does not install a package on your computer.

### What is a node, and where does the Python file go?

A ROS 2 node is a running program that communicates through ROS interfaces. Here, Python uses `rclpy` to subscribe to the robot's `/joint_states` topic.

| Path relative to `~/panda_franka_robot/` | Purpose |
| --- | --- |
| `experiments/nmpc/first_nmpc.py` | Standalone CasADi learning experiment; no ROS build or Gazebo required |
| `src/panda_nmpc/` | ROS package, with dependency and installation files |
| `src/panda_nmpc/panda_nmpc/` | Python module containing the node code |
| `src/panda_nmpc/panda_nmpc/nmpc_node.py` | Python file to create/edit for this tutorial |
| `src/panda_nmpc/setup.py` | Registers the executable used by `ros2 run` |
| `src/panda_nmpc/package.xml` | Declares ROS package dependencies |

The repeated `panda_nmpc/panda_nmpc` is intentional: the outer directory is the ROS package; the inner directory is the Python module.

### Step 1: Create the package

Use a terminal with the system ROS Python environment. If the earlier virtual environment is active (the prompt shows `(nmpc)`), run `deactivate` first. This subscriber does not need CasADi.

Run package creation only once. If `src/panda_nmpc` already exists, inspect it and continue with its files instead.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/panda_franka_robot/src

ros2 pkg create panda_nmpc \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs trajectory_msgs \
  --node-name nmpc_node
```

### Step 2: Open and edit the Python node

```bash
code ~/panda_franka_robot/src/panda_nmpc/panda_nmpc/nmpc_node.py
```

If VS Code's `code` command is unavailable, use `nano` with the same path. Replace the generated starter code with:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class PandaNMPCNode(Node):
    def __init__(self):
        super().__init__("panda_nmpc")
        self.joint_names = [
            f"panda_joint{i}" for i in range(1, 8)
        ]
        self.q = None

        self.joint_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0, self.show_state)
        self.get_logger().info(
            "Panda node started. Waiting for /joint_states..."
        )

    def joint_state_callback(self, msg):
        # Match by name; incoming message order may differ.
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in self.joint_names):
            return
        self.q = [positions[name] for name in self.joint_names]

    def show_state(self):
        if self.q is None:
            self.get_logger().info(
                "Still waiting for all seven Panda joints..."
            )
            return
        values = ", ".join(f"{angle:.3f}" for angle in self.q)
        self.get_logger().info(f"Joint angles [rad]: [{values}]")


def main(args=None):
    rclpy.init(args=args)
    node = PandaNMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

Save the file before building.

| Code | Meaning |
| --- | --- |
| `class PandaNMPCNode(Node)` | Defines the ROS node |
| `create_subscription(...)` | Receives joint-state messages |
| `joint_state_callback(...)` | Runs when a message arrives |
| `self.q` | Stores the seven measured joint angles in radians |
| `create_timer(1.0, ...)` | Displays the latest measurement once per second |
| `rclpy.spin(node)` | Keeps the node processing callbacks |

### Step 3: Build and source the package

```bash
source /opt/ros/jazzy/setup.bash
cd ~/panda_franka_robot
colcon build --symlink-install --packages-select panda_nmpc
source install/setup.bash
```

The `--node-name nmpc_node` option in Step 1 generates the console entry point in `setup.py`:

```python
'nmpc_node = panda_nmpc.nmpc_node:main'
```

The initial package requires a build. With `--symlink-install`, edits to this existing Python module normally require only saving and restarting the node. Rebuild after changing package installation settings or entry points.

### Step 4: Start the simulation in Terminal 1

Use the same `ROS_DOMAIN_ID` in every terminal. If your existing setup uses domain 42, run `export ROS_DOMAIN_ID=42` in each terminal **before** starting ROS programs. Otherwise keep your existing consistent domain setting.

If the main simulation is already running, use that instance instead of launching another.

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.xml
```

### Step 5: Run the subscriber in Terminal 2

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_franka_robot/install/setup.bash
ros2 run panda_nmpc nmpc_node --ros-args -p use_sim_time:=true
```

Expect a log containing `Joint angles [rad]: [...]`, with seven numbers from the simulated Panda. Unpause Gazebo: the timer uses simulation time and needs an advancing `/clock`.

Stop the subscriber with `Ctrl+C`.

### Step 6: Troubleshoot the connection

Run these checks in another terminal with the same ROS environment and domain:

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /clock --once
ros2 control list_controllers -c /controller_manager
ros2 pkg prefix panda_nmpc
```

- **Package not found:** build the package and source this workspace's `install/setup.bash`.
- **No executable found:** check the console entry point in `setup.py`, rebuild, and source again.
- **No joint messages:** check that Gazebo and `joint_state_broadcaster` are running and all terminals use the same domain/container.
- **Waiting for seven joints:** inspect the message names; the example expects `panda_joint1` through `panda_joint7`.
- **Only the startup log appears:** check that simulation time is advancing.
- **`rclpy` import fails:** use the sourced system ROS Python environment for this stage.

The printed values are the latest received values; this learning subscriber does not detect stale feedback yet.

### Step 7: Extend the verified node into NMPC

The following are planned implementation steps, not functionality supplied by this subscriber:

| File beside `nmpc_node.py` | Next responsibility |
| --- | --- |
| `model.py` | Panda forward kinematics, validated against TF |
| `optimizer.py` | CasADi prediction model, pose-error objective, and constraints |
| `nmpc_node.py` | Measured feedback, optimizer calls, and command publication |

Start with kinematic NMPC for a nearby fixed gripper pose. Use the seven measured joint angles as the initial state, optimize joint velocities, and convert the first optimized step into a joint-position reference. The nonlinear Panda forward kinematics enters the gripper-pose objective.

The existing arm uses position commands. The planned node can send all seven joint positions with timing in `trajectory_msgs/msg/JointTrajectory` messages on `/arm_controller/joint_trajectory`. Monitor actual tracking separately; topic publication alone does not confirm successful execution.

Before enabling motion, add joint limits, bounded command changes, collision considerations, stale-feedback checks, and handling for failed or late solves. The toy single-joint dynamics in the standalone experiment must not be used as the Panda model.

When the new node starts publishing arm commands, leave the pick-and-place commander stopped and avoid MoveIt execution. The existing `arm_controller` stays active to execute NMPC references. Add vision targets only after fixed-target tracking works.

CasADi will need to be installed in the Python environment actually used by the ROS executable. The standalone example's virtual environment is separate; merely activating it does not change an already installed executable's interpreter.

**Validation:** this section documents setup and provides subscriber source. ROS 2/Gazebo execution and NMPC integration have not been tested in this documentation update.


## Validation Status

This README update was checked against the current vision source, dependency declaration, and controller/bringup launch files. The previous README records XML parsing and static launch-structure checks. ROS 2/Gazebo was not run during this documentation update; compilation, controller activation, and complete grasp execution remain unverified here.

## Contributing

Suggestions and contributions are welcome for documentation, reproducible bug reports, vision and planning improvements, and simulation fixes. Describe the change, its relationship to the upstream implementation, and how it was validated.

## License

See the preserved [Apache License 2.0](LICENSE). Keep the original author's attribution and applicable notices when modifying or redistributing inherited source.

## Contact

Maintainer: **Yujie Tang** — [ytang19-glitch](https://github.com/ytang19-glitch).

For questions about this adapted repository, open an issue in [panda_franka_robot](https://github.com/ytang19-glitch/panda_franka_robot/issues). For the original implementation and its author, see the [reference repository](https://github.com/heimizhou1314/Franka-Panda-Robot-Project).
