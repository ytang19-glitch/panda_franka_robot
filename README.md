# panda_franka_robot

A Franka Panda vision-guided sorting simulation project for learning and further development, using ROS 2 Jazzy, Gazebo Harmonic, MoveIt 2, and OpenCV.

## Reference and acknowledgments

This project is based on [Franka-Panda-Robot-Project](https://github.com/heimizhou1314/Franka-Panda-Robot-Project) by **heimizhou1314 / zjs**. The original robot models, controller configuration, vision detection, MoveIt configuration, and pick-and-place program come from that project.

- Upstream baseline commit: `1eb59f4272fc6f98a4f828ec6a2c1f5b117edec4`.
- Original documentation: [UPSTREAM_README.md](UPSTREAM_README.md), preserved in its original language with the author's development notes and demonstration links. Use this README for this version's operating instructions.
- The original [Apache 2.0 LICENSE](LICENSE) and source attribution are preserved. The upstream documentation also retains the attribution for the Franka models.
- Changes in this version: rewritten setup instructions, removal of the extra standalone controller manager in Gazebo simulation, and removal of redundant controller activation commands when starting the task. Modified source files include change notices.

## 0. Start from the original project

The learning and development workflow is: **clone the original project → copy the six packages → reproduce the baseline → develop your own version**.

To study the original version, use a separate workspace:

```bash
cd ~
git clone https://github.com/heimizhou1314/Franka-Panda-Robot-Project.git
mkdir -p ~/panda_upstream_ws/src
cp -a ~/Franka-Panda-Robot-Project/src/. ~/panda_upstream_ws/src/
```

Here, `git clone` downloads the original repository, and `cp -a` copies its packages into a ROS 2 workspace. Follow the [upstream instructions](https://github.com/heimizhou1314/Franka-Panda-Robot-Project#readme) to build and run that version. This repository uses commit `1eb59f4272fc6f98a4f828ec6a2c1f5b117edec4` as its baseline; the clone command above retrieves the latest upstream version by default.

Steps 1–5 below run the `panda_franka_robot` version. It already includes the copied source and the startup fixes listed above. Do not overwrite those fixes with the original source. Use the workspaces separately and source only the workspace you intend to run.

## 1. Clone this repository

Run in an Ubuntu terminal:

```bash
cd ~
git clone https://github.com/ytang19-glitch/panda_franka_robot.git
cd ~/panda_franka_robot
```

If this repository is already cloned at that location, update it instead:

```bash
cd ~/panda_franka_robot
git pull --ff-only origin main
```

## 2. Copy the six packages

```bash
mkdir -p ~/panda_robot_ws/src
cd ~/panda_franka_robot
cp -a src/{panda_description,panda_controller,panda_moveit,panda_vision,panda_commander,panda_bringup} ~/panda_robot_ws/src/
ls ~/panda_robot_ws/src
```

Place all six packages directly under `~/panda_robot_ws/src/`. Do not put them inside another `panda_bringup` directory. The `cp -a` command overwrites files with matching names, so back up any local changes first. Avoid keeping duplicate copies of the same packages in the workspace because `colcon` may report duplicate package names.

| Package | Purpose |
| --- | --- |
| `panda_description` | URDF/Xacro, models, camera, and Gazebo scene |
| `panda_controller` | Controller configuration and spawners |
| `panda_moveit` | MoveIt 2 planning configuration and RViz |
| `panda_vision` | OpenCV color detection |
| `panda_commander` | Pick-and-place task program |
| `panda_bringup` | System and task launch files |

The source repository and runtime workspace are separate copies. After editing the repository, copy the relevant packages again and rebuild in the workspace.

## 3. Install dependencies and build

Prerequisites: Ubuntu 24.04, ROS 2 Jazzy, and a working graphical interface. Run these commands in the same environment that runs ROS. Docker users should run them inside the container and use their actual workspace path, such as `/panda_ws`.

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

Run `sudo rosdep init` only when setting up rosdep for the first time. Skip it if rosdep is already initialized.

```bash
rosdep update
cd ~/panda_robot_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

## 4. Start the simulation

Terminal A:

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_robot_ws/install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.xml
```

This launch file declares Gazebo, controller spawners, MoveIt/RViz, and the color detection node. Declaration order does not mean that each component waits for the previous component to finish initializing. The spawners wait for the controller manager inside Gazebo.

```xml
<launch>
    <!-- Gazebo simulation environment -->
    <include file="$(find-pkg-share panda_description)/launch/gazebo.launch.xml" />

    <!-- Robot controllers -->
    <include file="$(find-pkg-share panda_controller)/launch/controller.launch.xml" />

    <!-- MoveIt motion planning -->
    <include file="$(find-pkg-share panda_moveit)/launch/moveit.launch.py">
        <arg name="is_sim" value="True"/>
    </include>

    <!-- Color detection node -->
    <node pkg="panda_vision" exec="color_detector" name="color_detector" output="screen"/>
</launch>
```

Gazebo's `gz_ros2_control` plugin creates the simulation controller manager. The `controller.launch.xml` file starts only the three spawners; it does not start an additional `ros2_control_node`.

## 5. Check the controllers and run the task

Terminal B:

```bash
source /opt/ros/jazzy/setup.bash
source ~/panda_robot_ws/install/setup.bash
ros2 node list
ros2 control list_controllers -c /controller_manager
```

Expect a single `/controller_manager`, with `joint_state_broadcaster`, `arm_controller`, and `gripper_controller` all `active`. If the controllers are not ready, inspect the Gazebo plugin and launch logs first.

```bash
ros2 launch panda_bringup pick_and_place_commander.launch.xml target_color:=R
```

Use `R`, `G`, or `B` for red, green, or blue. The task launch no longer repeats `set_controller_state ... active` commands. Start the task manually after the controllers are ready. Press `Ctrl+C` after a task finishes before starting another run.

## 6. Develop your own work

This repository supports Yujie Tang's learning and further independent development. Existing changes are listed in the acknowledgments above. The following items are future plans, not completed features:

- Reproduce color detection and pick-and-place behavior, recording failure cases.
- Improve object localization, grasp poses, and task state management.
- Add failure detection and retries; compare grasp success rate, localization error, and task duration.
- After establishing a stable baseline, explore visual servoing, trajectory optimization, or NMPC.

Edit source files in the repository, then copy them into the workspace to run them. For example, when modifying the vision node:

```bash
cd ~/panda_franka_robot
git switch -c feature/vision-improvements
# Edit src/panda_vision/panda_vision/color_detector.py here.
cp -a src/panda_vision ~/panda_robot_ws/src/
cd ~/panda_robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select panda_vision
source install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.xml
```

Stop the previous simulation before launching another instance. After validation, commit your actual changes and push them to your GitHub branch:

```bash
cd ~/panda_franka_robot
git diff
git add src/panda_vision/panda_vision/color_detector.py
git commit -m "Improve color detection"
git push -u origin feature/vision-improvements
```

Record the reason for each change and its validation results. Preserve the original author's attribution and license, and include change notices in modified upstream files.

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
- **Source changes have no effect:** copy the changes from the repository into the workspace, rebuild, and source the workspace. Use `ros2 pkg prefix panda_controller` to confirm which installation is being loaded.

## Validation status

The launch files have undergone XML parsing and static launch-structure checks. ROS 2/Gazebo was not run in the preparation environment, so compilation, controller activation, and complete grasp execution remain unverified. The upstream demonstration does not establish runtime validation for this version.
