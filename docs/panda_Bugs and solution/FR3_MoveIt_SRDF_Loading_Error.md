# FR3 MoveIt 2: Cartesian Executor Cannot Load SRDF

**Status:** Diagnosed; fix not yet verified on hardware  
**Environment:** Ubuntu 24.04, ROS 2 Jazzy, MoveIt 2, Franka FR3  
**Pipeline:** Panda reference generator (ROS_DOMAIN_ID=10) → domain_bridge → FR3 Cartesian executor (ROS_DOMAIN_ID=20)

## Symptom

The Cartesian path arrives successfully:

```text
[INFO] [cartesian_path_executor]: Received 61 waypoints in fr3_link0
[ERROR] [cartesian_path_executor]: Could not find parameter robot_description_semantic and did not receive robot_description_semantic via std_msgs::msg::String subscription within 10.000000 seconds.
Error: Could not parse the SRDF XML File. Error=XML_ERROR_EMPTY_DOCUMENT
[FATAL] [move_group_interface]: Unable to construct robot model.
terminate called without an active exception
[ros2run]: Aborted
```

This is **not a domain_bridge failure**: 61 path poses were received on Domain 20. The failure happens when `MoveGroupInterface` constructs its local robot model.

## Root cause and reasoning

1. `robot_description` contains URDF (links, joints, geometry).
2. `robot_description_semantic` contains SRDF (planning groups, end-effectors, named poses, collision allowances).
3. The existing `/move_group` node can have these parameters, but ROS 2 parameters are **node-local**; a newly launched `/cartesian_path_executor` does not automatically inherit them.
4. The executor's RDF loader also tried to receive `robot_description_semantic` as a `std_msgs/msg/String` topic and timed out. Only `/robot_description` was visible during the reported test.
5. This caused the SRDF parser to read an empty document, so the robot model could not be constructed.
6. The `terminate called without an active exception` may be a separate C++ lifecycle bug: a joinable `std::thread` destroyed while an exception unwinds calls `std::terminate`. Fix thread cleanup independently of SRDF loading.

**Confirmed from the FR3 SRDF printed on the running system:**

```xml
<group name="fr3_arm">
  <chain base_link="fr3_link0" tip_link="fr3_hand_tcp"/>
</group>
```

Use `planning_group:=fr3_arm`. The source path already carries `frame_id: fr3_link0`. Matching names does **not** prove each pose is reachable or collision-free.

## Another, unrelated error: workspace path does not exist

```text
find: ‘/home/yujietang/franka_ros2_jazzy/src’: No such file or directory
```

This only says that the guessed path `~/franka_ros2_jazzy/src` does not exist. The package `franka_fr3_moveit_config` **is installed/discoverable**. Do not assume a source workspace location. Locate the active package with:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=20
ros2 pkg prefix franka_fr3_moveit_config
ros2 pkg prefix --share franka_fr3_moveit_config
find "$(ros2 pkg prefix --share franka_fr3_moveit_config)" -type f \
  \( -name '*.srdf' -o -name '*.srdf.xacro' -o -name 'kinematics.yaml' \)
```

The share directory may contain installed configs, generated files, or xacro sources; inspect the actual launch configuration before choosing how to load them.

## Diagnostic commands (safe: no robot motion)

In a terminal that can discover the running FR3 in Domain 20:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=20

ros2 param list /move_group | grep robot_description
ros2 param get /move_group robot_description_semantic
ros2 param get /move_group robot_description
ros2 param list /move_group | grep -E 'kinematics|robot_description'

ros2 pkg prefix --share franka_fr3_moveit_config
ros2 node list | grep move_group
ros2 topic info /fr3_reference/cartesian_path -v
ros2 topic echo /fr3_reference/cartesian_path --once
```

Do not assume that the presence of `/move_group` means the executor shares its model parameters. If `/move_group` also lacks a non-empty SRDF, fix the FR3 MoveIt launch first.

## Preferred fix: launch the Cartesian Executor with the same FR3 model configuration

**Do not change the Generator or Domain Bridge for this issue.** Create an FR3-specific ROS 2 launch file for the executor, using the same actual URDF, SRDF and kinematics YAML as the working `franka_fr3_moveit_config` launch. The executor must receive:

- `robot_description` (URDF XML)
- `robot_description_semantic` (SRDF XML)
- `robot_description_kinematics` (IK solver parameters)
- `path_topic: /fr3_reference/cartesian_path`
- `planning_group: fr3_arm`
- `execute: false` (planning-only test)

Use package share paths discovered above, not the guessed source path. URDF/SRDF may be generated via xacro and may require launch arguments. Avoid dumping all parameters from `/move_group` into the executor unchanged: some are node-specific and may conflict.

Alternatively, configure supported robot-description topic publication in the FR3 MoveIt launch with matching QoS, but a dedicated executor launch with explicit parameters is easier to reproduce.

## Fix the C++ failure path separately

If the executor starts a `std::thread` spinning ROS callbacks, ensure every exit path cancels the executor and joins the thread **before** the thread object is destroyed. Prefer RAII cleanup or a `rclcpp::executors::MultiThreadedExecutor` with carefully managed lifetime. Merely adding an exception handler around model construction may not be enough if a joinable thread unwinds through the exception.

## Verification sequence

1. Confirm installed FR3 config location via `ros2 pkg prefix --share franka_fr3_moveit_config`.
2. Verify `/move_group` has a non-empty SRDF and a valid URDF.
3. Launch the executor with **the same** complete robot description and IK config, `execute:=false`.
4. Confirm no SRDF loader error and that the RobotModel loads with group `fr3_arm` and TCP `fr3_hand_tcp`.
5. Confirm the 61-point path arrives.
6. Check MoveIt IK, collision checking, approach planning, Cartesian completion fraction, and trajectory timing; display the result in RViz. Planning success does not by itself mean RViz animation is published.
7. Only after explicit physical safety review and a verified controller-compatible time-parameterized trajectory should hardware execution be considered.

## Safety boundary

A received `nav_msgs/Path` is **geometric**, not a time-parameterized executable joint trajectory. No direct torque/trajectory control commands should be bridged to the real robot; keep `execute:=false` through these debugging steps. Do not assume `fr3_link0` labels alone perform TF2 transformations.

## Expected next result

```text
Domain 10: generator produces 61 Cartesian poses
  -> Domain Bridge (one approved Path topic)
Domain 20: executor receives the 61 poses
  -> MoveIt RobotModel loads FR3 URDF/SRDF/kinematics
  -> computeCartesianPath() (planning-only)
  -> inspect fraction, feasibility and RViz preview
```
