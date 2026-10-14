# FR3 Cartesian approach flipping: target orientation and current TCP pose

## Symptom and observed result

The approach to a linear path took a large detour and the end effector flipped, while the subsequent Cartesian segment looked straight. The user reported that explicitly supplying an orientation close to the measured TCP orientation made the motion behave as expected.

The generator defaults to quaternion (xyzw) = (0, 0, 0, 1) when orientation parameters are omitted. This is identity orientation relative to the path frame, not a universal gripper-down orientation.

The original comparison also changed X and path length. It supports orientation as the main suspected cause, but a controlled comparison should keep all XYZ parameters unchanged and change only orientation. Reaching a target does not establish trajectory safety.

## Read the current TCP pose

Run in the FR3 environment:

```bash
export ROS_DOMAIN_ID=20
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

Observed sample:

```text
Translation: [0.333, -0.044, 0.467]
Quaternion (xyzw): [-0.014, 1.000, -0.017, 0.011]
Matrix:
 -0.999 -0.028  0.023  0.333
 -0.029  0.999 -0.033 -0.044
 -0.022 -0.034 -0.999  0.467
  0.000  0.000  0.000  1.000
```

This quaternion is close to (0, 1, 0, 0), a 180-degree rotation about Y relative to identity. The measured orientation differs from identity by approximately 179 degrees. The third rotation-matrix column [0.023, -0.033, -0.999] describes the TCP +Z axis expressed in the base frame: it points nearly toward base -Z. The physical gripper direction also depends on the TCP/tool definition.

**The last matrix row (0, 0, 0, 1) is NOT the quaternion.** It is the fixed homogeneous-coordinate row in T = [R p; 0 1]. Read orientation from the explicitly labeled Quaternion line or the upper-left 3-by-3 rotation matrix. Translation is the upper-right column.

## Working reference-path example

Stop the previous generator first. In the generator workspace, generate the user-reported example:

```bash
export ROS_DOMAIN_ID=10
ros2 run panda_reference_trajectories reference_generator --ros-args \
  -p frame_id:=fr3_link0 \
  -p path_topic:=/panda_reference/cartesian_path \
  -p path_type:=linear \
  -p orientation_x:=-0.010 \
  -p orientation_y:=1.000 \
  -p orientation_z:=-0.011 \
  -p orientation_w:=0.016 \
  -p start_x:=0.334 \
  -p start_y:=-0.15 \
  -p start_z:=0.25 \
  -p goal_x:=0.334 \
  -p goal_y:=0.044 \
  -p goal_z:=0.25 \
  -p step_size:=0.005
```

This is a 19.4 cm geometric path along base +Y with constant orientation; it does not prescribe a speed or duration. The executor must produce a timed joint trajectory. Parameter overrides require no rebuild.

The example quaternion is an earlier measured sample, not a permanent calibration constant. For a current-orientation test, obtain a fresh sample and use its values. Copying is valid only when the measured reference frame and TCP match the path frame and executor end-effector link.

The supplied generator normalizes the quaternion, then assigns goal.orientation = start.orientation. Normalization enforces unit magnitude; it does not select or correct the desired physical orientation. Confirm intermediate generated poses retain the intended orientation.


## Generator code review: source of the orientation request

The complete generator supplied during debugging declares these defaults:

```cpp
start.orientation.x = declare_parameter<double>("orientation_x", 0.0);
start.orientation.y = declare_parameter<double>("orientation_y", 0.0);
start.orientation.z = declare_parameter<double>("orientation_z", 0.0);
start.orientation.w = declare_parameter<double>("orientation_w", 1.0);
Pose goal = start;
```

Omitting all four orientation arguments requests identity orientation unless a parameter file or another override supplies different values. It does not mean “keep the current orientation.” Pose goal = start copies both position and orientation; subsequent goal.position assignments only change position.

After validation, the code normalizes the quaternion and explicitly copies it to the goal:

```cpp
start.orientation.x /= qnorm;
start.orientation.y /= qnorm;
start.orientation.z /= qnorm;
start.orientation.w /= qnorm;
goal.orientation = start.orientation;
```

This is valid quaternion normalization. It preserves the requested rotation and does not repair an unintended target direction. Negating every quaternion component also does not change the physical rotation.

| Component | Responsibility |
|---|---|
| Reference generator | Selects requested start/goal poses and publishes geometric waypoints |
| generateLinearPath | Implements waypoint interpolation; its implementation must be inspected separately |
| Cartesian executor / MoveIt | Converts waypoints to joint motion and plans the approach |
| Current robot state | Supplies the actual starting joint configuration |

The generator contributes the orientation request; it does not calculate the approach detour. A pose target can require a substantial wrist rotation and a different elbow configuration. A straight TCP path also does not imply straight or small movements of every joint.

The shown node has no TF listener or joint-state subscription. It therefore cannot automatically preserve the current TCP orientation. For now, pass the desired orientation explicitly rather than replacing the generic default with a measurement from one robot session.

## Controlled orientation comparison

Keep the same XYZ coordinates, step size, starting robot state, and executor settings. Compare an identity target with the measured-orientation target. The identity preview can be generated by the following command:

```bash
export ROS_DOMAIN_ID=10
ros2 run panda_reference_trajectories reference_generator --ros-args \
  -p frame_id:=fr3_link0 \
  -p path_topic:=/panda_reference/cartesian_path \
  -p path_type:=linear \
  -p start_x:=0.40 \
  -p start_y:=-0.15 \
  -p start_z:=0.25 \
  -p goal_x:=0.40 \
  -p goal_y:=0.15 \
  -p goal_z:=0.25 \
  -p orientation_x:=0.0 \
  -p orientation_y:=0.0 \
  -p orientation_z:=0.0 \
  -p orientation_w:=1.0 \
  -p step_size:=0.005
```

For the second preview, replace only the four orientation values with the latest measured sample:

```text
orientation_x: -0.014
orientation_y:  1.000
orientation_z: -0.017
orientation_w:  0.011
```

These are recorded example values; refresh them if the robot state has changed. Stop the old generator before launching the replacement. Keep execute=false and replan for each comparison. Check the published path, rather than assuming the new command reached the executor.

If the flip disappears, orientation is supported as the main cause. If a large approach detour remains, inspect the executor's start state, end-effector link, inverse-kinematics configuration, planner and collision constraints. This document does not claim that the unseen executor or interpolation implementation has been verified.

## Why the current pose matters

The geometric start pose and robot's current pose are separate inputs. For the latest sample, the example start is approximately 24.2 cm away and 21.7 cm lower. Thus it still requires an approach movement even though orientation is similar.

- Current TCP pose helps choose an appropriate target orientation and measure the approach displacement.
- Current joint state is needed by the planner: the same TCP pose can correspond to different configurations of a redundant 7-joint arm.
- The generator does not automatically read TF in the posted code. Values copied into command arguments are static snapshots.
- An approach planned to a pose target need not follow a Cartesian straight line.
- Matching endpoint orientation does not guarantee constant orientation throughout the approach or eliminate every detour.
- After any robot movement, refresh the state before planning again.

For diagnosis, first use a short path starting close to a freshly measured current pose, with that orientation fixed. Then compare identical XYZ paths with different target orientations in RViz.

## Domain bridge and RViz checks

Observed topic mapping:

| Domain | Topic |
|---|---|
| 10: generator | /panda_reference/cartesian_path |
| 20: bridge output | /fr3_reference/cartesian_path |

The configured remap changes the topic name; the bridge does not transform coordinates.

```bash
export ROS_DOMAIN_ID=20
ros2 topic echo /fr3_reference/cartesian_path nav_msgs/msg/Path \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --once
```

Path reception in Domain 20 was confirmed in this session. Fast DDS shared-memory initialization errors were also printed, but did not prevent this particular reception; their root cause was not established.

RViz Path display should subscribe to /fr3_reference/cartesian_path, use Reliable and Transient Local to receive a retained path, and use fr3_link0 as Fixed Frame. A Volatile subscriber can receive new publications but will not request the retained historical sample. The executor must also subscribe to the remapped topic.

The full generator code confirms a 1-second wall timer repeatedly republishes the path with refreshed header timestamps:

```cpp
timer_ = create_wall_timer(1s, [this]() {
  const auto stamp = now();
  path_.header.stamp = stamp;
  for (auto & pose : path_.poses) pose.header.stamp = stamp;
  publisher_->publish(path_);
});
```

Thus, while this generator and bridge remain running, a compatible Volatile RViz subscriber should receive a later publication. Missing historical delivery alone cannot explain a persistently invisible path in that situation. Check the remapped topic, RViz domain, Path Status/TF, visibility and actual message arrival. The refreshed timestamps are publication times, not waypoint execution times; the geometry stays unchanged. Executor handling of repeated messages must be inspected before attributing repeated planning to this timer.

## Validation boundary

Keep execute=false for diagnostic previews. Verify the actual published orientation, current start state, approach/Cartesian continuity, collisions, joint limits, and time parameterization before considering physical execution. This note records user observations and command/message inspection; it is not independent hardware validation.
