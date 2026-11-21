# FR3 simulation bringup (ROS 2 Jazzy)

Simulation-only wrapper for the **official Franka Gazebo** model, plus RViz,
clock bridge and joint state broadcaster. No arm controller is activated by
this package. It does **not** implement or activate NMPC and never connects to
real hardware.

## Why this package?

The upstream `gazebo_franka_arm_example_controller.launch.py` automatically
spawns a motion controller (default gravity compensation; selecting
`joint_impedance_example_controller` can move joints 4 and 5 periodically).
For controlled experiments, start Gazebo without a motion example.

## Install

The existing Franka workspace must contain
`franka_gazebo_bringup` and `franka_description`. On Ubuntu with Jazzy:

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws_jazzy/install/setup.bash
colcon build --packages-select fr3_sim_bringup --symlink-install
source install/setup.bash
```

## Start: Terminal 1 (simulation domain 10)

Stop any previous Panda or FR3 Gazebo launch first.

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws_jazzy/install/setup.bash
source ~/panda_franka_robot/install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch fr3_sim_bringup fr3_gazebo.launch.py rviz:=true
```

For a headless RViz-free test, use `rviz:=false`.

## Inspect: Terminal 2

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws_jazzy/install/setup.bash
source ~/panda_franka_robot/install/setup.bash
export ROS_DOMAIN_ID=10
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

Expected: `joint_state_broadcaster` active; no arm motion controller loaded.
FR3 joint command interfaces can include `effort`, but should be *unclaimed*.
The joint states should include valid `fr3_joint1` ... `fr3_joint7`
positions and velocities.

**Physics warning:** with no active effort controller, a gravity-enabled robot
may sag or fall under gravity; this is a connectivity test, **not** a gravity
hold controller. Pause physics if you want to freeze the configuration. Do not
activate a disabled/incomplete NMPC controller, or command constant/zero torques
expecting a stable hold.

## Next integration milestones

1. Implement a Gazebo-only, fail-safe controller for gravity hold using a
   verified *matching* FR3 dynamics model and appropriate torque convention.
2. Implement bounded state/command handoff to the existing
   `fr3_nmpc_controller` `SolverWorker` without IPOPT in the 1 kHz
   `update()` callback.
3. Test hold and a tiny joint move; log solver deadline misses and joint tracking.
4. Connect MoveIt-produced, *timed* joint references. Never execute a
   JointTrajectoryController and NMPC against the same effort interfaces.

The existing `fr3_nmpc_controller` intentionally returns ERROR during
activation; nothing in this package changes that protection.
