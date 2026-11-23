# Gazebo → NMPC read-only diagnostic

This package includes `src/gazebo_nmpc_diagnostic.cpp`, a **read-only**
ROS 2 node for validating joint state ingestion and NMPC optimization.
It **does not send torque commands** and does not replace the disabled
`Fr3NmpcController` ros2_control plugin.

## Build after pulling this feature branch

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws_jazzy/install/setup.bash
cd ~/panda_franka_robot
colcon build --packages-select fr3_nmpc_controller --symlink-install \
  --cmake-args -DFR3_NMPC_WITH_CASADI=ON
source install/setup.bash
ros2 pkg executables fr3_nmpc_controller
```

If your local CasADi install needs an explicit `-Dcasadi_DIR=...`,
reuse the path from your previously successful offline build. On `main`
the CMake target is conditionally created **only** with CasADi ON.

## Launch Gazebo (terminal 1)

`fr3_sim_bringup` launches Gazebo + RViz but does not activate a hold
controller. Without effort control, the robot might sag under gravity.
For read-only NMPC evaluation, use a **separately verified stable**
simulation controller, e.g. the official gravity compensation example;
do not confuse its torques with NMPC torques.

With the simulation in ROS_DOMAIN_ID=10, inspect
`ros2 topic echo /joint_states --once` first.

## Run diagnostic (terminal 2)

Use the seven-joint reduced **FR3** model with correct inertias and limits;
the file below existed on the development machine in earlier offline tests:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws_jazzy/install/setup.bash
source ~/panda_franka_robot/install/setup.bash
export ROS_DOMAIN_ID=10
test -f /tmp/fr3_arm_only.urdf || echo "Missing FR3 reduced URDF"
ros2 run fr3_nmpc_controller gazebo_nmpc_diagnostic --ros-args \
  -p model_urdf_path:=/tmp/fr3_arm_only.urdf
```

Expected messages after a valid joint-state sample: captured hold reference,
then `NMPC read-only: solve=... violation=... candidate_tau1=... (NOT applied)`.
The first solve may be slower than subsequent solves. A candidate torque is
an optimization diagnostic, not a command.

**Model caution:** A URDF previously used for `offline_tracking` is not
automatically identical to the Gazebo loaded FR3. Verify the mass/inertia,
gravity, payload, base frame and torque convention before interpreting results.

## Integrating with an older local NMPC branch

The local development branch may contain uncommitted `offline_tracking.cpp`
and an independent CMake configuration. Do **not** replace all local NMPC
files with GitHub `main`, and do not blindly apply an old stash.

On a clean *integration* branch, fetch the feature ref and copy only:
- `src/fr3_nmpc_controller/src/gazebo_nmpc_diagnostic.cpp`
- necessary `CMakeLists.txt` changes (merge manually)
- `package.xml` sensor_msgs dependency (merge manually)

If you want to copy only the new source:

```bash
git fetch origin
git restore --source=origin/feat/gazebo-nmpc-readonly-diagnostic -- \
  src/fr3_nmpc_controller/src/gazebo_nmpc_diagnostic.cpp
```

Then merge these CMake lines into your existing CMake file:

```cmake
find_package(sensor_msgs REQUIRED)
if(FR3_NMPC_WITH_CASADI)
  add_executable(gazebo_nmpc_diagnostic src/gazebo_nmpc_diagnostic.cpp)
  target_compile_features(gazebo_nmpc_diagnostic PRIVATE cxx_std_17)
  target_link_libraries(gazebo_nmpc_diagnostic ${PROJECT_NAME})
  ament_target_dependencies(gazebo_nmpc_diagnostic rclcpp sensor_msgs)
  install(TARGETS gazebo_nmpc_diagnostic DESTINATION lib/${PROJECT_NAME})
endif()
```

Add `<depend>sensor_msgs</depend>` to `package.xml`. Preserve the
existing `offline_tracking` executable and existing custom CasADi settings.

## Limitations

- Uses a stationary hold reference captured from first valid state.
- Samples diagnostic input at 4 Hz, not a real-time control deadline.
- The `previous_total_torque` seed is nominal model gravity, not actual
  command history; do not treat this as a valid closed-loop controller.
- ROS feedback is checked for finite values and sample age at reception,
  but this is not a fully synchronized state estimator.
- No explicit motion commands; no controller switching.
- If solving fails or times out, inspect `SolverWorker` errors, model limits,
  the actual seven-joint URDF and the difference between simulation and model.
