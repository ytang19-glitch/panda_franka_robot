# FR3 Gazebo torque adapter (pure computation only)

The existing FR3 NMPC optimizer produces **total modeled joint torque**
`tau_total`, including model gravity. In the inspected Franka Gazebo hardware
implementation, `FrankaGazeboHardwareInterface::write()` adds gravity to an
existing effort command. The official gravity compensation example commands
zero effort. Under that **specific** interface convention:

```text
tau_applied ≈ tau_command + g_gazebo(q)
tau_command = tau_total - g_model(q)
```

This conversion is exact only if the modeled gravity matches the hardware's
gravity model and their joint ordering/signs/frames agree.

## Added files

- `include/fr3_nmpc_controller/torque_adapter.hpp`
- `src/torque_adapter.cpp`
- `test/test_torque_adapter.cpp`

`TorqueAdapter::to_gazebo_effort(total, gravity)` is a pure arithmetic
function. It checks all seven inputs and results are finite, but **does not**
enforce effort limits, rate limits, freshness, solve validity or model
equivalence. It can throw exceptions, so do not call it unguarded from a
hard-real-time control loop. No new motor commands are emitted by this change.

## Safely bring into a local branch with different NMPC files

Do not overwrite an older local `CMakeLists.txt` because it contains local
`offline_tracking` and derivative test definitions.

```bash
cd ~/panda_franka_robot
git status
git fetch origin
git restore --source=origin/main -- \
  src/fr3_nmpc_controller/include/fr3_nmpc_controller/torque_adapter.hpp \
  src/fr3_nmpc_controller/src/torque_adapter.cpp \
  src/fr3_nmpc_controller/test/test_torque_adapter.cpp \
  src/fr3_nmpc_controller/TORQUE_ADAPTER_GUIDE.md
```

Manually add `src/torque_adapter.cpp` to the existing
`add_library(${PROJECT_NAME} SHARED ...)` block in local CMake. Inside
`if(BUILD_TESTING)` add:

```cmake
ament_add_gtest(test_torque_adapter test/test_torque_adapter.cpp)
if(TARGET test_torque_adapter)
  target_link_libraries(test_torque_adapter ${PROJECT_NAME})
endif()
```

Then compile/test with the same previously successful CasADi configuration:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws_jazzy/install/local_setup.bash
cd ~/panda_franka_robot
colcon build --packages-select fr3_nmpc_controller --symlink-install \
  --cmake-args -DFR3_NMPC_WITH_CASADI=ON
colcon test --packages-select fr3_nmpc_controller --event-handlers console_direct+
colcon test-result --test-result-base build/fr3_nmpc_controller --verbose
```

## Future integration

First validate the hardware compensation convention, dynamic model, nominal
gravity and reference frames in Gazebo. Next implement safe non-real-time
handoff of solved torques to the effort controller, enforce both total and
command torque limits, and define a fault fallback. The existing
`Fr3NmpcController` activation guard stays enabled.
