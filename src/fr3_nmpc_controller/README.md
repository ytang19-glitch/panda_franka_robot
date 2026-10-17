# FR3 torque NMPC controller — development scaffold

This ROS 2 Jazzy package establishes the C++ file structure for a future
**dynamics-based nonlinear model predictive controller** for the real Franka FR3.

**It is not an implemented NMPC controller.** The model, optimizer and worker
are explicit placeholders. The controller plugin can be built and discovered
in a compatible ROS installation, but **activation always returns ERROR** and
`update()` never writes commands. There is no parameter to enable motion.
Do not deactivate your working arm controller to try this scaffold.

## Verified hardware contract

These observations come from the user's real FR3 inspection on 2026-10-16,
not from a hardware test of this package.

| Item | Observed value | Design implication |
| --- | --- | --- |
| Current arm controller | `fr3_arm_controller`, `JointTrajectoryController`, active | Preserve as the baseline during development |
| Command interface | `effort` for `fr3_joint1` through `fr3_joint7` | Future plugin outputs seven joint torque commands |
| State interfaces | `position`, `velocity` | State contains seven angles and seven angular velocities |
| Configured update rate | 1000 Hz, synchronous | Requested 1 ms controller period; not measured solver performance |
| Effort ownership | All seven claimed by `fr3_arm_controller` | Future NMPC activation requires coordinated ownership transfer |
| Other state interfaces | `fr3/robot_model`, `fr3/robot_state`, `fr3/robot_time` | Optional access through matching Franka semantic components |

The original logs and extended design notes remain in [`src/q`](src/q).
The [`command`](command) file contains read-only inspection commands; it is not
a controller launcher or an installation script.

## File structure

```text
fr3_nmpc_controller/
├── include/fr3_nmpc_controller/
│   ├── robot_model.hpp
│   ├── optimizer.hpp
│   ├── controller.hpp
│   └── solver_worker.hpp
├── src/
│   ├── robot_model.cpp
│   ├── optimizer.cpp
│   ├── controller.cpp
│   ├── solver_worker.cpp
│   └── q
├── config/
│   └── nmpc_controller.yaml
├── test/
│   └── test_unimplemented_backends.cpp
├── fr3_nmpc_controller_plugins.xml
├── CMakeLists.txt
├── package.xml
├── command
└── README.md
```

## Meaning of each file

| File | Role | Implemented now / next work |
| --- | --- | --- |
| [`robot_model.hpp`](include/fr3_nmpc_controller/robot_model.hpp) | Defines seven-joint vectors, measured state and dynamics API | Types and API exist; no robot model is loaded |
| [`robot_model.cpp`](src/robot_model.cpp) | Future forward dynamics at predicted states | `ready()` returns false; acceleration throws instead of inventing dynamics |
| [`optimizer.hpp`](include/fr3_nmpc_controller/optimizer.hpp) | Solver API and result format | Result includes total torque, state time, expiry, reference ID and convergence flag; timestamps are monotonic |
| [`optimizer.cpp`](src/optimizer.cpp) | Future horizon, objective, constraints and warm start | Returns no solution; no CasADi/IPOPT solver exists yet |
| [`solver_worker.hpp`](include/fr3_nmpc_controller/solver_worker.hpp) | Boundary between solver worker and hardware loop | Interface placeholder; no thread-safe exchange is implemented |
| [`solver_worker.cpp`](src/solver_worker.cpp) | Future solver thread, scheduling and teardown | No thread starts; no result is published |
| [`controller.hpp`](include/fr3_nmpc_controller/controller.hpp) | Declares the ROS 2 controller lifecycle and update methods | Declares `Fr3NmpcController` plugin class |
| [`controller.cpp`](src/controller.cpp) | Hardware interface requests and lifecycle | Validates exact FR3 joint order, declares effort/q/dq interfaces, always refuses activation and performs no writes |
| [`nmpc_controller.yaml`](config/nmpc_controller.yaml) | Controller-manager registration overlay and joint order | Does not switch or start a controller; only `joints` is currently consumed |
| [`test_unimplemented_backends.cpp`](test/test_unimplemented_backends.cpp) | Regression check against fictitious actuator commands | Checks missing backend readiness, model rejection and absent solver/worker outputs |
| [`fr3_nmpc_controller_plugins.xml`](fr3_nmpc_controller_plugins.xml) | Maps plugin name to C++ class and shared library | Enables plugin discovery through `pluginlib` |
| [`CMakeLists.txt`](CMakeLists.txt) | Builds library, exports plugin, installs headers/config and registers tests | Uses C++17 and `ament_cmake` |
| [`package.xml`](package.xml) | Package identity and currently used dependencies | Declares ROS controller and test dependencies |
| [`src/q`](src/q) | Hardware evidence and implementation plan | Documentation, not compiled code |
| [`command`](command) | Original inspection commands | Read-only CLI commands |
| [`README.md`](README.md) | Package status, architecture and development instructions | This guide |

Headers (`.hpp`) declare types and interfaces. Source files (`.cpp`) implement
them. This separation lets the model and solver be tested independently of
hardware integration.

## Intended model and optimization

The future state and control are:

$$
x = \begin{bmatrix}q\\v\end{bmatrix},\qquad u=\tau_{\mathrm{total}}.
$$

For nominal free-space prediction:

$$
\dot q=v,\qquad
M(q)\dot v = \tau_{\mathrm{total}}-c(q,v)-g(q).
$$

Here `c(q,v)` is the Coriolis/centrifugal torque vector. Numerical integration
produces `x[k+1] = F(x[k], u[k])`. Use a validated FR3 model with correct joint
order, inertial data and tool/payload definition. The existing Python synthetic
joint simulation is a learning example and cannot substitute for this model.

The future objective will penalize joint position/velocity tracking error,
control effort and command variation, subject to joint, torque and torque-rate
limits. Limits must match the chosen total-versus-command torque convention.
Do not copy placeholder limits or assume adding a nonlinear model provides
payload robustness, collision avoidance or stability guarantees.

The installed Franka driver's gravity-compensation convention must be checked
before defining the effort mapping. If the driver adds gravity, a model using
total torque needs a consistent conversion. The scaffold does not implement
that mapping or any torque output.

## Intended runtime architecture

1. The controller update loop reads q/dq with timestamps.
2. A separate solver worker receives a consistent state snapshot and timed reference.
3. The worker solves the finite-horizon problem and labels the result with its
   reference ID, source state time, validity and convergence information.
4. The hardware loop accepts only valid results, accounts for latency, enforces
   torque/rate limits and writes commands through its claimed effort interfaces.
5. Tested failure behavior handles solver failure, stale results, invalid state,
   activation, completion and deactivation.

These are future implementation requirements. No worker thread or exchange is
implemented yet. Blocking solves and ordinary ROS publication do not belong in
the real-time update path. A lower-frequency solver with a faster controller
loop needs an explicit command-selection/interpolation and expiry policy;
holding the last torque or writing zero is not automatically safe.

## Dependencies

| Stage | Dependencies | Purpose |
| --- | --- | --- |
| Scaffold build | `ament_cmake`, `controller_interface`, `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle` | Build and register the inactive controller plugin |
| Scaffold tests | `ament_cmake_gtest` | Backend rejection checks |
| Future model | Eigen3, Pinocchio, validated FR3 model | Linear algebra and predicted rigid-body dynamics |
| Future optimizer | CasADi with IPOPT | NLP formulation, derivatives and solution |
| Future worker/reference | `realtime_tools`, `trajectory_msgs`, threading support | Data exchange and timed reference handling |
| Hardware integration | Compatible `franka_ros2` and `libfranka`; optionally `franka_semantic_components` | Existing driver and measured-state model access |
| Optional extensions | HPP-FCL, vision components | Collision distances and visual feedback |

Future dependencies are not linked or declared as working features today.
Crocoddyl and qpOASES are alternatives, not required additions to CasADi/IPOPT.
A numeric Pinocchio backend does not automatically provide a differentiable
CasADi graph; validate the selected symbolic/code-generation integration.

## Build and test on your Jazzy computer

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fr3_nmpc_controller --symlink-install
source install/setup.bash
colcon test --packages-select fr3_nmpc_controller
colcon test-result --verbose
```

No robot connection is needed to build this scaffold. These commands do not
load, activate or switch hardware controllers. The package YAML is a
registration overlay; merge it deliberately when future integration is ready,
without replacing the existing controller configuration. No hardware launch
or controller-switch commands are provided.

Local validation for this change: the three ROS-independent C++ sources were
compiled and linked with a standalone rejection check; XML/YAML and local
README links were checked. ROS 2 is unavailable in the authoring environment,
so the complete Jazzy library/plugin build and gtest execution remain untested.

## Relationship to your existing project

| Component | Role |
| --- | --- |
| `panda_reference_trajectories` | Generate Cartesian geometry and plan/time-parameterize joint motion |
| `panda_nmpc` | Python optimizer learning and synthetic simulation |
| `fr3_nmpc_controller` | Future C++ FR3 effort-controller integration; scaffold only today |
| Existing `fr3_arm_controller` | Working baseline trajectory controller |

A `nav_msgs/Path` is geometric and cannot directly supply a timed joint MPC
reference. Define an execution-synchronized `JointTrajectory` input. Approach
and Cartesian segments need explicit sequencing; the existing Python bridge
rejects multi-segment display plans. Bridge receipt is not proof of execution
synchronization.

Keep Domain 10/20 as your chosen simulation/hardware organization, but domain
numbers do not provide actuation protection. During future NMPC trials MoveIt
may provide a reference, while only the NMPC plugin owns arm commands. During
baseline trials the existing trajectory controller owns them. Keep broadcasters
for measurement in both trials.

## Development milestones

1. Implement and validate FR3 model quantities and integration offline.
2. Implement NMPC with measured solve times, feasibility checks and constraints.
3. Implement timed reference handling and bounded worker/loop data exchange.
4. Validate command conventions, limits, lifecycle and failure behavior in a
   compatible effort-based dynamics simulation.
5. Implement and review the hardware command path, then follow laboratory
   procedures for FR3 integration and controlled controller switching.
6. Compare separate baseline/NMPC runs using the same timed reference, initial
   condition, payload and speed. Record measured errors, commands, solver time,
   solution age and deadline misses.

There is no completed NMPC or hardware-performance claim at this stage.

## Official references

- [ROS 2 Jazzy controller interface example](https://control.ros.org/jazzy/doc/api/classros2__control__demo__example__7_1_1RobotController.html)
- [Jazzy realtime_tools](https://control.ros.org/jazzy/doc/realtime_tools/doc/index.html)
- [Franka ROS 2 hardware documentation](https://frankarobotics.github.io/docs/doc/franka_ros2_jazzy/franka_hardware/doc/index.html)
- [Franka ROS 2 example controllers](https://frankarobotics.github.io/docs/doc/franka_ros2_jazzy/franka_example_controllers/doc/index.html)
