# FR3 numerical dynamics and offline torque NMPC

This package now includes a numerical Pinocchio robot model and an optional
CasADi/IPOPT multiple-shooting optimizer. **It is still not a working hardware
controller.** The ROS controller plugin continues to refuse activation, the
worker is a placeholder, and no commands are written to hardware.

`ready()` means initialized, not experimentally validated or hardware-safe.

## Files and responsibilities

| File | Responsibility / current implementation |
| --- | --- |
| [`robot_model.hpp`](include/fr3_nmpc_controller/robot_model.hpp) | Seven-joint state, limits, mass matrix and model API |
| [`robot_model.cpp`](src/robot_model.cpp) | Load URDF; compute mass, gravity, nonlinear effects, acceleration and RK4 step |
| [`optimizer.hpp`](include/fr3_nmpc_controller/optimizer.hpp) | Explicit solver configuration, reference horizon and offline result |
| [`optimizer.cpp`](src/optimizer.cpp) | Optional CasADi/IPOPT solve, finite-difference model callback, warm start and feasibility checks |
| [`controller.hpp`](include/fr3_nmpc_controller/controller.hpp), [`controller.cpp`](src/controller.cpp) | ROS 2 effort/q/dq interface declarations; activation disabled, no writes |
| [`solver_worker.hpp`](include/fr3_nmpc_controller/solver_worker.hpp), [`solver_worker.cpp`](src/solver_worker.cpp) | Placeholder only; no thread, command selection or validity policy |
| [`check_model.cpp`](examples/check_model.cpp) | Offline model-loading and gravity-equilibrium consistency check; no robot connection |
| [`test_robot_model.cpp`](test/test_robot_model.cpp) | Model equilibrium, invalid input and failed-reload tests using synthetic fixture |
| [`test_optimizer.cpp`](test/test_optimizer.cpp) | Optional nominal solve, rate-bound, warm-start and NaN rejection tests |
| [`test_unimplemented_backends.cpp`](test/test_unimplemented_backends.cpp) | Unloaded model/optimizer and unimplemented worker must not produce a command |
| [`seven_joint_test.urdf`](test/fixtures/seven_joint_test.urdf) | Synthetic test geometry/inertia; NOT an FR3 model |
| [`nmpc_controller.yaml`](config/nmpc_controller.yaml) | Registration overlay; only joint-order parameter is consumed by controller |
| [`fr3_nmpc_controller_plugins.xml`](fr3_nmpc_controller_plugins.xml) | Plugin discovery |
| [`CMakeLists.txt`](CMakeLists.txt), [`package.xml`](package.xml) | Build/export/dependency metadata |
| [`src/q`](src/q), [`command`](command) | Historical interface logs/design notes and original read-only commands |

## RobotModel

```cpp
RobotModel model;
model.load("/absolute/path/to/expanded_fr3_arm.urdf");
const auto M = model.mass(state.q);             // 49 values, column-major
const auto g = model.gravity(state.q);         // N*m
const auto h = model.nonlinear_effects(state);  // c(q,v) + g(q)
const auto a = model.acceleration(state, total_torque);
const auto next = model.step(state, total_torque, dt);
```

The implementation solves a linear system, rather than explicitly inverting M:

$$
M(q)\ddot q=\tau_{\mathrm{total}}-c(q,\dot q)-g(q).
$$

`load()` accepts an expanded URDF file, not Xacro source. It requires a fixed
base and exactly seven scalar joints in `fr3_joint1` through `fr3_joint7` order.
Movable finger joints must be locked/reduced before export. Preserve tool and
payload inertias in the arm model rather than deleting their mass arbitrarily.
No automatic gripper reduction or payload estimation is implemented.

Checks include finite position/velocity/effort limits, positive mass,
physically plausible inertias, and positive-definite mass matrix at a test
configuration. Failure leaves the object unloaded. These checks establish
numerical consistency, not that the URDF matches your physical FR3. Compare
quantities against the Franka model and verify tool/payload/base frames before
hardware use.

Gravity defaults to `[0,0,-9.81]` in the URDF base frame. Supply the correct
vector to `load()` for a differently oriented base. The backend includes URDF
rigid-body dynamics; it does not identify friction, contact or disturbances.
`step()` holds total torque constant during one RK4 step.

Workspace is allocated in numerical model calls. This is an offline/prototype
API, not suitable for a 1 kHz hardware update loop as implemented. Do not reload
a model while the optimizer uses it.

## Optimizer

Build with `FR3_NMPC_WITH_CASADI=ON` to include the backend. Default is OFF so
model-only work can proceed while CasADi is absent. With it OFF, configuration
throws a clear error and solve returns no result.

```cpp
auto model = std::make_shared<RobotModel>();
model->load(urdf_path);
OptimizerConfig config;
// Set horizon, dt and cost weights as needed.
// REQUIRED: provide explicit positive, verified limits; defaults are zero.
config.torque_limit = verified_total_torque_limits;
config.torque_rate_limit = verified_total_torque_rate_limits;
config.acceleration_limit = verified_acceleration_limits;
Optimizer solver;
solver.configure(model, config);
// refs contains N+1 states sampled at t, t+dt, ... t+N*dt.
auto candidate = solver.solve(measured_state, refs, previous_total_torque);
if (!candidate) {
  // Offline caller handles solver.last_error(); no command is produced.
}
```

The optimizer provides:

- State `[q;dq]`, total-torque inputs and RK4 multiple shooting.
- Position/velocity bounds from the URDF, explicit total-torque/rate/acceleration bounds.
- Position and velocity tracking costs, normalized torque/delta-torque penalties,
  and terminal position cost. Weights are development defaults, not FR3 tuning.
- Shifted primal warm start; `reset()` clears it for a new reference/time origin.
- IPOPT success check and post-solve evaluation of all NLP constraint residuals.
- `nullopt` and `last_error()` on invalid inputs, infeasibility or solver failure.

Acceleration bounds are checked at shooting-node states. Position/velocity
bounds are at nodes; continuous/intermediate-step safety is not guaranteed.
Torque-rate bounds use dt and the explicitly supplied previous total torque.
That is not a substitute for the future hardware-rate limiter or Franka's
command-specific requirements.

A numerical CasADi Callback wraps Pinocchio and uses finite-difference first
derivatives with IPOPT's limited-memory Hessian. Optional `pinocchio.casadi`
bindings are not required. This is intentionally a correctness prototype;
finite differences and repeated dynamics evaluations can be very slow. It does
not claim 1 ms solves. Upgrade to validated analytic/symbolic derivatives after
correctness and timing measurements.

`max_cpu_time_s` is IPOPT's CPU budget, not a hard wall-time watchdog. Results
record solve wall time. Use one optimizer per worker; calls are not thread-safe.

**Results are offline candidates.** `valid_until_ns`, `state_time_ns` and
`reference_id` remain zero. The future worker must assign validated timing and
reference metadata; the controller must reject absent/expired validity.
A result does not automatically authorize an actuator command.

The constant-reference overload exists for compatibility only. It assumes a
previous total torque equal to model gravity, so it is not an online command
history interface. Use the explicit horizon/previous-torque overload in tracking.

## Dependencies and build

Required: ROS 2 Jazzy development packages, Eigen3 and Pinocchio. Optional:
compatible C++ CasADi with its IPOPT plugin. No Crocoddyl or qpOASES is required.

```bash
cd ~/panda_franka_robot
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fr3_nmpc_controller --symlink-install \
  --cmake-args \
  -Dpinocchio_DIR=/opt/ros/jazzy/lib/x86_64-linux-gnu/cmake/pinocchio \
  -DFR3_NMPC_WITH_CASADI=OFF
source install/setup.bash
```

Once a compatible C++ CasADi installation is available, enable the backend with
`-DFR3_NMPC_WITH_CASADI=ON -Dcasadi_DIR=/absolute/path/to/casadi/cmake`.
The CMake package must export `casadi::casadi`; IPOPT must be included in that
installation. Optional CasADi/IPOPT provisioning is external and manual, not
installed by package.xml/colcon.

Check C++ ABI compatibility. Some CasADi Python wheels export
`_GLIBCXX_USE_CXX11_ABI=0`; do not force that into a ROS/Pinocchio stack built
with ABI=1. Use a compatible source/distribution build. A successful Python
import or `find_package` does not by itself establish ABI or solver compatibility.

With an exported arm URDF, check numerical model consistency offline:

```bash
ros2 run fr3_nmpc_controller check_model /absolute/path/to/expanded_fr3_arm.urdf
colcon test --packages-select fr3_nmpc_controller
colcon test-result --verbose
```

These commands do not require a robot connection and do not switch controllers.
A passing gravity-hold check is internal consistency, not hardware validation.

## Hardware work still pending

The actual FR3 reports effort commands and position/velocity states, with
`fr3_arm_controller` currently claiming all seven effort interfaces. Keep it
active during development. The plugin in this package still cannot activate.

Pending: reference subscription/time synchronization, solver thread,
nonblocking state/result exchange, measured-latency compensation, gravity and
payload convention checks, hardware torque/rate limits, validated failure and
completion behavior, simulation and hardware integration tests. Do not remove
the activation guard just because a model or solve succeeds.

The model uses TOTAL torque. Franka effort commands may exclude gravity added
by its driver. Implement and verify that mapping against the installed driver;
never blindly forward `total_torque` to hardware.

MoveIt supplies a timed joint reference for the NMPC experiment. A geometric
`nav_msgs/Path` alone is insufficient. Approach and Cartesian segments need
explicit sequencing. Baseline and NMPC trials must have a single arm-command
owner and comparable initial conditions, payload and reference timing.

## Validation evidence

Completed checks:

- Pinocchio 4.1.0 numerical backend compiled and linked outside ROS.
- Synthetic seven-joint URDF loading, nonzero-configuration gravity equilibrium,
  RK4 equilibrium, nonfinite input rejection and failed-reload invalidation passed.
- CasADi 3.7.2 optimizer compiled with the backend both enabled and disabled.
- Two NLP solves using a separate nonlinear test plant passed, including shifted
  warm start, first-step torque-rate checks and NaN-reference rejection.
- XML and README local-link checks passed.

The Pinocchio and CasADi tests ran separately because the available binaries
have different C++ ABIs. A combined Pinocchio/CasADi solve, ROS gtest execution,
full Jazzy plugin build and real FR3 model/hardware remain untested. The
synthetic test fixture is never an FR3 validation asset.

## References

- [Pinocchio model loading](https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/overview-urdf.cpp)
- [CasADi callbacks and differentiation](https://web.casadi.org/docs/)
- [Franka ROS 2 hardware interfaces](https://frankarobotics.github.io/docs/doc/franka_ros2_jazzy/franka_hardware/doc/index.html)
