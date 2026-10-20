# FR3 NMPC Optimizer Test Timeout: Evidence and Debugging Procedure

**Recorded:** October 20, 2026  
**Status:** Under investigation; no root cause or performance fix confirmed.

This note documents a timeout in the C++ CasADi/Pinocchio optimizer test. It separates observed behavior from hypotheses and describes the next diagnostic steps. These are offline tests, not robot motion commands.

## 1. Environment and affected code

- Ubuntu 24.04 / ROS 2 Jazzy.
- Workspace: `~/panda_franka_robot`.
- Python CasADi version reported by the user: 3.8.0.
- CasADi CMake directory: `/home/yujietang/venvs/nmpc/lib/python3.12/site-packages/casadi/cmake`.
- Build option: `FR3_NMPC_WITH_CASADI=ON`.
- [Test source](../../src/fr3_nmpc_controller/test/test_optimizer.cpp): `Optimizer.FeasibleGravityHoldAndBadReference`.
- [Optimizer](../../src/fr3_nmpc_controller/src/optimizer.cpp): CasADi Opti / IPOPT with a numerical dynamics callback.
- [Robot model](../../src/fr3_nmpc_controller/src/robot_model.cpp): Pinocchio dynamics and RK4 integration.

The test uses a **synthetic seven-joint fixture**, not the physical FR3 model. The reviewed test configures a horizon of two steps, dt = 0.02 s and 30-second CPU/post-solve wall-time acceptance budgets. These are test settings, not validated robot-control settings.

## 2. Initial symptom: CTest killed the test

Three test executables passed:

- `test_unimplemented_backends`.
- `test_solver_worker`.
- `test_robot_model`.

The optimizer test did not finish within CTest's 60-second limit:

```text
Start 4: test_optimizer
[ RUN      ] Optimizer.FeasibleGravityHoldAndBadReference
test_optimizer ... Timeout 60.13 sec
```

The later message:

```text
test_optimizer.gtest.missing_result
The test did not generate a result file.
```

is a consequence of the interrupted process failing to write its result XML. It is not evidence of a separate optimizer defect.

A successful build and three passing tests do not mean the optimizer test passed.

## 3. Isolate basic IPOPT availability

A minimal Python problem was tested with BLAS/OpenMP thread counts restricted to one:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
timeout 20s /home/yujietang/venvs/nmpc/bin/python -u - <<'PY'
import time
import casadi as ca

x = ca.MX.sym("x")
problem = {"x": x, "f": (x - 1)**2}

print("Creating IPOPT solver...", flush=True)
start = time.monotonic()
solver = ca.nlpsol("check", "ipopt", problem, {
    "ipopt.print_level": 0,
    "print_time": False,
})

print("Solving...", flush=True)
result = solver(x0=0)
print("x =", float(result["x"]))
print("Elapsed:", time.monotonic() - start, "s")
PY

echo "Exit code: $?"
```

Observed output:

```text
Creating IPOPT solver...
Solving...
x = 1.0
Elapsed: 0.09839310869574547 s
Exit code: 0
```

**Conclusion:** basic IPOPT initialization and scalar optimization worked in that Python environment under those settings.

**Limit:** this does not validate the C++ runtime libraries, numerical callback derivatives, rigid-body dynamics integration, warm start or NMPC performance. Because this check was run with thread restrictions, it also does not establish behavior without them.

## 4. C++ diagnostic output narrowed the location

Local progress markers were added to the test. A subsequent run used:

```bash
cd ~/panda_franka_robot
source install/setup.bash

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
timeout --kill-after=5s 30s \
  ./build/fr3_nmpc_controller/test_optimizer

echo "Exit code: $?"
```

Observed output:

```text
[ RUN      ] Optimizer.FeasibleGravityHoldAndBadReference
[DIAG] Loading model
[DIAG] Configuring optimizer
[DIAG] Configuration finished
[DIAG] Solve 0 starting
[DIAG] Solve returned:
[DIAG] Solve 1 starting
Exit code: 124
```

| Observation | Supported interpretation |
|---|---|
| Configuration finished | Model loading and optimizer configuration returned |
| Solve 0 returned with an empty error string | The first solve returned without a reported optimizer error |
| Execution entered Solve 1 | The test proceeded through the first iteration's assertions and state update |
| Exit code 124 | The external GNU timeout command reached its time limit |
| No second completion marker | The second solve had not returned before termination |

**Critical timing distinction:** the 30-second external timeout applies to the **entire executable**. It does not give the second solve a fresh 30-second budget. If the first solve consumed most of the budget, the second may have had little time left.

The evidence narrows the investigation to work performed during repeated solving; it does not prove the second solve is stuck or that warm start is defective.

## 5. Candidate causes — not confirmed diagnoses

| Hypothesis | Why investigate it? | Evidence needed |
|---|---|---|
| Both solves are expensive | Numerical dynamics and finite-difference derivatives can require repeated model evaluations | Per-call durations, callback counts and profiling |
| Warm-start initialization is ineffective for the updated state | The second iteration shifts the prior prediction while the measured/simulated state changes | Controlled warm-start versus reset comparison |
| IPOPT iterations stall or progress slowly | A difficult numerical problem may take substantial work | IPOPT iteration log and termination status |
| C++ initialization or runtime behavior differs from Python | The scalar Python check does not exercise the C++ integration | Separate plugin/first-call timing and library inspection if indicated |

Thread limits did not make this C++ run finish within its external budget. That result alone does not identify the remaining bottleneck.

## 6. Next step: measure each solve

Add these includes to `test_optimizer.cpp` if absent:

```cpp
#include <chrono>
#include <iostream>
```

Inside the existing `for (int k=0; k<2; ++k)` loop, **replace the existing solve statement** with this block. Do not retain a second declaration of `result`:

```cpp
std::cerr << "[DIAG] Solve " << k << " starting" << std::endl;

const auto start = std::chrono::steady_clock::now();
auto result = optimizer.solve(x, refs, previous);
const double seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start).count();

std::cerr << "[DIAG] Solve " << k
          << " took " << seconds << " s"
          << ", success=" << result.has_value()
          << ", error=" << optimizer.last_error()
          << std::endl;
```

Keep the existing assertions, torque checks and state updates after this block. The timer measures the entire public solve call, which can include work beyond the solver's internal reported time.

Rebuild the test target:

```bash
cd ~/panda_franka_robot
colcon build --packages-select fr3_nmpc_controller \
  --cmake-target test_optimizer
```

Only after a successful build, rerun the bounded command from Section 4. If the second solve is interrupted, the first solve's printed duration still helps interpret the cumulative timeout.

Do not simply increase CTest's timeout and call the issue fixed. First establish where the time is spent. A later longer observation window, if justified, must be reported explicitly.

## 7. Warm-start comparison after timing is available

For a separate diagnostic run, insert the following immediately before the second iteration's solve:

```cpp
if (k == 1) {
  optimizer.reset();
}
```

Compare with the original warm-start-enabled run using the same model, reference, previous torque, thread settings and executable build configuration.

Record:

| Run | Solve 0 duration | Solve 1 duration or interruption | Convergence and constraint results |
|---|---|---|---|
| Warm start enabled | To measure | To measure | To measure |
| Reset before Solve 1 | To measure | To measure | To measure |

This is an experiment, not a confirmed fix. Resetting changes solver initialization; it does not correct the dynamics or constraints. Restore the intended repeated-solve test afterward so the regression suite continues to exercise warm start.

If timing remains unclear, add configuration/solve-stage measurements and IPOPT iteration logging in a diagnostic build. Distinguish graph/plugin setup from numerical iterations before optimizing the implementation.

## 8. Why an internal time setting may not stop the executable

There are different timing mechanisms:

| Mechanism | Scope |
|---|---|
| External `timeout` | Whole process, including configuration and all solves |
| CTest timeout | Whole test execution |
| IPOPT `max_cpu_time` | CPU-time limit checked within IPOPT; not a hard process-level watchdog |
| Project `max_wall_time_s` | Acceptance check after the solve returns; cannot interrupt that call |

The reviewed test performs two nominal solves and a later deliberately tiny wall-budget rejection solve. Multiple solves share the outer test budget. The latest diagnostic run had not yet reached the final rejection case.

Finite differences are a legitimate callback derivative mechanism, but their repeated evaluations can be expensive. They are a performance hypothesis here, not a measured root cause. The numerical model also allocates workspace and is not a proven 1 kHz control-loop implementation.

## 9. The diagnostic-script message is harmless

```text
Diagnostics already present; no changes made.
```

means the helper script found an existing `[DIAG]` marker and deliberately avoided inserting duplicate instrumentation.

It does not indicate compilation failure or solver failure. To extend existing diagnostics, edit the marked code rather than repeatedly running the original insertion script.

## 10. Completion criteria and current boundary

A resolved investigation should establish:

1. Which phase consumes the time.
2. Whether the problem is initialization, model evaluation, derivatives, iteration behavior or another measured cause.
3. Whether warm start helps or hurts under controlled comparison.
4. Whether the test completes and its original assertions pass.
5. Actual timing distributions for the intended moving-reference workload.

Passing a synthetic gravity-hold test does not demonstrate real-time FR3 trajectory tracking. Keep physical torque output disabled while investigating; these tests require no controller switching.

**Current conclusion:** basic Python IPOPT works; C++ model/configuration and one solve return; the executable reaches a second solve before a cumulative external timeout. Root cause and corrective implementation remain unconfirmed.

## References

- [Optimizer test source](../../src/fr3_nmpc_controller/test/test_optimizer.cpp)
- [Optimizer implementation](../../src/fr3_nmpc_controller/src/optimizer.cpp)
- [CasADi callback and finite-difference documentation](https://web.casadi.org/docs/)
- [IPOPT termination options](https://coin-or.github.io/Ipopt/OPTIONS.html)
