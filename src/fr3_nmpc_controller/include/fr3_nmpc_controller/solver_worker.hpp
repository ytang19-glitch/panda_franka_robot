#pragma once
#include "fr3_nmpc_controller/optimizer.hpp"
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace fr3_nmpc_controller
{
struct SolverRequest
{
  RobotState state;
  std::vector<RobotState> reference_horizon;  // N+1 samples, aligned to state time.
  JointVector previous_total_torque{};
  std::int64_t state_time_ns{0};              // steady_clock, not ROS time.
  std::uint64_t reference_id{0};             // Positive, increasing for new paths.
};

// Background prototype, not a real-time mailbox. submit/latest use mutexes and
// dynamic copies; call from non-RT code only. No actuator interface is present.
class SolverWorker
{
public:
  using SolveFunction = std::function<std::optional<SolverResult>(const SolverRequest &, bool reset)>;
  SolverWorker() = default;
  ~SolverWorker();
  SolverWorker(const SolverWorker &) = delete;
  SolverWorker & operator=(const SolverWorker &) = delete;
  // Lifecycle methods must be serialized by one non-RT owner. stop() waits for
  // a running solve to return; it cannot preempt IPOPT and has no hard deadline.
  void start(std::shared_ptr<Optimizer> optimizer, std::size_t horizon,
    std::int64_t max_state_age_ns);
  // Dependency injection for isolated scheduling tests or alternative solvers.
  void start_with_solver(SolveFunction solve, std::size_t horizon,
    std::int64_t max_state_age_ns);
  void stop();
  void cancel_reference(); // Invalidates pending/in-flight results, not solver execution.
  bool ready() const noexcept;
  bool submit(SolverRequest request); // One pending slot: newest accepted snapshot wins.
  std::optional<SolverResult> latest() const; // Returns only current, unexpired result.
  std::string last_error() const;
  static std::int64_t now_ns() noexcept;
private:
  void run();
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::thread thread_;
  std::atomic<bool> running_{false};
  bool stopping_{true};
  std::size_t horizon_{0};
  std::int64_t max_age_ns_{0}, last_state_ns_{0};
  std::uint64_t reference_id_{0}, epoch_{0};
  SolveFunction solve_;
  std::optional<SolverRequest> pending_;
  std::optional<SolverResult> result_;
  std::string error_;
};
}  // namespace fr3_nmpc_controller
