#include "fr3_nmpc_controller/solver_worker.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fr3_nmpc_controller
{
namespace
{
bool finite(const JointVector & a)
{
  for (double v:a) { if (!std::isfinite(v)) { return false; } }
  return true;
}
bool finite(const RobotState & x) { return finite(x.q) && finite(x.dq); }
}
SolverWorker::~SolverWorker() { stop(); }
std::int64_t SolverWorker::now_ns() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool SolverWorker::ready() const noexcept { return running_.load(); }
void SolverWorker::start(std::shared_ptr<Optimizer> optimizer, std::size_t horizon,
  std::int64_t max_age)
{
  if (!optimizer || !optimizer->ready()) {
    throw std::invalid_argument("Configure the optimizer before starting worker");
  }
  // The caller must not use/reconfigure this optimizer concurrently.
  start_with_solver([optimizer](const SolverRequest & request, bool reset) {
    if (reset) { optimizer->reset(); }
    auto result=optimizer->solve(request.state,request.reference_horizon,request.previous_total_torque);
    if (!result) { throw std::runtime_error(optimizer->last_error()); }
    return result;
  },horizon,max_age);
}
void SolverWorker::start_with_solver(SolveFunction solve, std::size_t horizon, std::int64_t max_age)
{
  if (ready() || thread_.joinable()) { throw std::logic_error("Stop existing worker before starting"); }
  if (!solve || horizon<2 || horizon>100 || max_age<=0) {
    throw std::invalid_argument("Require solver, horizon 2..100 and explicit positive max state age");
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    solve_=std::move(solve); horizon_=horizon; max_age_ns_=max_age;
    pending_.reset(); result_.reset(); error_.clear();
    reference_id_=0; last_state_ns_=0; ++epoch_; stopping_=false;
  }
  running_.store(true);
  try { thread_=std::thread(&SolverWorker::run,this); }
  catch (...) { running_.store(false); std::lock_guard<std::mutex> lock(mutex_); stopping_=true; solve_={}; throw; }
}
void SolverWorker::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_=true; running_.store(false); ++epoch_;
    pending_.reset(); result_.reset();
  }
  wake_.notify_all();
  if (thread_.joinable()) { thread_.join(); }
  std::lock_guard<std::mutex> lock(mutex_);
  solve_={};
}
void SolverWorker::cancel_reference()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++epoch_; pending_.reset(); result_.reset(); reference_id_=0; last_state_ns_=0;
}
bool SolverWorker::submit(SolverRequest request)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now=now_ns();
  if (stopping_ || !running_.load()) { error_="Worker is stopped"; return false; }
  if (!finite(request.state) || !finite(request.previous_total_torque) ||
      request.reference_horizon.size()!=horizon_+1 || request.reference_id==0 ||
      request.state_time_ns<=0 || request.state_time_ns>now ||
      now-request.state_time_ns>=max_age_ns_ ||
      request.state_time_ns>std::numeric_limits<std::int64_t>::max()-max_age_ns_) {
    error_="Invalid state, horizon, ID or monotonic source timestamp"; return false;
  }
  for (const auto & ref:request.reference_horizon) {
    if (!finite(ref)) { error_="Non-finite reference"; return false; }
  }
  if (request.reference_id<reference_id_ ||
      (request.reference_id==reference_id_ && request.state_time_ns<=last_state_ns_)) {
    error_="Out-of-order reference ID or state time"; return false;
  }
  if (request.reference_id!=reference_id_) {
    ++epoch_; result_.reset(); reference_id_=request.reference_id;
  }
  last_state_ns_=request.state_time_ns;
  pending_=std::move(request);
  wake_.notify_one();
  return true;
}
std::optional<SolverResult> SolverWorker::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_ || !result_ || result_->reference_id!=reference_id_ ||
      now_ns()>=result_->valid_until_ns) { return std::nullopt; }
  return result_;
}
std::string SolverWorker::last_error() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return error_;
}
void SolverWorker::run()
{
  std::uint64_t solved_epoch=0;
  while (true) {
    SolverRequest request;
    std::uint64_t epoch;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock,[this] { return stopping_ || pending_.has_value(); });
      if (stopping_) { return; }
      request=std::move(*pending_); pending_.reset(); epoch=epoch_;
    }
    const auto expiry=request.state_time_ns+max_age_ns_;
    std::optional<SolverResult> candidate;
    std::string error;
    if (now_ns()>=expiry) { error="State expired before solve"; }
    else {
      try {
        candidate=solve_(request,epoch!=solved_epoch);
        solved_epoch=epoch;
        if (!candidate || !candidate->converged || !finite(candidate->total_torque)) {
          candidate.reset(); error="Solver returned no converged finite torque";
        }
      } catch (const std::exception & e) { error=e.what(); solved_epoch=0; }
      catch (...) { error="Unknown solver exception"; solved_epoch=0; }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // Discard results of stopped/canceled/replaced references even if successful.
    if (stopping_) { return; }
    if (epoch!=epoch_ || request.reference_id!=reference_id_) { continue; }
    if (!candidate || now_ns()>=expiry) {
      result_.reset(); error_=error.empty()?"Solution expired during solve":error;
      continue;
    }
    candidate->state_time_ns=request.state_time_ns;
    candidate->reference_id=request.reference_id;
    candidate->valid_until_ns=expiry; // Age measured from source state, NOT solve completion.
    result_=std::move(candidate); error_.clear();
  }
}
}  // namespace fr3_nmpc_controller
