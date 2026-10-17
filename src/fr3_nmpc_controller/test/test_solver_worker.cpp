// Standalone concurrency regression checks; no ROS, robot or CasADi required.
#include "fr3_nmpc_controller/solver_worker.hpp"
#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
using namespace std::chrono_literals;
namespace f=fr3_nmpc_controller;
void require(bool ok,const char * what) { if (!ok) { throw std::runtime_error(what); } }
template<class Predicate> void until(Predicate p)
{
  const auto end=std::chrono::steady_clock::now()+2s;
  while (!p()) {
    if (std::chrono::steady_clock::now()>=end) { throw std::runtime_error("Test timed out"); }
    std::this_thread::sleep_for(1ms);
  }
}
f::SolverRequest request(std::uint64_t id)
{
  f::SolverRequest r; r.reference_horizon.resize(3);
  r.reference_id=id; r.state_time_ns=f::SolverWorker::now_ns(); return r;
}
f::SolverResult candidate(double value=1)
{
  f::SolverResult r; r.converged=true; r.total_torque.fill(value); return r;
}
int main()
{
  try {
    f::SolverWorker worker;
    require(!worker.ready() && !worker.latest(),"Unstarted worker produced result");
    // A blocked old reference must never become the current result. Only the
    // newest pending state of the new reference should be consumed.
    std::promise<void> entered,release;
    auto gate=release.get_future().share();
    worker.start_with_solver([&](const f::SolverRequest & r,bool reset) -> std::optional<f::SolverResult> {
      require(reset,"New reference must reset warm start");
      if (r.reference_id==1) { entered.set_value(); gate.wait(); }
      return candidate(r.state.q[0]);
    },2,2'000'000'000);
    auto one=request(1);one.state.q[0]=1;require(worker.submit(one),"Submit failed");
    auto started=entered.get_future();
    if(started.wait_for(2s)!=std::future_status::ready) { release.set_value(); throw std::runtime_error("Worker did not enter solve"); }
    auto two=request(2);two.state.q[0]=2;
    const bool accepted_two=worker.submit(two);
    auto three=request(2);three.state_time_ns=two.state_time_ns+1;three.state.q[0]=3;
    // Wait for now_ns to exceed the intentionally incremented source timestamp.
    until([&]{return f::SolverWorker::now_ns()>=three.state_time_ns;});
    const bool accepted_three=worker.submit(three);
    release.set_value();
    require(accepted_two && accepted_three,"Replacement submit failed");
    until([&]{auto r=worker.latest();return r && r->reference_id==2;});
    auto r=worker.latest();require(r && r->total_torque[0]==3,"Newest pending request was not used");
    require(r->state_time_ns==three.state_time_ns && r->valid_until_ns==three.state_time_ns+2'000'000'000,"Wrong result timestamps");
    require(!worker.submit(one),"Old reference ID accepted");
    auto bad=request(3);bad.state.q[0]=std::numeric_limits<double>::quiet_NaN();
    require(!worker.submit(bad),"NaN state accepted");
    worker.cancel_reference();require(!worker.latest(),"Canceled result visible");
    worker.stop();require(!worker.ready() && !worker.latest(),"Stop failed");

    // Result freshness must count time spent solving, starting at source state.
    std::promise<void> expiry_entered,expiry_release;
    auto expiry_gate=expiry_release.get_future().share();
    worker.start_with_solver([&](const f::SolverRequest &,bool)->std::optional<f::SolverResult>{
      expiry_entered.set_value();expiry_gate.wait();return candidate();
    },2,100'000'000);
    const auto expiring=request(1);require(worker.submit(expiring),"Fresh state rejected");
    auto expiry_started=expiry_entered.get_future();
    if(expiry_started.wait_for(2s)!=std::future_status::ready){expiry_release.set_value();throw std::runtime_error("No expiry solve");}
    until([&]{return f::SolverWorker::now_ns()>=expiring.state_time_ns+100'000'000;});
    expiry_release.set_value();until([&]{return !worker.last_error().empty();});
    require(!worker.latest(),"Expired solve was exposed");worker.stop();

    worker.start_with_solver([](const f::SolverRequest &,bool)->std::optional<f::SolverResult>{
      throw std::runtime_error("Test solver failure");
    },2,2'000'000'000);
    require(worker.submit(request(1)),"Failure test submit rejected");
    until([&]{return worker.last_error()=="Test solver failure";});
    require(!worker.latest(),"Failed solve exposed torque");worker.stop();
    std::cout<<"Worker tests passed: lifecycle, replacement, metadata, NaN, expiry and exceptions\n";
    return 0;
  } catch(const std::exception & e) { std::cerr<<e.what()<<'\n';return 1; }
}
