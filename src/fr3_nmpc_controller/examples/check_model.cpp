// Offline numerical consistency check. No ROS node or hardware access.
#include "fr3_nmpc_controller/robot_model.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
int main(int argc,char ** argv)
{
  if (argc!=2) {
    std::cerr<<"Usage: check_model /absolute/path/to/expanded_seven_joint_fr3.urdf\n";
    return 2;
  }
  try {
    fr3_nmpc_controller::RobotModel model;
    model.load(argv[1]);
    const auto lim=model.limits();
    fr3_nmpc_controller::RobotState x;
    for (std::size_t i=0;i<7;++i) { x.q[i]=lim.lower[i]+.55*(lim.upper[i]-lim.lower[i]); }
    const auto g=model.gravity(x.q), a=model.acceleration(x,g);
    const auto y=model.step(x,g,.01);
    auto moving=x;
    fr3_nmpc_controller::JointVector requested{};
    for (std::size_t i=0;i<7;++i) { moving.dq[i]=0.02*(i+1); requested[i]=-0.03*(i+1); }
    const auto recovered=model.acceleration(moving,model.inverse_dynamics(moving,requested));
    double round_trip_error=0;
    for (std::size_t i=0;i<7;++i) {
      round_trip_error=std::max(round_trip_error,std::abs(recovered[i]-requested[i]));
    }
    double max_acceleration=0, max_drift=0;
    for (std::size_t i=0;i<7;++i) {
      max_acceleration=std::max(max_acceleration,std::abs(a[i]));
      max_drift=std::max(max_drift,std::abs(y.q[i]-x.q[i]));
    }
    std::cout<<"max |qdd(q,0,g(q))| = "<<max_acceleration<<" rad/s^2\n"
             <<"gravity-hold integration drift = "<<max_drift<<" rad\n"
             <<"inverse/forward acceleration error = "<<round_trip_error<<" rad/s^2\n"
             <<"Numerical consistency only; not an FR3 hardware validation.\n";
    return max_acceleration<1e-8 && max_drift<1e-8 && round_trip_error<1e-8 ? 0 : 1;
  } catch (const std::exception & e) {
    std::cerr<<"Model check failed: "<<e.what()<<'\n';
    return 1;
  }
}

