// Standalone diagnostic executable; finite differences are used ONLY to test.
#include "fr3_nmpc_controller/robot_model.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace fc = fr3_nmpc_controller;

double component(const fc::RobotState & x, std::size_t i)
{
  return i < 7 ? x.q[i] : x.dq[i - 7];
}

int main(int argc, char ** argv)
{
  if (argc != 2) { std::cerr << "Usage: check_dynamics_derivatives MODEL.urdf\n"; return 1; }
  try {
    fc::RobotModel model;
    model.load(argv[1]);
    const auto limits = model.limits();
    double max_scaled_error = 0;
    double max_prediction_error = 0;
    for (int sample = 0; sample < 6; ++sample) {
      fc::RobotState x{};
      for (std::size_t j = 0; j < fc::kJoints; ++j) {
        const double phase = static_cast<double>((sample + 1) * (j + 1));
        x.q[j] = 0.5 * (limits.lower[j] + limits.upper[j]) +
          0.1 * (limits.upper[j] - limits.lower[j]) * std::sin(phase);
        x.dq[j] = 0.05 * limits.velocity[j] * std::cos(phase);
      }
      fc::JointVector u = model.gravity(x.q);
      for (std::size_t j = 0; j < fc::kJoints; ++j) {
        u[j] += 0.01 * limits.effort[j] * std::sin(static_cast<double>(sample + j + 1));
      }
      for (double dt : {0.005, 0.02, 0.05}) {
        const auto d = model.linearize_step(x, u, dt);
        const auto nominal = model.step(x, u, dt);
        const auto nominal_acc = model.acceleration(x, u);
        for (std::size_t row = 0; row < 14; ++row) {
          max_prediction_error = std::max(max_prediction_error,
            std::abs(component(d.next_state, row) - component(nominal, row)));
        }
        for (std::size_t row = 0; row < 7; ++row) {
          max_prediction_error = std::max(max_prediction_error,
            std::abs(d.acceleration[row] - nominal_acc[row]));
        }
        for (std::size_t col = 0; col < 21; ++col) {
          auto xp = x, xm = x;
          auto up = u, um = u;
          const double base = col < 14 ? component(x, col) : u[col - 14];
          const double epsilon = 1e-6 * std::max(1.0, std::abs(base));
          if (col < 7) { xp.q[col] += epsilon; xm.q[col] -= epsilon; }
          else if (col < 14) { xp.dq[col - 7] += epsilon; xm.dq[col - 7] -= epsilon; }
          else { up[col - 14] += epsilon; um[col - 14] -= epsilon; }
          const auto yp = model.step(xp, up, dt), ym = model.step(xm, um, dt);
          const auto ap = model.acceleration(xp, up), am = model.acceleration(xm, um);
          auto compare = [&](double analytic, double numerical) {
            const double error = std::abs(analytic - numerical) /
              std::max({1.0, std::abs(analytic), std::abs(numerical)});
            if (!std::isfinite(error)) { throw std::runtime_error("Non-finite derivative error"); }
            max_scaled_error = std::max(max_scaled_error, error);
          };
          for (std::size_t row = 0; row < 14; ++row) {
            const double analytic = col < 14 ? d.step_dx[col * 14 + row] :
              d.step_du[(col - 14) * 14 + row];
            compare(analytic, (component(yp, row) - component(ym, row)) / (2 * epsilon));
          }
          for (std::size_t row = 0; row < 7; ++row) {
            const double analytic = col < 14 ? d.acceleration_dx[col * 7 + row] :
              d.acceleration_du[(col - 14) * 7 + row];
            compare(analytic, (ap[row] - am[row]) / (2 * epsilon));
          }
        }
      }
    }
    std::cout << "Maximum prediction difference: " << max_prediction_error
              << "\nMaximum scaled derivative error: " << max_scaled_error << '\n';
    if (!std::isfinite(max_prediction_error) || max_prediction_error > 1e-10 ||
        max_scaled_error > 1e-5) {
      throw std::runtime_error("Dynamics derivative comparison failed");
    }
    std::cout << "PASS: acceleration and RK4 Jacobians (6 states, 3 dt values, all 21 columns)\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "FAIL: " << e.what() << '\n'; return 1;
  }
}
