#include "openpid/openpid.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace {

// 一阶惯性对象：dy/dt = (-y + k * u) / tau
double step_plant(double y, double u, double dt, double gain, double tau) {
  return y + dt * (-y + gain * u) / tau;
}

} // namespace

int main() {
  constexpr double dt = 0.01;
  constexpr double plant_gain = 1.0;
  constexpr double plant_tau = 1.0;
  constexpr double setpoint = 1.0;
  constexpr int steps = 1000;
  constexpr int print_every = 100;

  openpid::Config<double> config;
  config.gains = {1.8, 1.2, 0.15};
  config.derivative = openpid::DerivativeMode::OnMeasurement;
  config.derivative_filter_tau = 0.05;
  config.limits.output_min = -10.0;
  config.limits.output_max = 10.0;

  openpid::Controller<double> pid(config);
  double measurement = 0.0;

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "t      setpoint  measurement  output    error\n";

  for (int step = 0; step < steps; ++step) {
    const double output = pid.update(setpoint, measurement, dt);
    measurement = step_plant(measurement, output, dt, plant_gain, plant_tau);
    if (step % print_every == 0 || step + 1 == steps) {
      const double time = (step + 1) * dt;
      std::cout << std::setw(6) << time << "  " << std::setw(8) << setpoint
                << "  " << std::setw(11) << measurement << "  " << std::setw(8)
                << output << "  " << std::setw(8) << (setpoint - measurement)
                << "\n";
    }
  }

  const double final_error = std::abs(setpoint - measurement);
  if (!(final_error < 0.05)) {
    std::cerr << "closed loop did not converge, |error| = " << final_error
              << "\n";
    return 1;
  }
  return 0;
}
