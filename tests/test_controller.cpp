#include "openpid/openpid.hpp"

#include <cmath>
#include <iostream>

namespace {

int g_failed = 0;

template <typename T>
void expect_near(T actual, T expected, T tolerance, const char *file,
                 int line) {
  if (!(std::abs(actual - expected) <= tolerance)) {
    std::cerr << file << ":" << line << " failed: " << actual
              << " != " << expected << "\n";
    ++g_failed;
  }
}

#define EXPECT_NEAR(actual, expected, tolerance)                               \
  expect_near((actual), (expected), (tolerance), __FILE__, __LINE__)

void test_proportional_only() {
  openpid::Config<double> config;
  config.gains = {2.0, 0.0, 0.0};
  openpid::Controller<double> pid(config);

  const double output = pid.update(5.0, 2.0, 0.1);
  EXPECT_NEAR(output, 6.0, 1e-12);
  EXPECT_NEAR(pid.proportional(), 6.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 0.0, 1e-12);
  EXPECT_NEAR(pid.derivative(), 0.0, 1e-12);
  EXPECT_NEAR(pid.error(), 3.0, 1e-12);
}

void test_integral_accumulates_constant_error() {
  openpid::Config<double> config;
  config.gains = {0.0, 4.0, 0.0};
  openpid::Controller<double> pid(config);

  EXPECT_NEAR(pid.update(2.0, 0.0, 0.25), 2.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 2.0, 1e-12);
  EXPECT_NEAR(pid.update(2.0, 0.0, 0.25), 4.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 4.0, 1e-12);
}

void test_derivative_on_measurement_has_no_setpoint_kick() {
  openpid::Config<double> config;
  config.gains = {0.0, 0.0, 3.0};
  config.derivative = openpid::DerivativeMode::OnMeasurement;
  openpid::Controller<double> pid(config);

  EXPECT_NEAR(pid.update(0.0, 1.0, 0.25), 0.0, 1e-12);
  EXPECT_NEAR(pid.update(8.0, 1.0, 0.25), 0.0, 1e-12);
  EXPECT_NEAR(pid.derivative(), 0.0, 1e-12);
}

void test_derivative_on_error_reacts_to_setpoint_step() {
  openpid::Config<double> config;
  config.gains = {0.0, 0.0, 3.0};
  config.derivative = openpid::DerivativeMode::OnError;
  openpid::Controller<double> pid(config);

  EXPECT_NEAR(pid.update(0.0, 1.0, 0.25), 0.0, 1e-12);
  EXPECT_NEAR(pid.update(8.0, 1.0, 0.25), 96.0, 1e-12);
  EXPECT_NEAR(pid.derivative(), 96.0, 1e-12);
}

void test_derivative_on_measurement_formula_and_filter() {
  openpid::Config<double> config;
  config.gains = {0.0, 0.0, 1.0};
  config.derivative = openpid::DerivativeMode::OnMeasurement;
  openpid::Controller<double> raw(config);
  EXPECT_NEAR(raw.update(0.0, 0.0, 0.5), 0.0, 1e-12);
  EXPECT_NEAR(raw.update(0.0, 2.0, 0.5), -4.0, 1e-12);

  config.gains.kd = 2.0;
  config.derivative_filter_tau = 0.5;
  openpid::Controller<double> filtered(config);
  EXPECT_NEAR(filtered.update(0.0, 0.0, 0.5), 0.0, 1e-12);
  EXPECT_NEAR(filtered.update(0.0, 1.0, 0.5), -2.0, 1e-12);
  EXPECT_NEAR(filtered.derivative(), -2.0, 1e-12);
}

void test_setpoint_weight() {
  openpid::Config<double> config;
  config.gains = {2.0, 0.0, 0.0};
  config.setpoint_weight = 0.0;
  openpid::Controller<double> pid(config);

  EXPECT_NEAR(pid.update(4.0, 1.0, 0.1), -2.0, 1e-12);
  EXPECT_NEAR(pid.proportional(), -2.0, 1e-12);
  EXPECT_NEAR(pid.error(), 3.0, 1e-12);
}

void test_output_clamp() {
  openpid::Config<double> config;
  config.gains = {10.0, 0.0, 0.0};
  config.limits.output_min = -1.0;
  config.limits.output_max = 1.0;
  openpid::Controller<double> pid(config);

  EXPECT_NEAR(pid.update(5.0, 0.0, 0.1), 1.0, 1e-12);
  EXPECT_NEAR(pid.proportional(), 50.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 0.0, 1e-12);
}

void test_integral_limit() {
  openpid::Config<double> config;
  config.gains = {0.0, 1.0, 0.0};
  config.limits.integral_min = -3.0;
  config.limits.integral_max = 3.0;
  openpid::Controller<double> pid(config);

  EXPECT_NEAR(pid.update(10.0, 0.0, 1.0), 3.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 3.0, 1e-12);
  EXPECT_NEAR(pid.update(10.0, 0.0, 1.0), 3.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 3.0, 1e-12);
}

void test_conditional_integration_does_not_wind_up() {
  openpid::Config<double> config;
  config.gains = {0.0, 5.0, 0.0};
  config.limits.output_min = -2.0;
  config.limits.output_max = 2.0;
  config.anti_windup = openpid::AntiWindup::ConditionalIntegration;
  openpid::Controller<double> pid(config);

  EXPECT_NEAR(pid.update(1.0, 0.0, 1.0), 2.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 2.0, 1e-12);
  EXPECT_NEAR(pid.update(1.0, 0.0, 1.0), 2.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 2.0, 1e-12);

  EXPECT_NEAR(pid.update(0.75, 1.0, 1.0), 0.75, 1e-12);
  EXPECT_NEAR(pid.integral(), 0.75, 1e-12);
}

void test_back_calculation_holds_integral_at_saturation() {
  openpid::Config<double> config;
  config.gains = {0.0, 1.0, 0.0};
  config.limits.output_min = -2.0;
  config.limits.output_max = 2.0;
  config.anti_windup = openpid::AntiWindup::BackCalculation;
  openpid::Controller<double> pid(config);

  EXPECT_NEAR(pid.update(5.0, 0.0, 1.0), 2.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 2.0, 1e-12);
  EXPECT_NEAR(pid.update(5.0, 0.0, 1.0), 2.0, 1e-12);
  EXPECT_NEAR(pid.integral(), 2.0, 1e-12);
}

void test_non_positive_dt_keeps_last_output() {
  openpid::Config<double> config;
  config.gains = {0.0, 2.0, 0.0};
  openpid::Controller<double> pid(config);

  const double output = pid.update(1.0, 0.0, 0.5);
  const double integral = pid.integral();
  EXPECT_NEAR(output, 1.0, 1e-12);

  EXPECT_NEAR(pid.update(100.0, -100.0, 0.0), output, 1e-12);
  EXPECT_NEAR(pid.integral(), integral, 1e-12);
  EXPECT_NEAR(pid.update(100.0, -100.0, -1.0), output, 1e-12);
  EXPECT_NEAR(pid.integral(), integral, 1e-12);
  EXPECT_NEAR(pid.error(), 1.0, 1e-12);
}

void test_reset_presets_integral_and_gains_do_not_clear_state() {
  openpid::Config<double> config;
  config.gains = {0.0, 2.0, 0.0};
  openpid::Controller<double> pid(config);

  pid.reset(4.5);
  EXPECT_NEAR(pid.integral(), 4.5, 1e-12);
  EXPECT_NEAR(pid.last_output(), 4.5, 1e-12);
  EXPECT_NEAR(pid.update(0.0, 0.0, 0.0), 4.5, 1e-12);

  EXPECT_NEAR(pid.update(1.0, 0.0, 0.5), 5.5, 1e-12);
  EXPECT_NEAR(pid.integral(), 5.5, 1e-12);
  pid.set_gains({0.0, 0.0, 0.0});
  EXPECT_NEAR(pid.integral(), 5.5, 1e-12);
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.5), 5.5, 1e-12);
  EXPECT_NEAR(pid.integral(), 5.5, 1e-12);
}

void test_float_controller() {
  openpid::Config<float> config;
  config.gains = {2.0f, 0.0f, 0.0f};
  openpid::Controller<float> pid(config);
  EXPECT_NEAR(pid.update(3.0f, 1.0f, 0.5f), 4.0f, 1e-6f);
}

void test_incremental_first_step_and_saturation() {
  openpid::IncrementalController<double> pid({2.0, 4.0, 8.0});
  EXPECT_NEAR(pid.update(3.0, 0.0, 0.5), 12.0, 1e-12);
  EXPECT_NEAR(pid.last_delta(), 12.0, 1e-12);
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.5), 10.0, 1e-12);
  EXPECT_NEAR(pid.last_delta(), -2.0, 1e-12);

  openpid::Limits<double> limits;
  limits.output_min = -5.0;
  limits.output_max = 5.0;
  openpid::IncrementalController<double> saturated({10.0, 0.0, 0.0}, limits);
  EXPECT_NEAR(saturated.update(1.0, 0.0, 1.0), 5.0, 1e-12);
  EXPECT_NEAR(saturated.last_delta(), 5.0, 1e-12);
  EXPECT_NEAR(saturated.update(2.0, 0.0, 1.0), 5.0, 1e-12);
  EXPECT_NEAR(saturated.last_delta(), 0.0, 1e-12);
}

void test_incremental_derivative_starts_on_third_sample() {
  openpid::IncrementalController<double> pid({0.0, 0.0, 2.0});
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.5), 0.0, 1e-12);
  EXPECT_NEAR(pid.update(3.0, 0.0, 0.5), 0.0, 1e-12);
  EXPECT_NEAR(pid.update(3.0, 0.0, 0.5), -8.0, 1e-12);
  EXPECT_NEAR(pid.last_delta(), -8.0, 1e-12);
}

void test_non_positive_dt_on_incremental() {
  openpid::IncrementalController<double> pid({2.0, 0.0, 0.0});
  pid.reset(1.25);
  EXPECT_NEAR(pid.update(4.0, 0.0, 0.0), 1.25, 1e-12);
  EXPECT_NEAR(pid.last_delta(), 0.0, 1e-12);
}

} // namespace

int main() {
  test_proportional_only();
  test_integral_accumulates_constant_error();
  test_derivative_on_measurement_has_no_setpoint_kick();
  test_derivative_on_error_reacts_to_setpoint_step();
  test_derivative_on_measurement_formula_and_filter();
  test_setpoint_weight();
  test_output_clamp();
  test_integral_limit();
  test_conditional_integration_does_not_wind_up();
  test_back_calculation_holds_integral_at_saturation();
  test_non_positive_dt_keeps_last_output();
  test_reset_presets_integral_and_gains_do_not_clear_state();
  test_float_controller();
  test_incremental_first_step_and_saturation();
  test_incremental_derivative_starts_on_third_sample();
  test_non_positive_dt_on_incremental();

  if (g_failed != 0) {
    std::cerr << g_failed << " assertion(s) failed\n";
    return 1;
  }
  std::cout << "all tests passed\n";
  return 0;
}
