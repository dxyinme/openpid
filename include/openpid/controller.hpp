#pragma once

#include "openpid/config.hpp"

namespace openpid {

// 位置式离散 PID。
//
// 每个周期由调用方传入 dt。dt <= 0 时不更新状态，直接返回上一次输出。
// 输出使用本拍更新后的积分；反算修正写回积分，供下一拍使用。
// 条件积分会把本拍积分增量限制在输出余量之内，避免积分越过饱和边界。
template <typename T = double> class Controller {
public:
  using value_type = T;

  Controller() = default;
  explicit Controller(const Config<T> &config) : config_(config) {}

  void set_config(const Config<T> &config) { config_ = config; }

  void set_gains(const Gains<T> &gains) { config_.gains = gains; }

  void set_limits(const Limits<T> &limits) { config_.limits = limits; }

  void reset() { reset(T{0}); }

  // 清微分历史，并把积分预置为 initial_output，便于无扰切换。
  void reset(T initial_output) {
    const OrderedRange<T> output = output_range();
    const OrderedRange<T> integral = integral_range();
    integral_ = detail::clamp(initial_output, integral.min, integral.max);
    proportional_ = T{0};
    derivative_ = T{0};
    error_ = T{0};
    last_output_ = detail::clamp(initial_output, output.min, output.max);
    prev_error_ = T{0};
    prev_measurement_ = T{0};
    has_history_ = false;
  }

  [[nodiscard]] T update(T setpoint, T measurement, T dt) {
    if (!(dt > T{0})) {
      return last_output_;
    }

    error_ = setpoint - measurement;
    proportional_ =
        config_.gains.kp * (config_.setpoint_weight * setpoint - measurement);
    derivative_ = apply_derivative_filter(raw_derivative(measurement, dt), dt);

    T delta_i = config_.gains.ki * error_ * dt;
    if (config_.anti_windup == AntiWindup::ConditionalIntegration) {
      delta_i = limit_integral_delta(delta_i);
    }

    integral_ += delta_i;
    integral_ = detail::clamp(integral_, config_.limits.integral_min,
                              config_.limits.integral_max);

    const T unsaturated = proportional_ + integral_ + derivative_;
    const OrderedRange<T> output = output_range();
    const T saturated = detail::clamp(unsaturated, output.min, output.max);

    if (config_.anti_windup == AntiWindup::BackCalculation) {
      integral_ +=
          effective_back_calculation_gain() * (saturated - unsaturated) * dt;
      integral_ = detail::clamp(integral_, config_.limits.integral_min,
                                config_.limits.integral_max);
    }

    prev_error_ = error_;
    prev_measurement_ = measurement;
    has_history_ = true;
    last_output_ = saturated;
    return last_output_;
  }

  [[nodiscard]] T last_output() const { return last_output_; }
  [[nodiscard]] T proportional() const { return proportional_; }
  [[nodiscard]] T integral() const { return integral_; }
  [[nodiscard]] T derivative() const { return derivative_; }
  [[nodiscard]] T error() const { return error_; }
  [[nodiscard]] const Config<T> &config() const { return config_; }

private:
  [[nodiscard]] OrderedRange<T> output_range() const {
    return ordered_range(config_.limits.output_min, config_.limits.output_max);
  }

  [[nodiscard]] OrderedRange<T> integral_range() const {
    return ordered_range(config_.limits.integral_min,
                         config_.limits.integral_max);
  }

  [[nodiscard]] T effective_back_calculation_gain() const {
    if (config_.back_calculation_gain != T{0}) {
      return config_.back_calculation_gain;
    }
    return config_.gains.ki;
  }

  [[nodiscard]] T raw_derivative(T measurement, T dt) const {
    if (!has_history_ || config_.gains.kd == T{0}) {
      return T{0};
    }
    if (config_.derivative == DerivativeMode::OnError) {
      return config_.gains.kd * (error_ - prev_error_) / dt;
    }
    return -config_.gains.kd * (measurement - prev_measurement_) / dt;
  }

  [[nodiscard]] T apply_derivative_filter(T raw, T dt) const {
    if (!(config_.derivative_filter_tau > T{0})) {
      return raw;
    }
    const T alpha = dt / (config_.derivative_filter_tau + dt);
    return derivative_ + alpha * (raw - derivative_);
  }

  // 输出已经顶在限幅上时，丢掉继续朝该方向推的积分；否则只积分到饱和边界。
  [[nodiscard]] T limit_integral_delta(T delta_i) const {
    const OrderedRange<T> output = output_range();
    const T base = proportional_ + integral_ + derivative_;
    if (delta_i > T{0}) {
      if (base >= output.max) {
        return T{0};
      }
      if (base + delta_i > output.max) {
        return output.max - base;
      }
    } else if (delta_i < T{0}) {
      if (base <= output.min) {
        return T{0};
      }
      if (base + delta_i < output.min) {
        return output.min - base;
      }
    }
    return delta_i;
  }

  Config<T> config_{};
  T integral_{};
  T proportional_{};
  T derivative_{};
  T error_{};
  T last_output_{};
  T prev_error_{};
  T prev_measurement_{};
  bool has_history_ = false;
};

} // namespace openpid
