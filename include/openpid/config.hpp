#pragma once

#include <limits>

namespace openpid {

template <typename T> struct OrderedRange {
  T min;
  T max;
};

template <typename T> constexpr OrderedRange<T> ordered_range(T a, T b) {
  if (a <= b) {
    return OrderedRange<T>{a, b};
  }
  return OrderedRange<T>{b, a};
}

namespace detail {

template <typename T> constexpr T clamp(T value, T lo, T hi) {
  const OrderedRange<T> range = ordered_range(lo, hi);
  if (value < range.min) {
    return range.min;
  }
  if (value > range.max) {
    return range.max;
  }
  return value;
}

} // namespace detail

enum class DerivativeMode {
  OnError,
  OnMeasurement,
};

enum class AntiWindup {
  ConditionalIntegration,
  BackCalculation,
};

template <typename T = double> struct Gains {
  T kp{};
  T ki{};
  T kd{};
};

template <typename T = double> struct Limits {
  T output_min = std::numeric_limits<T>::lowest();
  T output_max = std::numeric_limits<T>::max();
  T integral_min = std::numeric_limits<T>::lowest();
  T integral_max = std::numeric_limits<T>::max();
};

// 离散 PID 的全部可调参数。热路径只读取这些值，不分配内存。
// back_calculation_gain 为 0 时，反算增益取 Ki；Ki 也为 0 时反算关闭。
template <typename T = double> struct Config {
  Gains<T> gains{};
  Limits<T> limits{};
  DerivativeMode derivative = DerivativeMode::OnMeasurement;
  AntiWindup anti_windup = AntiWindup::ConditionalIntegration;
  T derivative_filter_tau{};
  T setpoint_weight = T{1};
  T back_calculation_gain{};
};

} // namespace openpid
