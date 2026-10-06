#pragma once

#include "openpid/config.hpp"

namespace openpid {

// 增量式离散 PID：
//   du = Kp * (e - e_prev) + Ki * e * dt + Kd * (e - 2 e_prev + e_prev2) / dt
// 历史误差初值为 0，因此第一拍的比例增量是 Kp * e。
// 凑满两拍历史之前微分项为 0。输出已经顶在限幅上时，朝饱和方向的增量被丢掉。
template <typename T = double> class IncrementalController {
public:
  using value_type = T;

  IncrementalController() = default;

  explicit IncrementalController(Gains<T> gains, Limits<T> limits = {})
      : gains_(gains), limits_(limits) {}

  void set_gains(const Gains<T> &gains) { gains_ = gains; }

  void set_limits(const Limits<T> &limits) { limits_ = limits; }

  void reset() { reset(T{0}); }

  void reset(T initial_output) {
    const OrderedRange<T> output = output_range();
    output_ = detail::clamp(initial_output, output.min, output.max);
    prev_error_ = T{0};
    prev_error2_ = T{0};
    last_delta_ = T{0};
    sample_count_ = 0;
  }

  [[nodiscard]] T update(T setpoint, T measurement, T dt) {
    if (!(dt > T{0})) {
      return output_;
    }

    const T error = setpoint - measurement;
    T delta = gains_.kp * (error - prev_error_) + gains_.ki * error * dt;
    if (sample_count_ >= 2) {
      delta += gains_.kd * (error - T{2} * prev_error_ + prev_error2_) / dt;
    }

    const OrderedRange<T> range = output_range();
    const T previous = output_;
    const bool freeze = (output_ >= range.max && delta > T{0}) ||
                        (output_ <= range.min && delta < T{0});
    if (!freeze) {
      output_ = detail::clamp(output_ + delta, range.min, range.max);
    }
    last_delta_ = output_ - previous;

    prev_error2_ = prev_error_;
    prev_error_ = error;
    if (sample_count_ < 2) {
      ++sample_count_;
    }
    return output_;
  }

  [[nodiscard]] T last_output() const { return output_; }
  [[nodiscard]] T last_delta() const { return last_delta_; }
  [[nodiscard]] const Gains<T> &gains() const { return gains_; }

private:
  [[nodiscard]] OrderedRange<T> output_range() const {
    return ordered_range(limits_.output_min, limits_.output_max);
  }

  Gains<T> gains_{};
  Limits<T> limits_{};
  T output_{};
  T prev_error_{};
  T prev_error2_{};
  T last_delta_{};
  int sample_count_ = 0;
};

} // namespace openpid
