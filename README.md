# OpenPID

[![CI](https://github.com/dxyinme/openpid/actions/workflows/ci.yml/badge.svg)](https://github.com/dxyinme/openpid/actions/workflows/ci.yml)

[中文](README-CN.md)

A header-only C++17 discrete PID library. The hot path does not allocate or throw. The scalar type defaults to `double` and can also be `float`.

## Features

- Positional PID: proportional, integral, and derivative terms, with output and integral limits
- Setpoint weighting, and a derivative taken on the error or on the measurement
- First-order low-pass filter on the derivative
- Anti-windup by conditional integration or back-calculation
- Incremental PID that freezes further increments once the output is saturated in that direction

## Build

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/openpid_first_order
```

Tests and examples are built by default when this repository is the top-level project. Both are off by default when the project is added as a subdirectory. After installation, link `openpid::openpid` via `find_package(openpid)`.

## Quick start

```cpp
#include "openpid/openpid.hpp"

openpid::Config<double> config;
config.gains = {1.8, 1.2, 0.15};
config.limits.output_min = -10.0;
config.limits.output_max = 10.0;

openpid::Controller<double> pid(config);
const double output = pid.update(setpoint, measurement, dt);
```

If `dt <= 0`, the step is skipped and the previous output is returned. `reset()` clears the state. `reset(u0)` presets the integral to `u0` for bumpless transfer. `set_gains` and `set_limits` change parameters only and do not reset the state.

## Positional form

The error is `e = setpoint - measurement`, and the sample period is `dt`.

- Proportional: `P = Kp * (b * setpoint - measurement)`, where `b` is the setpoint weight and defaults to `1`
- Integral: `I += Ki * e * dt`, then clamped to the integral limits
- Derivative: on error, `D = Kd * (e - e_prev) / dt`; on measurement, `D = -Kd * (measurement - measurement_prev) / dt`. The first sample has no history, so the derivative is 0
- Filter: when `tau > 0`, `Df += dt / (tau + dt) * (D - Df)`; otherwise the derivative passes through
- Output: `u = clamp(P + I + Df, u_min, u_max)`

The derivative uses the measurement by default, which avoids a derivative kick on a setpoint step. The integral, and the derivative-on-error term, still use the full error.

## Anti-windup

**Conditional integration** (default): if this sample's integral step would push `P + I + D` outside the output limits, integration stops at the limit. Once the output is already saturated, further integration in that direction is dropped. An error of the opposite sign still unwinds the integral.

**Back-calculation**: compute the unsaturated output `u` with the full integral update, saturate it to `u_sat`, then correct the integral:

```text
I += Kb * (u_sat - u) * dt
```

When `Kb` is 0, it falls back to `Ki`. When `Ki` is also 0, back-calculation is disabled. The corrected integral is used on the next sample; this sample's output stays at the saturated value.

Integral limits apply together with anti-windup, so the integrator state stays inside its own bounds.

## Incremental form

```text
du = Kp * (e - e_prev) + Ki * e * dt + Kd * (e - 2 * e_prev + e_prev2) / dt
u += du
```

Previous errors start at 0, so the first proportional increment is `Kp * e`. The derivative term stays 0 until two samples of history exist. Once the output is saturated, further increments in that direction are dropped. The incremental controller uses output limits only.

```cpp
openpid::Limits<double> limits;
limits.output_min = -5.0;
limits.output_max = 5.0;
openpid::IncrementalController<double> pid({2.0, 1.0, 0.1}, limits);
const double output = pid.update(setpoint, measurement, dt);
```

## Example

`examples/first_order.cpp` controls a first-order plant, `dy/dt = (-y + u) / tau`, with the positional PID. At the end of the simulation the absolute error is below `0.05`.

## License

MIT. See [LICENSE](LICENSE).
