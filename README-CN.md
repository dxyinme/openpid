# OpenPID

[![CI](https://github.com/dxyinme/openpid/actions/workflows/ci.yml/badge.svg)](https://github.com/dxyinme/openpid/actions/workflows/ci.yml)

[English](README.md)

header-only 的 C++17 离散 PID 库。热路径不分配内存、不抛异常，标量类型默认为 `double`，也可以实例化为 `float`。

## 特性

- 位置式 PID：比例、积分、微分，输出限幅，积分限幅
- 设定值权重，微分可对误差或对测量值
- 微分一阶低通
- 条件积分与反算两种抗积分饱和
- 增量式 PID，饱和时冻结继续顶限幅的增量

## 构建

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/openpid_first_order
```

本仓库作为顶层工程时会默认编译测试和示例。作为子目录引入时这两项默认关闭。安装后可通过 `find_package(openpid)` 链接 `openpid::openpid`。

## 快速使用

```cpp
#include "openpid/openpid.hpp"

openpid::Config<double> config;
config.gains = {1.8, 1.2, 0.15};
config.limits.output_min = -10.0;
config.limits.output_max = 10.0;

openpid::Controller<double> pid(config);
const double output = pid.update(setpoint, measurement, dt);
```

`dt <= 0` 时本拍不更新，返回上一次输出。`reset()` 清状态；`reset(u0)` 把积分预置为 `u0`，便于无扰切换。`set_gains` 和 `set_limits` 只改参数，不重置状态。

## 位置式

记误差 `e = setpoint - measurement`，采样周期为 `dt`。

- 比例：`P = Kp * (b * setpoint - measurement)`，`b` 为设定值权重，默认 `1`
- 积分：`I += Ki * e * dt`，再夹紧到积分限幅
- 微分：对误差时 `D = Kd * (e - e_prev) / dt`；对测量值时 `D = -Kd * (measurement - measurement_prev) / dt`。第一拍没有历史，微分为 0
- 滤波：`tau > 0` 时 `Df += dt / (tau + dt) * (D - Df)`，否则直通
- 输出：`u = clamp(P + I + Df, u_min, u_max)`

默认微分对测量值，避免设定值阶跃造成微分冲击。积分和误差微分仍使用完整误差。

## 抗积分饱和

**条件积分**（默认）：本拍积分增量如果会把 `P + I + D` 推出输出限幅，就只积到边界；输出已经顶在限幅上时，丢掉继续朝该方向的积分。反向误差仍会把积分退回来。

**反算**：先按完整积分算出未饱和输出 `u`，饱和后得到 `u_sat`，再修正积分

```text
I += Kb * (u_sat - u) * dt
```

`Kb` 为 0 时取 `Ki`。`Ki` 也为 0 时反算关闭。反算写回的是下一拍使用的积分，本拍输出保持饱和值。

积分限幅与抗饱和同时生效，积分状态不会超过自己的上下界。

## 增量式

```text
du = Kp * (e - e_prev) + Ki * e * dt + Kd * (e - 2 * e_prev + e_prev2) / dt
u += du
```

历史误差初值为 0，所以第一拍的比例增量是 `Kp * e`。凑满两拍历史之前微分项为 0。输出已经顶在限幅上时，朝饱和方向的增量被丢掉。增量式只使用输出限幅。

```cpp
openpid::Limits<double> limits;
limits.output_min = -5.0;
limits.output_max = 5.0;
openpid::IncrementalController<double> pid({2.0, 1.0, 0.1}, limits);
const double output = pid.update(setpoint, measurement, dt);
```

## 示例

`examples/first_order.cpp` 用位置式 PID 控制一阶惯性对象 `dy/dt = (-y + u) / tau`。仿真结束时绝对误差应小于 `0.05`。

## 许可

MIT，见 [LICENSE](LICENSE)。
