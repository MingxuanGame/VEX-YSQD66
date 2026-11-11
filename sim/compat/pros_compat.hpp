#pragma once

/**
 * @file pros_compat.hpp
 * @brief PROS 仿真兼容层声明。
 *        具体时间函数实现在 sim/compat/pros_compat.cpp，头文件不再写函数体。
 */

#include "sim/core/sim_clock.hpp"
#include "sim/hardware/sim_imu.hpp"
#include "sim/hardware/sim_motor.hpp"

#ifdef ENABLE_SIMULATOR

namespace pros {

// ---- 核心硬件类映射 ----
using Motor = sim::Motor;
using Imu = sim::Imu;

// ---- 齿轮箱枚举映射（保持 PROS 宏名以兼容业务代码） ----
using MotorGearset = sim::Gearset;
constexpr auto E_MOTOR_GEARSET_18 = sim::Gearset::Green;
constexpr auto E_MOTOR_GEARSET_36 = sim::Gearset::Red;
constexpr auto E_MOTOR_GEARSET_06 = sim::Gearset::Blue;

// ---- 刹车模式枚举映射 ----
using MotorBrakeMode = sim::BrakeMode;
constexpr auto E_MOTOR_BRAKE_COAST = sim::BrakeMode::Coast;
constexpr auto E_MOTOR_BRAKE_BRAKE = sim::BrakeMode::Brake;
constexpr auto E_MOTOR_BRAKE_HOLD = sim::BrakeMode::Hold;

// ---- 时间 API ----
void delay(uint32_t ms);
uint32_t millis();

} // namespace pros

#endif // ENABLE_SIMULATOR
