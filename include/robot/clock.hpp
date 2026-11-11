#pragma once
#include <cstdint>

/**
 * @file clock.hpp
 * @brief 时间接口声明。
 *        固件实现和仿真实现分别放在 src/robot/clock.cpp 与 sim/compat/clock_sim.cpp。
 */

namespace robot {

uint32_t nowMs();
void delayMs(uint32_t ms);

} // namespace robot
