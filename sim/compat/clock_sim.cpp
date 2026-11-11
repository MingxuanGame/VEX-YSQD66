#include "robot/clock.hpp"
#include "sim/core/sim_clock.hpp"

/**
 * @file clock_sim.cpp
 * @brief 仿真时间实现。
 */

namespace robot {

uint32_t nowMs() {
    return sim::millis();
}

void delayMs(uint32_t ms) {
    sim::delay(ms);
}

} // namespace robot
