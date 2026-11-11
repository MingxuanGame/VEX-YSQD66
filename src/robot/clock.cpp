#include "robot/clock.hpp"
#include "pros/rtos.hpp"

/**
 * @file clock.cpp
 * @brief 真车时间实现。只有硬件层可以包含 pros/rtos.hpp。
 */

namespace robot {

uint32_t nowMs() {
    return pros::millis();
}

void delayMs(uint32_t ms) {
    pros::delay(ms);
}

} // namespace robot
