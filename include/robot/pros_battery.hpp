#pragma once
#include "robot/battery.hpp"

/**
 * @file pros_battery.hpp
 * @brief 真车电池工厂声明。实现放在 src/robot/battery.cpp。
 */

namespace robot {

Battery &prosBattery();

} // namespace robot
