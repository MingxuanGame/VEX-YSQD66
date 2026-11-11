#pragma once
#include "config/motion_config.hpp"

/**
 * @file units.hpp
 * @brief 单位换算声明。实现放在 src/vxmath/units.cpp。
 */

namespace vxmath {

double ticksToInches(double ticks);
double inchesToTicks(double inches);
double rpmToInchesPerSec(double rpm);
double inchesPerSecToRpm(double ips);

int clampServoAngle(int angle);
int clampMotorOutput(int out);
double clampMotorOutput(double out);

int percentToMotor(double percent);
double motorToPercent(int out);

} // namespace vxmath
