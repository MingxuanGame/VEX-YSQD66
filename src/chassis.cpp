#include "chassis.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

#include <algorithm>
#include <limits>

namespace chassis {
namespace {
// 面向车头：左前 2、左后 1，右前 3、右后 4；蓝齿轮，编码器为电机轴角度。
// TODO：左侧正向、右侧反转沿用原左右侧约定，正功率向前仍需实机核对。
pros::MotorGroup _g_left({2, 1}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup _g_right({-3, -4}, pros::MotorGears::blue, pros::MotorUnits::degrees);
} // namespace

void initialize() {
    _g_left.set_brake_mode_all(pros::MotorBrake::brake);
    _g_right.set_brake_mode_all(pros::MotorBrake::brake);
}

void stop() {
    _g_left.move_voltage(0);
    _g_right.move_voltage(0);
    _g_left.brake();
    _g_right.brake();
}

bool set_power(int left_power, int right_power) {
    if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
        stop();
        return false;
    }
    const auto left_result = _g_left.move(std::clamp(left_power, -127, 127));
    const auto right_result = _g_right.move(std::clamp(right_power, -127, 127));
    if (left_result == std::numeric_limits<std::int32_t>::max() ||
        right_result == std::numeric_limits<std::int32_t>::max()) {
        stop();
        return false;
    }
    return true;
}
} // namespace chassis
