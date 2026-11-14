#include "chassis.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

#include <algorithm>
#include <limits>

namespace chassis {
namespace {
// 沿用原设备映射：左 3/4 正向、右 1/2 反向；蓝齿轮，编码器为电机轴角度。
// TODO：实机核对正功率是否向前；沿用原手动安装方向，不声称已标定。
pros::MotorGroup _g_left({3, 4}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup _g_right({-1, -2}, pros::MotorGears::blue, pros::MotorUnits::degrees);
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
