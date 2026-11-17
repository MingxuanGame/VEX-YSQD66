#include "chassis.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

#include <algorithm>

namespace chassis {
namespace {

// 功率偏移量：正值补偿右侧，负值补偿左侧。
constexpr int kRightMotorOffsetToLeft = 0;

// 面向车头：左前 2、左后 1，右前 3、右后 4；蓝齿轮，编码器单位为电机轴角度。
pros::MotorGroup _g_left({2, 1}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup _g_right({-3, -4}, pros::MotorGears::blue, pros::MotorUnits::degrees);

int apply_offset(int power, int offset) {
    // 两侧预留相同补偿余量，避免满功率时限幅吞掉偏移；零输入不启动电机。
    constexpr int kOffsetMagnitude =
        kRightMotorOffsetToLeft < 0 ? -kRightMotorOffsetToLeft : kRightMotorOffsetToLeft;
    constexpr int kMaxBasePower = 127 - kOffsetMagnitude;
    const int base_power = std::clamp(power, -kMaxBasePower, kMaxBasePower);
    if (power > 0) {
        return base_power + offset;
    }
    if (power < 0) {
        return base_power - offset;
    }
    return 0;
}

} // namespace

void initialize() {
    _g_left.set_brake_mode_all(pros::MotorBrake::brake);
    _g_right.set_brake_mode_all(pros::MotorBrake::brake);
}

void stop() {
    _g_left.brake();
    _g_right.brake();
}

void reset_state() {
    stop();
    _g_left.set_brake_mode_all(pros::MotorBrake::brake);
    _g_right.set_brake_mode_all(pros::MotorBrake::brake);
    _g_left.tare_position_all();
    _g_right.tare_position_all();
}

void set_power(int left_power, int right_power) {
    if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
        stop();
        return;
    }
    const int adjusted_left = apply_offset(left_power, std::max(0, -kRightMotorOffsetToLeft));
    const int adjusted_right = apply_offset(right_power, std::max(0, kRightMotorOffsetToLeft));
    _g_left.move(adjusted_left);
    _g_right.move(adjusted_right);
}

} // namespace chassis
