#include "chassis.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

#include <algorithm>
#include <cstdlib>
#include <limits>

// 功率偏移量，正值补偿右侧，负值补偿左侧。
#define RIGHT_MOTOR_OFFSET_TO_LEFT 10

namespace chassis {
namespace {
static_assert(RIGHT_MOTOR_OFFSET_TO_LEFT >= -127 && RIGHT_MOTOR_OFFSET_TO_LEFT <= 127,
              "RIGHT_MOTOR_OFFSET_TO_LEFT must be within [-127, 127]");

// 面向车头：左前 2、左后 1，右前 3、右后 4；蓝齿轮，编码器为电机轴角度。
pros::MotorGroup _g_left({2, 1}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup _g_right({-3, -4}, pros::MotorGears::blue, pros::MotorUnits::degrees);

int apply_offset(int power, int offset) {
    // 两侧先预留相同补偿余量，避免满功率时独立限幅吞掉偏移；零输入不启动电机。
    const int max_base_power = 127 - std::abs(RIGHT_MOTOR_OFFSET_TO_LEFT);
    const int base_power = std::clamp(power, -max_base_power, max_base_power);
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
    _g_left.move_voltage(0);
    _g_right.move_voltage(0);
    _g_left.brake();
    _g_right.brake();
}

bool reset_state() {
    stop();
    const auto left_brake = _g_left.set_brake_mode_all(pros::MotorBrake::brake);
    const auto right_brake = _g_right.set_brake_mode_all(pros::MotorBrake::brake);
    const auto left_zero = _g_left.tare_position_all();
    const auto right_zero = _g_right.tare_position_all();
    return left_brake == 1 && right_brake == 1 && left_zero == 1 && right_zero == 1;
}

bool set_power(int left_power, int right_power) {
    if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
        stop();
        return false;
    }
    const int adjusted_left_power =
        apply_offset(left_power, std::max(0, -RIGHT_MOTOR_OFFSET_TO_LEFT));
    const int adjusted_right_power =
        apply_offset(right_power, std::max(0, RIGHT_MOTOR_OFFSET_TO_LEFT));
    const auto left_result = _g_left.move(adjusted_left_power);
    const auto right_result = _g_right.move(adjusted_right_power);
    if (left_result == std::numeric_limits<std::int32_t>::max() ||
        right_result == std::numeric_limits<std::int32_t>::max()) {
        stop();
        return false;
    }
    return true;
}
} // namespace chassis
