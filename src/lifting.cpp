#include "lifting.hpp"

#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <algorithm>

namespace lifting {
namespace {

// V5 智能电机，红齿轮。P9 正向、P10 反转。
pros::Motor _g_motor_9(9, pros::MotorGears::red, pros::MotorUnits::degrees);
pros::Motor _g_motor_10(-10, pros::MotorGears::red, pros::MotorUnits::degrees);

bool enabled_in_mode(bool autonomous) {
    // Override 2.1 <R9>：仅在相应的竞赛模式启用时驱动。
    return !pros::competition::is_disabled() && pros::competition::is_autonomous() == autonomous;
}

int apply_power(int power) {
    power = std::clamp(power, -127, 127);
    if (power != 0 && _g_motor_9.move(power) == 1 && _g_motor_10.move(power) == 1) {
        return power;
    }
    stop();
    return 0;
}

} // namespace

void initialize() {
    _g_motor_9.set_brake_mode(pros::MotorBrake::hold);
    _g_motor_10.set_brake_mode(pros::MotorBrake::hold);
}

int set_power(int power) {
    return apply_power(enabled_in_mode(false) ? power : 0);
}

bool move_for(int power, std::uint32_t duration_ms) {
    const auto start_ms = pros::millis();
    for (auto elapsed_ms = 0U; elapsed_ms < duration_ms; elapsed_ms = pros::millis() - start_ms) {
        if (!enabled_in_mode(true) || apply_power(power) == 0) {
            stop();
            return false;
        }
        pros::delay(std::min<std::uint32_t>(20, duration_ms - elapsed_ms));
    }
    stop();
    return enabled_in_mode(true);
}

void stop() {
    _g_motor_9.brake();
    _g_motor_10.brake();
}

} // namespace lifting
