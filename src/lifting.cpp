#include "lifting.hpp"

#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace lifting {
namespace {

constexpr std::uint32_t kPositionPollMs = 20;
constexpr double kPositionToleranceDeg = 3.0; // 初值，待实机标定。

// V5 智能电机，红齿轮。P9 正向、P10 反转。
pros::Motor _g_motor_9(9, pros::MotorGears::red, pros::MotorUnits::degrees);
pros::Motor _g_motor_10(-10, pros::MotorGears::red, pros::MotorUnits::degrees);

std::optional<double> read_angle_deg(const pros::Motor &motor) {
    const double angle_deg = motor.get_position();
    return std::isfinite(angle_deg) ? std::optional{angle_deg} : std::nullopt;
}

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
    _g_motor_9.tare_position();
    _g_motor_10.tare_position();
}

MotorAngles motor_angles() {
    return {read_angle_deg(_g_motor_9), read_angle_deg(_g_motor_10)};
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

bool move_to(TargetAngles target_deg, std::int32_t max_velocity_rpm, std::uint32_t timeout_ms) {
    if (!enabled_in_mode(true) || timeout_ms == 0 || max_velocity_rpm <= 0 ||
        !std::isfinite(target_deg.port_9_deg) || !std::isfinite(target_deg.port_10_deg)) {
        return false;
    }

    const auto start_ms = pros::millis();
    const auto initial = motor_angles();
    if (!initial.port_9_deg || !initial.port_10_deg || !enabled_in_mode(true) ||
        _g_motor_9.move_absolute(target_deg.port_9_deg, max_velocity_rpm) != 1 ||
        _g_motor_10.move_absolute(target_deg.port_10_deg, max_velocity_rpm) != 1) {
        stop();
        return false;
    }

    for (auto elapsed_ms = pros::millis() - start_ms; elapsed_ms < timeout_ms;
         elapsed_ms = pros::millis() - start_ms) {
        if (!enabled_in_mode(true)) {
            break;
        }
        const auto position = motor_angles();
        if (!position.port_9_deg || !position.port_10_deg) {
            break;
        }
        if (std::abs(*position.port_9_deg - target_deg.port_9_deg) <= kPositionToleranceDeg &&
            std::abs(*position.port_10_deg - target_deg.port_10_deg) <= kPositionToleranceDeg) {
            stop();
            return enabled_in_mode(true);
        }
        pros::delay(std::min(kPositionPollMs, timeout_ms - elapsed_ms));
    }
    stop();
    return false;
}

void stop() {
    _g_motor_9.brake();
    _g_motor_10.brake();
}

} // namespace lifting
