#include "lifting.hpp"

#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lifting {
namespace {

constexpr std::uint32_t kPositionPollMs = 20;
constexpr double kPositionToleranceDeg = 3.0; // TODO: 实机标定位置容差。需要测定。
// TODO: 留出减速距离，初值沿用原升降限位设置。需要测定。
constexpr double kSlowZoneDeg = 45.0;
constexpr int kNearLimitPower = 20;

enum class PositionStatus { Moving, Reached, Failed };

pros::Motor _g_motor_9(9, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::Motor _g_motor_10(-10, pros::MotorGears::green, pros::MotorUnits::degrees);
bool _g_reference_ready = false;   // 电机是否准备好

bool initialize_motor(const pros::Motor &motor) {
    return motor.set_brake_mode(pros::MotorBrake::hold) == 1 && motor.tare_position() == 1;
}

std::optional<double> read_angle_deg(const pros::Motor &motor) {
    const double angle_deg = motor.get_position();
    return std::isfinite(angle_deg) ? std::optional{angle_deg} : std::nullopt;
}

int limit_power(int power) {
    if (!_g_reference_ready || power == 0) {
        return 0;
    }
    const auto angles = motor_angles();
    if (!angles.port_9_deg || !angles.port_10_deg) {
        return 0;
    }
    const double remaining_deg =
        power > 0 ? kLowerLimitDeg - std::max(*angles.port_9_deg, *angles.port_10_deg)
                  : std::min(*angles.port_9_deg, *angles.port_10_deg) - kUpperLimitDeg;
    if (remaining_deg <= 0.0) {
        return 0;
    }
    const int max_power = remaining_deg <= kSlowZoneDeg ? kNearLimitPower : 127;
    return std::clamp(power, -max_power, max_power);
}

int apply_power(int power) {
    power = limit_power(power);
    if (power != 0 && !pros::competition::is_disabled() && _g_motor_9.move(power) == 1 &&
        _g_motor_10.move(power) == 1) {
        return power;
    }
    stop();
    return 0;
}

bool within_limits(double angle_deg) {
    return angle_deg >= kUpperLimitDeg && angle_deg <= kLowerLimitDeg;
}

bool beyond_travel(double angle_deg, double initial_deg) {
    // 允许由限位外向范围内退回，不能继续向外走。
    return angle_deg < std::min(kUpperLimitDeg, initial_deg) ||
           angle_deg > std::max(kLowerLimitDeg, initial_deg);
}

std::optional<std::pair<double, double>> start_position_move(double target_deg,
                                                             std::int32_t velocity_rpm) {
    const auto initial = motor_angles();
    if (!initial.port_9_deg || !initial.port_10_deg ||
        _g_motor_9.move_absolute(target_deg, velocity_rpm) != 1 ||
        _g_motor_10.move_absolute(target_deg, velocity_rpm) != 1) {
        return std::nullopt;
    }
    return std::pair{*initial.port_9_deg, *initial.port_10_deg};
}

PositionStatus check_position(double target_deg, const std::pair<double, double> &initial_deg) {
    const auto position = motor_angles();
    if (!position.port_9_deg || !position.port_10_deg ||
        beyond_travel(*position.port_9_deg, initial_deg.first) ||
        beyond_travel(*position.port_10_deg, initial_deg.second)) {
        return PositionStatus::Failed;
    }
    if (within_limits(*position.port_9_deg) && within_limits(*position.port_10_deg) &&
        std::abs(*position.port_9_deg - target_deg) <= kPositionToleranceDeg &&
        std::abs(*position.port_10_deg - target_deg) <= kPositionToleranceDeg) {
        return PositionStatus::Reached;
    }
    return PositionStatus::Moving;
}

} // namespace

void initialize() {
    const bool port_9_ready = initialize_motor(_g_motor_9);
    const bool port_10_ready = initialize_motor(_g_motor_10);
    _g_reference_ready = port_9_ready && port_10_ready;
}

MotorAngles motor_angles() {
    return {read_angle_deg(_g_motor_9), read_angle_deg(_g_motor_10)};
}

int set_power(int power) {
    return apply_power(power);
}

bool move_for(int power, std::uint32_t duration_ms) {
    const auto start_ms = pros::millis();
    for (auto elapsed_ms = 0U; elapsed_ms < duration_ms; elapsed_ms = pros::millis() - start_ms) {
        if (apply_power(power) == 0) {
            return false;
        }
        pros::delay(std::min(kPositionPollMs, duration_ms - elapsed_ms));
    }
    stop();
    return !pros::competition::is_disabled();
}

bool move_to(double target_deg, std::int32_t max_velocity_rpm, std::uint32_t timeout_ms) {
    if (!_g_reference_ready || timeout_ms == 0 || max_velocity_rpm <= 0 ||
        !within_limits(target_deg)) {
        return false;
    }

    const auto start_ms = pros::millis();
    const auto initial = !pros::competition::is_disabled()
                             ? start_position_move(target_deg, max_velocity_rpm)
                             : std::nullopt;
    if (!initial) {
        stop();
        return false;
    }

    for (auto elapsed_ms = pros::millis() - start_ms; elapsed_ms < timeout_ms;
         elapsed_ms = pros::millis() - start_ms) {
        if (pros::competition::is_disabled()) {
            break;
        }
        const auto status = check_position(target_deg, *initial);
        if (status != PositionStatus::Moving) {
            stop();
            return status == PositionStatus::Reached && !pros::competition::is_disabled();
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
