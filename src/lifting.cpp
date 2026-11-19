#include "lifting.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace lifting {
namespace {

constexpr int kJogPower = 20;
// 电机轴角度，近限位开始减速的初值，需根据实机制动距离调整。
constexpr double kSlowZoneDeg = 45.0;
constexpr char kLimitsPath[] = "/usd/lift_limits.csv";

// V5 智能电机，绿齿轮 200 rpm。暂按镜像安装取 P9 正向、P10 反转，装车须确认。
pros::Motor _g_motor_9(9, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::Motor _g_motor_10(-10, pros::MotorGears::green, pros::MotorUnits::degrees);
std::optional<Limits> _g_limits;
std::optional<Position> _g_lower_reference;
std::optional<Position> _g_saved_travel;

std::optional<Position> read_saved_travel() {
    auto *file = std::fopen(kLimitsPath, "rb");
    if (!file) {
        return std::nullopt;
    }
    Position travel{};
    int end = 0;
    char line[128]{};
    const bool valid =
        std::fgets(line, sizeof(line), file) &&
        std::sscanf(line, "LIFT_LIMITS,1,%lf,%lf,END%n", &travel[0], &travel[1], &end) == 2 &&
        end > 0 && line[end] == '\n' && line[end + 1] == '\0' && std::fgetc(file) == EOF &&
        !std::ferror(file);
    if (std::fclose(file) != 0 || !valid || !std::isfinite(travel[0]) ||
        !std::isfinite(travel[1]) || travel[0] <= 0 || travel[1] <= 0) {
        return std::nullopt;
    }
    return travel;
}

void apply_saved_travel() {
    _g_limits.reset();
    if (_g_lower_reference && _g_saved_travel) {
        const auto &lower = *_g_lower_reference;
        const auto &travel = *_g_saved_travel;
        Position upper{lower[0] + travel[0], lower[1] + travel[1]};
        if (std::isfinite(upper[0]) && std::isfinite(upper[1]) && upper[0] > lower[0] &&
            upper[1] > lower[1]) {
            _g_limits = Limits{lower, upper};
        }
    }
}

bool driver_enabled() {
    // Override 2.1 <R9>：仅在启用的手动控制期间驱动升降。
    return !pros::competition::is_disabled() && !pros::competition::is_autonomous();
}

bool configure_hold() {
    const auto motor_9 = _g_motor_9.set_brake_mode(pros::MotorBrake::hold);
    const auto motor_10 = _g_motor_10.set_brake_mode(pros::MotorBrake::hold);
    return motor_9 == 1 && motor_10 == 1;
}

int apply_power(int power) {
    if (power != 0) {
        // PROS 4.2.3 的 MotorGroup::move() 只返回首端口的结果。
        if (_g_motor_9.move(power) == 1 && _g_motor_10.move(power) == 1) {
            return power;
        }
        _g_limits.reset();
        _g_lower_reference.reset();
    }
    stop();
    return 0;
}

} // namespace

void initialize() {
    _g_lower_reference.reset();
    configure_hold();
    load_limits();
}

void load_limits() {
    _g_saved_travel = read_saved_travel();
    if (_g_lower_reference) {
        read_position();
    }
    apply_saved_travel();
}

bool has_saved_limits() {
    return _g_saved_travel.has_value();
}

bool use_saved_limits(const Position &lower_deg) {
    stop();
    load_limits();
    if (!_g_saved_travel || !configure_hold()) {
        return false;
    }
    _g_lower_reference = lower_deg;
    apply_saved_travel();
    return _g_limits.has_value();
}

std::optional<Position> read_position() {
    const Position position_deg{_g_motor_9.get_position(), _g_motor_10.get_position()};
    if (!std::isfinite(position_deg[0]) || !std::isfinite(position_deg[1])) {
        _g_limits.reset();
        _g_lower_reference.reset();
        stop();
        return std::nullopt;
    }
    return position_deg;
}

const std::optional<Limits> &limits() {
    return _g_limits;
}

bool set_limits(const Limits &measured) {
    // 由现场确定留有制动余量的端点，并核对 Override 2.1 <SG2>、<SG3> 的尺寸限制。
    const Position travel{measured.upper_deg[0] - measured.lower_deg[0],
                          measured.upper_deg[1] - measured.lower_deg[1]};
    stop();
    if (!std::isfinite(travel[0]) || !std::isfinite(travel[1]) || travel[0] <= 0 ||
        travel[1] <= 0 || !configure_hold()) {
        return false;
    }
    auto *file = std::fopen(kLimitsPath, "wb");
    if (!file) {
        return false;
    }
    const bool written =
        std::fprintf(file, "LIFT_LIMITS,1,%.17g,%.17g,END\n", travel[0], travel[1]) > 0;
    const bool closed = std::fclose(file) == 0;
    auto saved = read_saved_travel();
    if (!written || !closed || saved != travel) {
        return false;
    }
    _g_saved_travel = saved;
    _g_lower_reference = measured.lower_deg;
    apply_saved_travel();
    return _g_limits.has_value();
}

int set_power(int power) {
    if (!driver_enabled() || !_g_limits) {
        stop();
        return 0;
    }
    const auto position = read_position();
    if (!position || power == 0) {
        stop();
        return 0;
    }
    const bool up = power > 0;
    const auto &limit = up ? _g_limits->upper_deg : _g_limits->lower_deg;
    const double remaining_deg =
        up ? std::min(limit[0] - (*position)[0], limit[1] - (*position)[1])
           : std::min((*position)[0] - limit[0], (*position)[1] - limit[1]);
    if (remaining_deg <= 0.0) {
        stop();
        return 0;
    }
    const int max_power = remaining_deg <= kSlowZoneDeg ? kJogPower : 127;
    return apply_power(std::clamp(power, -max_power, max_power));
}

void jog(int direction) {
    if (direction == 0 || !driver_enabled() || pros::competition::is_connected() ||
        !read_position()) {
        stop();
        return;
    }
    apply_power(direction > 0 ? kJogPower : -kJogPower);
}

void stop() {
    _g_motor_9.brake();
    _g_motor_10.brake();
}

} // namespace lifting
