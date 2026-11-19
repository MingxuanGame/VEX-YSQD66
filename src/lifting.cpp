#include "lifting.hpp"

#include "lemlib/exitcondition.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/timer.hpp"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <numeric>

namespace lifting {
namespace {

constexpr int kJogPower = 20;
constexpr std::uint32_t kControlPeriodMs = 20;
// 电机轴角度，近限位开始减速的初值，需根据实机制动距离调整。
constexpr double kSlowZoneDeg = 45.0;
constexpr char kLimitsPath[] = "/usd/lift_limits.csv";

// TODO: 升降位置 PID 与退出判据均为初值，需按实机负载与摩擦标定。
constexpr float kLiftKp = 0.8F;
constexpr float kLiftKi = 0.0F;
constexpr float kLiftKd = 1.5F;
constexpr float kLiftSettleRangeDeg = 6.0F;
constexpr int kLiftSettleTimeMs = 120;

// V5 智能电机，绿齿轮 200 rpm。暂按镜像安装取 P9 正向、P10 反转，装车须确认。
pros::Motor _g_motor_9(9, pros::MotorGears::green, pros::MotorUnits::degrees);
pros::Motor _g_motor_10(-10, pros::MotorGears::green, pros::MotorUnits::degrees);
std::optional<Limits> _g_limits;
std::optional<Position> _g_lower_reference;
std::optional<Position> _g_saved_travel;

pros::Mutex _g_motion_mutex;
std::unique_ptr<pros::Task> _g_motion_task;
std::atomic<bool> _g_motion_active{false};
std::atomic<bool> _g_cancel_requested{false};

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

bool autonomous_enabled() {
    return !pros::competition::is_disabled() && pros::competition::is_autonomous();
}

bool configure_hold() {
    const auto motor_9 = _g_motor_9.set_brake_mode(pros::MotorBrake::hold);
    const auto motor_10 = _g_motor_10.set_brake_mode(pros::MotorBrake::hold);
    return motor_9 == 1 && motor_10 == 1;
}

void brake_motors() {
    _g_motor_9.brake();
    _g_motor_10.brake();
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
    brake_motors();
    return 0;
}

int limit_power(int power) {
    if (!_g_limits || power == 0) {
        brake_motors();
        return 0;
    }
    const auto position = read_position();
    if (!position) {
        brake_motors();
        return 0;
    }
    const bool up = power > 0;
    const auto &limit = up ? _g_limits->upper_deg : _g_limits->lower_deg;
    const double remaining_deg =
        up ? std::min(limit[0] - (*position)[0], limit[1] - (*position)[1])
           : std::min((*position)[0] - limit[0], (*position)[1] - limit[1]);
    if (remaining_deg <= 0.0) {
        brake_motors();
        return 0;
    }
    const int max_power = remaining_deg <= kSlowZoneDeg ? kJogPower : 127;
    return apply_power(std::clamp(power, -max_power, max_power));
}

void cancel_motion_task() {
    _g_cancel_requested.store(true);
    while (_g_motion_active.load()) {
        pros::delay(2);
    }
    _g_motion_task.reset();
    _g_cancel_requested.store(false);
}

bool run_move_to_blocking(double target_offset_deg, std::uint32_t timeout_ms, int max_power) {
    lemlib::PID pid(kLiftKp, kLiftKi, kLiftKd, 0.0F, true);
    lemlib::ExitCondition settled(kLiftSettleRangeDeg, kLiftSettleTimeMs);
    lemlib::Timer timer(timeout_ms);
    const int capped_power = std::clamp(std::abs(max_power), 1, 127);

    while (!timer.isDone()) {
        std::unique_lock<pros::Mutex> lock(_g_motion_mutex);
        if (_g_cancel_requested.load() || !autonomous_enabled()) {
            brake_motors();
            return false;
        }
        const auto current = offset_deg();
        if (!current) {
            brake_motors();
            return false;
        }
        const double error_deg = target_offset_deg - *current;
        if (settled.update(static_cast<float>(std::abs(error_deg)))) {
            brake_motors();
            return true;
        }
        const int cmd =
            std::clamp(static_cast<int>(std::lround(pid.update(static_cast<float>(error_deg)))),
                       -capped_power, capped_power);
        if (cmd != 0 && limit_power(cmd) == 0) {
            // 到达限位或编码器失效即结束。
            const bool valid = _g_limits.has_value();
            brake_motors();
            return valid;
        }
        lock.unlock();
        pros::delay(kControlPeriodMs);
    }
    std::lock_guard<pros::Mutex> lock(_g_motion_mutex);
    brake_motors();
    return false;
}

bool run_move_for_blocking(int power, std::uint32_t duration_ms) {
    lemlib::Timer timer(duration_ms);
    while (!timer.isDone()) {
        std::unique_lock<pros::Mutex> lock(_g_motion_mutex);
        if (_g_cancel_requested.load() || !autonomous_enabled()) {
            brake_motors();
            return false;
        }
        if (power != 0 && limit_power(power) == 0) {
            const bool valid = _g_limits.has_value();
            brake_motors();
            return valid;
        }
        lock.unlock();
        pros::delay(kControlPeriodMs);
    }
    std::lock_guard<pros::Mutex> lock(_g_motion_mutex);
    brake_motors();
    return true;
}

} // namespace

void initialize() {
    cancel_motion_task();
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
        brake_motors();
        return std::nullopt;
    }
    return position_deg;
}

std::optional<double> offset_deg() {
    if (!_g_limits) {
        return std::nullopt;
    }
    const auto position = read_position();
    if (!position) {
        return std::nullopt;
    }
    return std::midpoint((*position)[0] - _g_limits->lower_deg[0],
                         (*position)[1] - _g_limits->lower_deg[1]);
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
    if (_g_motion_active.load()) {
        cancel_motion_task();
    }
    return limit_power(driver_enabled() ? power : 0);
}

int set_autonomous_power(int power) {
    if (!autonomous_enabled()) {
        brake_motors();
        return 0;
    }
    return limit_power(power);
}

bool move_to(double target_offset_deg, std::uint32_t timeout_ms, int max_power, bool async) {
    cancel_motion_task();
    if (!autonomous_enabled() || !_g_limits || timeout_ms == 0) {
        brake_motors();
        return false;
    }
    const double max_travel = std::min(_g_limits->upper_deg[0] - _g_limits->lower_deg[0],
                                       _g_limits->upper_deg[1] - _g_limits->lower_deg[1]);
    const double clamped_target = std::clamp(target_offset_deg, 0.0, max_travel);
    if (!async) {
        return run_move_to_blocking(clamped_target, timeout_ms, max_power);
    }
    _g_cancel_requested.store(false);
    _g_motion_active.store(true);
    _g_motion_task = std::make_unique<pros::Task>([clamped_target, timeout_ms, max_power]() {
        run_move_to_blocking(clamped_target, timeout_ms, max_power);
        _g_motion_active.store(false);
    });
    return true;
}

bool move_for(int power, std::uint32_t duration_ms, bool async) {
    cancel_motion_task();
    if (!autonomous_enabled() || !_g_limits || duration_ms == 0) {
        brake_motors();
        return false;
    }
    if (!async) {
        return run_move_for_blocking(power, duration_ms);
    }
    _g_cancel_requested.store(false);
    _g_motion_active.store(true);
    _g_motion_task = std::make_unique<pros::Task>([power, duration_ms]() {
        run_move_for_blocking(power, duration_ms);
        _g_motion_active.store(false);
    });
    return true;
}

void wait_until_done() {
    while (_g_motion_active.load()) {
        if (!autonomous_enabled()) {
            cancel_motion_task();
            return;
        }
        pros::delay(kControlPeriodMs);
    }
}

bool is_moving() {
    return _g_motion_active.load();
}

void jog(int direction) {
    if (_g_motion_active.load()) {
        cancel_motion_task();
    }
    if (direction == 0 || !driver_enabled() || pros::competition::is_connected() ||
        !read_position()) {
        brake_motors();
        return;
    }
    apply_power(direction > 0 ? kJogPower : -kJogPower);
}

void stop() {
    cancel_motion_task();
    std::lock_guard<pros::Mutex> lock(_g_motion_mutex);
    brake_motors();
}

} // namespace lifting
