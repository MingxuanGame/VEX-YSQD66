#include "chassis.h"
#include "io/pros_controller.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace chassis {
namespace {
// 沿用原设备映射：左 3/4 正向、右 1/2 反向；蓝齿轮，编码器统一为电机轴角度。
// TODO：实机核对正电压是否向前，以及右正转角是否符合自动赛坐标约定。
pros::MotorGroup _g_left({3, 4}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup _g_right({-1, -2}, pros::MotorGears::blue, pros::MotorUnits::degrees);
constexpr std::array<int, 4> kPorts{3, 4, -1, -2};
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

bool set_voltage(int left_mv, int right_mv) {
    if (pros::competition::is_disabled()) {
        stop();
        return false;
    }
    const auto left_result = _g_left.move_voltage(std::clamp(left_mv, -12000, 12000));
    const auto right_result = _g_right.move_voltage(std::clamp(right_mv, -12000, 12000));
    if (left_result == std::numeric_limits<std::int32_t>::max() ||
        right_result == std::numeric_limits<std::int32_t>::max()) {
        stop();
        return false;
    }
    return true;
}

void move() {
    auto &controller = io::prosController();
    if (!controller.isConnected() || pros::competition::is_disabled() ||
        pros::competition::is_autonomous()) {
        stop();
        return;
    }
    const int forward = -controller.analog(io::Analog::LeftY);
    const int turn = -controller.analog(io::Analog::RightX);
    const int left = forward + turn;
    const int right = forward - turn;
    const int scale = std::max({127, std::abs(left), std::abs(right)});
    // 等比归一化，避免混控饱和改变转向比例；保留原手动正负号。
    _g_left.move(left * 127 / scale);
    _g_right.move(right * 127 / scale);
}

std::array<auton::MotorReading, 4> readings() {
    std::array<auton::MotorReading, 4> result{};
    for (int i = 0; i < 4; ++i) {
        const auto &group = i < 2 ? _g_left : _g_right;
        const auto index = static_cast<std::uint8_t>(i % 2);
        auto &motor = result[i];
        motor.port = kPorts[i];
        motor.voltage_mv = group.get_voltage(index);
        motor.velocity_rpm = group.get_actual_velocity(index);
        motor.position_deg = group.get_position(index);
        motor.current_ma = group.get_current_draw(index);
        motor.valid = motor.voltage_mv != std::numeric_limits<std::int32_t>::max() &&
                      motor.current_ma >= 0 &&
                      motor.current_ma != std::numeric_limits<std::int32_t>::max() &&
                      std::isfinite(motor.velocity_rpm) && std::isfinite(motor.position_deg);
    }
    return result;
}
} // namespace chassis
