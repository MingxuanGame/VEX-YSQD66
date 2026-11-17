#include "chassis.h"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace chassis {
namespace {

// 功率偏移量：正值补偿右侧，负值补偿左侧。
constexpr int kRightMotorOffsetToLeft = 0;
constexpr std::uint8_t kImuPort = 5;
constexpr std::uint32_t kImuCalibrationTimeoutMs = 3000;
constexpr std::uint32_t kImuCalibrationPollMs = 10;

// 面向车头：左前 2、左后 1，右前 3、右后 4；蓝齿轮，编码器单位为电机轴角度。
pros::MotorGroup _g_left({2, 1}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup _g_right({-3, -4}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Imu _g_imu(kImuPort);
bool _g_imu_calibrated = false;

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
    stop();
    _g_left.set_brake_mode_all(pros::MotorBrake::brake);
    _g_right.set_brake_mode_all(pros::MotorBrake::brake);
    _g_imu_calibrated = false;
    const auto start_ms = pros::millis();
    if (_g_imu.reset() != 1) {
        return;
    }
    while (_g_imu.get_status() == pros::ImuStatus::calibrating &&
           pros::millis() - start_ms < kImuCalibrationTimeoutMs) {
        pros::delay(kImuCalibrationPollMs);
    }
    _g_imu_calibrated = pros::millis() - start_ms < kImuCalibrationTimeoutMs &&
                        _g_imu.get_status() == pros::ImuStatus::ready;
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
    if (_g_imu_calibrated && _g_imu.tare_rotation() != 1) {
        _g_imu_calibrated = false;
    }
}

bool read_feedback(std::array<double, 4> &motor_position_deg, double &imu_rotation_deg) {
    if (!_g_imu_calibrated || _g_imu.get_status() != pros::ImuStatus::ready) {
        return false;
    }
    motor_position_deg = {_g_left.get_position(0), _g_left.get_position(1),
                          _g_right.get_position(0), _g_right.get_position(1)};
    imu_rotation_deg = _g_imu.get_rotation();
    return std::isfinite(imu_rotation_deg) &&
           std::all_of(motor_position_deg.begin(), motor_position_deg.end(),
                       [](double position_deg) { return std::isfinite(position_deg); });
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
