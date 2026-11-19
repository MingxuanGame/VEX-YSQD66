#include "chassis.h"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <utility>

namespace chassis {
namespace {

enum class ImuFailure { None, Reset, Timeout, CalibError, Tare };

// 功率偏移量：正值补偿右侧，负值补偿左侧。
constexpr int kRightMotorOffsetToLeft = 0;
constexpr std::uint8_t kImuPort = 5;
constexpr std::uint32_t kImuCalibrationTimeoutMs = 3000;
constexpr std::uint32_t kImuCalibrationPollMs = 10;
// VEXos 状态位 4 表示已自动校准，PROS 4.2.3 的 get_status() 原样返回此位。
constexpr int kImuAutoCalibrated = 1 << 4;

// 面向车头：左前 2、左后 1，右前 3、右后 4；蓝齿轮，编码器单位为电机轴角度。
pros::MotorGroup _g_left({2, 1}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::MotorGroup _g_right({-3, -4}, pros::MotorGears::blue, pros::MotorUnits::degrees);
pros::Imu _g_imu(kImuPort);
bool _g_imu_calibrated = false;
ImuFailure _g_imu_failure = ImuFailure::None;
int _g_imu_failure_errno = 0;
int _g_imu_reset_ret = 0;
std::uint32_t _g_imu_calib_ms = 0;
int _g_left_tare_errno = 0;
int _g_right_tare_errno = 0;
SensorDiagnostics _g_diagnostics{};

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

bool imu_is_calibrating(pros::ImuStatus status) {
    // 与 PROS 4.2.3 内核一致，校准只看最低位，其他位包含安装方向等信息。
    return (static_cast<int>(status) & pros::E_IMU_STATUS_CALIBRATING) != 0;
}

bool imu_auto_calibrated(pros::ImuStatus status) {
    return status != pros::ImuStatus::error && !imu_is_calibrating(status) &&
           (static_cast<int>(status) & kImuAutoCalibrated) != 0;
}

const char *imu_status_label(pros::ImuStatus status) {
    if (status == pros::ImuStatus::error) {
        return "ERR";
    }
    if (imu_is_calibrating(status)) {
        return "CAL";
    }
    return imu_auto_calibrated(status) ? "AUTO" : "IDLE";
}

const char *imu_failure_stage(ImuFailure failure) {
    switch (failure) {
    case ImuFailure::Reset:
        return "IMU_RESET";
    case ImuFailure::Timeout:
        return "IMU_TIMEOUT";
    case ImuFailure::CalibError:
        return "IMU_CALIB";
    case ImuFailure::Tare:
        return "IMU_TARE";
    case ImuFailure::None:
        return "IMU_NOT_RDY";
    }
    return "IMU_NOT_RDY";
}

void capture_failure_details(pros::ImuStatus status) {
    _g_diagnostics = {};
    _g_diagnostics.has_error = true;
    _g_diagnostics.imu_calibrated = _g_imu_calibrated;
    _g_diagnostics.imu_reset_ret = _g_imu_reset_ret;
    _g_diagnostics.calib_ms = _g_imu_calib_ms;
    _g_diagnostics.left_tare_errno = _g_left_tare_errno;
    _g_diagnostics.right_tare_errno = _g_right_tare_errno;

    _g_diagnostics.imu_errno = status == pros::ImuStatus::error ? errno : 0;
    _g_diagnostics.imu_status = static_cast<int>(status);
    _g_diagnostics.imu_status_label = imu_status_label(status);

    constexpr std::array<std::pair<pros::MotorGroup *, std::uint8_t>, 4> kMotors{{
        {&_g_left, 0},
        {&_g_left, 1},
        {&_g_right, 0},
        {&_g_right, 1},
    }};
    bool any_motor_nonfinite = false;
    for (std::size_t i = 0; i < kMotors.size(); ++i) {
        errno = 0;
        const double pos = kMotors[i].first->get_position(kMotors[i].second);
        _g_diagnostics.motor_position_deg[i] = pos;
        if (!std::isfinite(pos)) {
            _g_diagnostics.motor_errno[i] = errno;
            any_motor_nonfinite = true;
        }
    }

    errno = 0;
    _g_diagnostics.imu_rotation_deg = _g_imu.get_rotation();
    const bool imu_nonfinite = !std::isfinite(_g_diagnostics.imu_rotation_deg);
    if (imu_nonfinite && _g_diagnostics.imu_errno == 0) {
        _g_diagnostics.imu_errno = errno;
    }

    if (!_g_imu_calibrated) {
        if (_g_imu_failure_errno != 0) {
            _g_diagnostics.imu_errno = _g_imu_failure_errno;
        }
        _g_diagnostics.stage = imu_failure_stage(_g_imu_failure);
    } else if (status == pros::ImuStatus::error || imu_is_calibrating(status)) {
        _g_diagnostics.stage = any_motor_nonfinite ? "IMU+ENC" : "IMU_STAT";
    } else if (_g_left_tare_errno != 0 || _g_right_tare_errno != 0) {
        _g_diagnostics.stage = "ENC_TARE";
    } else if (imu_nonfinite && any_motor_nonfinite) {
        _g_diagnostics.stage = "IMU+ENC_RD";
    } else if (imu_nonfinite) {
        _g_diagnostics.stage = "IMU_READ";
    } else if (any_motor_nonfinite) {
        _g_diagnostics.stage = "ENC_READ";
    } else {
        _g_diagnostics.stage = "UNKNOWN";
    }
}

void apply_power(int left_power, int right_power) {
    const int adjusted_left = apply_offset(left_power, std::max(0, -kRightMotorOffsetToLeft));
    const int adjusted_right = apply_offset(right_power, std::max(0, kRightMotorOffsetToLeft));
    _g_left.move(adjusted_left);
    _g_right.move(adjusted_right);
}

} // namespace

void initialize() {
    stop();
    _g_left.set_brake_mode_all(pros::MotorBrake::brake);
    _g_right.set_brake_mode_all(pros::MotorBrake::brake);
    _g_imu_calibrated = false;
    _g_imu_failure = ImuFailure::None;
    _g_imu_failure_errno = 0;
    _g_imu_reset_ret = 0;
    _g_imu_calib_ms = 0;
    _g_left_tare_errno = 0;
    _g_right_tare_errno = 0;
    _g_diagnostics = {};

    const auto start_ms = pros::millis();
    auto status = _g_imu.get_status();
    bool calibration_started = status != pros::ImuStatus::error && imu_is_calibrating(status);
    int reset_errno = 0;
    if (status != pros::ImuStatus::error && !calibration_started && !imu_auto_calibrated(status)) {
        _g_imu_reset_ret = _g_imu.reset();
        calibration_started = _g_imu_reset_ret == 1;
        reset_errno = calibration_started ? 0 : errno;
        status = _g_imu.get_status();
        if (!calibration_started && reset_errno != EAGAIN) {
            _g_imu_failure = ImuFailure::Reset;
            _g_imu_failure_errno = reset_errno;
            capture_failure_details(status);
            return;
        }
    }

    // EAGAIN 也可能是 reset() 等待校准标志超时。只有观察到校准结束或自动校准完成才放行。
    while (status != pros::ImuStatus::error &&
           pros::millis() - start_ms < kImuCalibrationTimeoutMs) {
        if (imu_is_calibrating(status)) {
            calibration_started = true;
        } else if (calibration_started || imu_auto_calibrated(status)) {
            _g_imu_calibrated = true;
            return;
        }
        pros::delay(kImuCalibrationPollMs);
        status = _g_imu.get_status();
    }
    _g_imu_failure =
        status == pros::ImuStatus::error ? ImuFailure::CalibError : ImuFailure::Timeout;
    _g_imu_failure_errno = status == pros::ImuStatus::error ? errno : reset_errno;
    _g_imu_calib_ms = pros::millis() - start_ms;
    capture_failure_details(status);
}

void stop() {
    _g_left.brake();
    _g_right.brake();
}

void reset_state() {
    stop();
    _g_left.set_brake_mode_all(pros::MotorBrake::brake);
    _g_right.set_brake_mode_all(pros::MotorBrake::brake);
    errno = 0;
    _g_left_tare_errno = (_g_left.tare_position_all() == 1) ? 0 : errno;
    errno = 0;
    _g_right_tare_errno = (_g_right.tare_position_all() == 1) ? 0 : errno;
    // 自动校准可能晚于上电等待窗口。只在停车重建参考时接纳其完成状态。
    if (_g_imu_failure == ImuFailure::Timeout && imu_auto_calibrated(_g_imu.get_status())) {
        _g_imu_calibrated = true;
        _g_imu_failure = ImuFailure::None;
        _g_imu_failure_errno = 0;
        _g_imu_calib_ms = 0;
    }
    if (_g_imu_calibrated) {
        errno = 0;
        if (_g_imu.tare_rotation() != 1) {
            _g_imu_calibrated = false;
            _g_imu_failure = ImuFailure::Tare;
            _g_imu_failure_errno = errno;
        }
    }
}

bool read_feedback(std::array<double, 4> &motor_position_deg, double &imu_rotation_deg) {
    const auto status = _g_imu.get_status();
    if (!_g_imu_calibrated || status == pros::ImuStatus::error || imu_is_calibrating(status) ||
        _g_left_tare_errno != 0 || _g_right_tare_errno != 0) {
        capture_failure_details(status);
        return false;
    }
    motor_position_deg = {_g_left.get_position(0), _g_left.get_position(1),
                          _g_right.get_position(0), _g_right.get_position(1)};
    imu_rotation_deg = _g_imu.get_rotation();
    const bool all_finite =
        std::isfinite(imu_rotation_deg) &&
        std::all_of(motor_position_deg.begin(), motor_position_deg.end(),
                    [](double position_deg) { return std::isfinite(position_deg); });
    if (!all_finite) {
        capture_failure_details(status);
        return false;
    }
    _g_diagnostics = {};
    _g_diagnostics.imu_calibrated = true;
    _g_diagnostics.imu_status = static_cast<int>(status);
    _g_diagnostics.imu_status_label = imu_status_label(status);
    _g_diagnostics.imu_reset_ret = _g_imu_reset_ret;
    _g_diagnostics.imu_rotation_deg = imu_rotation_deg;
    _g_diagnostics.motor_position_deg = motor_position_deg;
    return true;
}

const SensorDiagnostics &sensor_diagnostics() {
    return _g_diagnostics;
}

void set_power(int left_power, int right_power) {
    if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
        stop();
        return;
    }
    apply_power(left_power, right_power);
}

void set_autonomous_power(int left_power, int right_power) {
    if (pros::competition::is_disabled() || !pros::competition::is_autonomous()) {
        stop();
        return;
    }
    apply_power(left_power, right_power);
}

} // namespace chassis
