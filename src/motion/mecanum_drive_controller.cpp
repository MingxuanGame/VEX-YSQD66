#include "motion/mecanum_drive_controller.hpp"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {
constexpr double kDefaultDtSec = 0.02;

double limitValue(double current, double target, double max_delta) {
    if (max_delta <= 0.0 || !std::isfinite(max_delta))
        return target;
    const double delta = std::clamp(target - current, -max_delta, max_delta);
    return current + delta;
}

int limitPower(int target, int previous, int max_delta) {
    if (max_delta <= 0)
        return target;
    const int delta = std::clamp(target - previous, -max_delta, max_delta);
    return std::clamp(previous + delta, -127, 127);
}
} // namespace

MecanumDriveController::MecanumDriveController(const MecanumGeometry &geometry,
                                               const MecanumDriveConfig &config)
    : m_kinematics(geometry), m_config(config) {}

void MecanumDriveController::setGeometry(const MecanumGeometry &geometry) {
    m_kinematics.setGeometry(geometry);
    reset();
}

void MecanumDriveController::setConfig(const MecanumDriveConfig &config) {
    m_config = config;
}

void MecanumDriveController::setWheelGains(const WheelVelocityGains &gains) {
    m_wheels.setGains(gains);
}

MecanumDriveResult MecanumDriveController::update(const BodyVelocity &target,
                                                  const WheelSpeeds &measured, double dt_sec) {
    m_result = MecanumDriveResult{};
    if (!std::isfinite(target.vx_ips) || !std::isfinite(target.vy_ips) ||
        !std::isfinite(target.omega_dps) || !std::isfinite(dt_sec) || dt_sec <= 0.0) {
        reset();
        return m_result;
    }

    const double dt = std::max(dt_sec, 0.001);
    BodyVelocity limited = m_last_body;
    limited.vx_ips = limitValue(limited.vx_ips, target.vx_ips, m_config.max_accel_ips2 * dt);
    limited.vy_ips = limitValue(limited.vy_ips, target.vy_ips, m_config.max_accel_ips2 * dt);
    limited.omega_dps =
        limitValue(limited.omega_dps, target.omega_dps, m_config.max_alpha_dps2 * dt);
    m_last_body = limited;
    m_result.target_body = limited;

    m_result.target_wheels = m_kinematics.inverse(limited, m_config.max_wheel_speed_ips);
    m_result.saturated = m_result.target_wheels.normalized;

    for (double value : measured.ips) {
        if (!std::isfinite(value)) {
            reset();
            return m_result;
        }
    }

    const auto target_rpm = m_kinematics.toWheelRpm(m_result.target_wheels);
    const auto measured_rpm = m_kinematics.toWheelRpm(measured);

    std::array<double, 4> current_rpm{};
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(measured_rpm[i])) {
            reset();
            return m_result;
        }
        current_rpm[i] = measured_rpm[i];
        if (m_config.enable_slip_detection &&
            std::fabs(target_rpm[i] - current_rpm[i]) > m_config.slip_rpm &&
            std::fabs(target_rpm[i]) > m_config.slip_rpm) {
            m_result.slip_detected = true;
        }
    }

    const auto raw_power = m_wheels.update(target_rpm, current_rpm, dt);
    for (int i = 0; i < 4; ++i) {
        m_result.power[i] =
            limitPower(raw_power[i], m_last_power[i], m_config.max_power_delta_per_update);
        m_last_power[i] = m_result.power[i];
    }
    m_result.valid = true;
    return m_result;
}

const MecanumDriveResult &MecanumDriveController::getResult() const {
    return m_result;
}

void MecanumDriveController::reset() {
    m_wheels.reset();
    m_result = MecanumDriveResult{};
    m_last_power.fill(0);
    m_last_body = BodyVelocity{};
}

} // namespace motion
