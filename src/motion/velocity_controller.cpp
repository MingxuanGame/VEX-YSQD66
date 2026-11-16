#include "motion/velocity_controller.hpp"
#include "config/motion_config.hpp"
#include <cmath>

namespace motion {

namespace {

int clampOutput(double value) {
    int output = static_cast<int>(std::lround(value));
    if (output > cfg::kOutputLimit)
        output = cfg::kOutputLimit;
    if (output < -cfg::kOutputLimit)
        output = -cfg::kOutputLimit;
    return output;
}

} // namespace

VelocityController::VelocityController()
    : m_left_output(0), m_right_output(0), m_kp(cfg::kVelocityKp), m_ki(cfg::kVelocityKi),
      m_kd(cfg::kVelocityKd) {
    applyGains();
    // 速度反馈有齿轮振动和高频噪声，微分项先低通，避免 D 项放大毛刺。
    setDerivativeFilterTau(0.02);
}

void VelocityController::setGains(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    applyGains();
}

void VelocityController::setKp(double kp) {
    m_kp = kp;
    applyGains();
}

void VelocityController::setKi(double ki) {
    m_ki = ki;
    applyGains();
}

void VelocityController::setKd(double kd) {
    m_kd = kd;
    applyGains();
}

void VelocityController::setDerivativeFilterTau(double tau_sec) {
    m_left_pid.setDerivativeFilterTau(tau_sec);
    m_right_pid.setDerivativeFilterTau(tau_sec);
}

void VelocityController::applyGains() {
    m_left_pid.setGains(m_kp, m_ki, m_kd);
    m_right_pid.setGains(m_kp, m_ki, m_kd);

    m_left_pid.setIntegralLimits(-cfg::kIntegralLimit, cfg::kIntegralLimit);
    m_right_pid.setIntegralLimits(-cfg::kIntegralLimit, cfg::kIntegralLimit);

    m_left_pid.setOutputLimits(-cfg::kOutputLimit, cfg::kOutputLimit);
    m_right_pid.setOutputLimits(-cfg::kOutputLimit, cfg::kOutputLimit);
}

void VelocityController::update(double target_rpm_left, double target_rpm_right,
                                double current_rpm_left, double current_rpm_right, double dt_sec) {
    if (!std::isfinite(target_rpm_left) || !std::isfinite(target_rpm_right) ||
        !std::isfinite(current_rpm_left) || !std::isfinite(current_rpm_right)) {
        reset();
        return;
    }

    const double error_left = target_rpm_left - current_rpm_left;
    const double error_right = target_rpm_right - current_rpm_right;

    m_left_output = clampOutput(m_left_pid.update(error_left, dt_sec));
    m_right_output = clampOutput(m_right_pid.update(error_right, dt_sec));
}

int VelocityController::getLeftOutput() const {
    return m_left_output;
}

int VelocityController::getRightOutput() const {
    return m_right_output;
}

void VelocityController::reset() {
    m_left_pid.reset();
    m_right_pid.reset();
    m_left_output = 0;
    m_right_output = 0;
}

} // namespace motion
