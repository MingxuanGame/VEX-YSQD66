#include "motion/pid.hpp"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {
constexpr double kMinDtSec = 1e-4;
}

Pid::Pid()
    : m_kp(0.0), m_ki(0.0), m_kd(0.0), m_integral(0.0), m_prev_error(0.0), m_output(0.0),
      m_output_min(-127.0), m_output_max(127.0), m_integral_min(-100.0), m_integral_max(100.0),
      m_derivative_filter_tau(0.0), m_filtered_derivative(0.0), m_has_prev_error(false),
      m_has_filtered_derivative(false) {}

void Pid::setGains(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void Pid::setIntegralLimits(double min, double max) {
    m_integral_min = min;
    m_integral_max = max;
    m_integral = std::clamp(m_integral, m_integral_min, m_integral_max);
}

void Pid::setOutputLimits(double min, double max) {
    m_output_min = min;
    m_output_max = max;
}

void Pid::setDerivativeFilterTau(double tau_sec) {
    m_derivative_filter_tau = std::isfinite(tau_sec) && tau_sec > 0.0 ? tau_sec : 0.0;
    m_filtered_derivative = 0.0;
    m_has_filtered_derivative = false;
}

double Pid::update(double error, double dt_sec, double feedforward) {
    if (!std::isfinite(error) || !std::isfinite(feedforward)) {
        reset();
        return 0.0;
    }

    double dt = std::isfinite(dt_sec) ? dt_sec : kMinDtSec;
    if (dt < kMinDtSec)
        dt = kMinDtSec;

    const double raw_derivative = m_has_prev_error ? (error - m_prev_error) / dt : 0.0;
    if (m_derivative_filter_tau > 0.0) {
        const double alpha = dt / (m_derivative_filter_tau + dt);
        if (m_has_filtered_derivative) {
            m_filtered_derivative += alpha * (raw_derivative - m_filtered_derivative);
        } else {
            m_filtered_derivative = raw_derivative;
            m_has_filtered_derivative = true;
        }
    } else {
        m_filtered_derivative = raw_derivative;
        m_has_filtered_derivative = true;
    }

    // 先尝试积分，再用钳位后的输出反算积分，防止饱和期间继续累积。
    m_integral += error * dt;
    m_integral = std::clamp(m_integral, m_integral_min, m_integral_max);

    const double derivative = m_kd * m_filtered_derivative;
    const double unclamped = m_kp * error + m_ki * m_integral + derivative + feedforward;
    double output = std::clamp(unclamped, m_output_min, m_output_max);

    const bool saturated_high = unclamped > m_output_max && error > 0.0;
    const bool saturated_low = unclamped < m_output_min && error < 0.0;
    if (m_ki != 0.0 && (saturated_high || saturated_low)) {
        // 反算积分：让积分项刚好不再把输出推过限幅，避免反向恢复时长时间退饱和。
        const double allowed_integral = (output - m_kp * error - derivative - feedforward) / m_ki;
        m_integral = std::clamp(allowed_integral, m_integral_min, m_integral_max);
        output = std::clamp(m_kp * error + m_ki * m_integral + derivative + feedforward,
                            m_output_min, m_output_max);
    }

    m_output = output;
    m_prev_error = error;
    m_has_prev_error = true;
    return m_output;
}

void Pid::reset() {
    m_integral = 0.0;
    m_prev_error = 0.0;
    m_output = 0.0;
    m_filtered_derivative = 0.0;
    m_has_prev_error = false;
    m_has_filtered_derivative = false;
}

double Pid::getOutput() const {
    return m_output;
}

double Pid::getIntegral() const {
    return m_integral;
}

} // namespace motion
