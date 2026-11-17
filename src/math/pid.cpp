#include "math/pid.hpp"

#include <algorithm>

namespace math {

Pid::Pid(const PidConfig &config) : m_config(config) {}

void Pid::reset() {
    m_integral = 0.0;
    m_previous_error = 0.0;
    m_has_previous = false;
}

double Pid::update(double error, double dt_s) {
    const double derivative = m_has_previous ? (error - m_previous_error) / dt_s : 0.0;
    const double integral =
        std::clamp(m_integral + error * dt_s, -m_config.integral_limit, m_config.integral_limit);
    const double proportional_derivative = m_config.kp * error + m_config.kd * derivative;
    const double candidate = proportional_derivative + m_config.ki * integral;
    // 允许积分向退出饱和的方向变化，避免反向纠偏时仍背负同向积分。
    if (!((candidate > m_config.output_limit && error > 0.0) ||
          (candidate < -m_config.output_limit && error < 0.0))) {
        m_integral = integral;
    }
    m_previous_error = error;
    m_has_previous = true;
    return std::clamp(candidate, -m_config.output_limit, m_config.output_limit);
}

} // namespace math
