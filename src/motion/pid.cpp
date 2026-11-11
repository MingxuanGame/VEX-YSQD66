#include "motion/pid.hpp"

namespace motion {

namespace {
constexpr double kMinDtSec = 1e-4;
}

Pid::Pid()
    : m_kp(0.0), m_ki(0.0), m_kd(0.0), m_integral(0.0), m_prev_error(0.0), m_output(0.0),
      m_output_min(-127.0), m_output_max(127.0), m_integral_min(-100.0), m_integral_max(100.0),
      m_has_prev_error(false) {}

void Pid::setGains(double kp, double ki, double kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void Pid::setIntegralLimits(double min, double max) {
    m_integral_min = min;
    m_integral_max = max;
}

void Pid::setOutputLimits(double min, double max) {
    m_output_min = min;
    m_output_max = max;
}

double Pid::update(double error, double dt_sec) {
    double dt = dt_sec;
    if (dt < kMinDtSec)
        dt = kMinDtSec;

    m_integral += error * dt;
    if (m_integral > m_integral_max)
        m_integral = m_integral_max;
    if (m_integral < m_integral_min)
        m_integral = m_integral_min;

    double derivative = 0.0;
    if (m_has_prev_error) {
        derivative = (error - m_prev_error) / dt;
    }

    m_output = m_kp * error + m_ki * m_integral + m_kd * derivative;
    if (m_output > m_output_max)
        m_output = m_output_max;
    if (m_output < m_output_min)
        m_output = m_output_min;

    m_prev_error = error;
    m_has_prev_error = true;
    return m_output;
}

void Pid::reset() {
    m_integral = 0.0;
    m_prev_error = 0.0;
    m_output = 0.0;
    m_has_prev_error = false;
}

double Pid::getOutput() const {
    return m_output;
}

double Pid::getIntegral() const {
    return m_integral;
}

} // namespace motion
