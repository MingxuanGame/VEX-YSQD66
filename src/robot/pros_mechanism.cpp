#include "robot/pros_mechanism.hpp"

#include <algorithm>
#include <cmath>

namespace robot {

ProsMechanism::ProsMechanism(const ProsMechanismConfig &config) : m_config(config) {}

void ProsMechanism::setConfig(const ProsMechanismConfig &config) {
    m_config = config;
}

void ProsMechanism::liftTo(double pos, int max_speed) {
    if (!m_config.lift_motor)
        return;
    const double current = m_config.lift_motor->get_position();
    if ((pos > current && m_config.lift_upper_limit && m_config.lift_upper_limit()) ||
        (pos < current && m_config.lift_lower_limit && m_config.lift_lower_limit())) {
        liftHold();
        return;
    }
    m_config.lift_motor->move_absolute(pos, std::clamp(max_speed, 0, 127));
}

void ProsMechanism::liftManual(int speed) {
    if (!m_config.lift_motor)
        return;
    if ((speed > 0 && m_config.lift_upper_limit && m_config.lift_upper_limit()) ||
        (speed < 0 && m_config.lift_lower_limit && m_config.lift_lower_limit())) {
        liftHold();
        return;
    }
    const int voltage_mv = std::clamp(speed, -127, 127) * 12000 / 127;
    m_config.lift_motor->move_voltage(voltage_mv);
}

void ProsMechanism::liftHold() {
    if (!m_config.lift_motor)
        return;
    // 依赖升降电机预先设置为 brake/hold；这里只撤销输出，不假装能机械锁定。
    m_config.lift_motor->move_voltage(0);
}

void ProsMechanism::clampOpen() {
    if (m_config.clamp_open) {
        m_config.clamp_open();
    } else if (m_config.clamp_set_angle) {
        m_config.clamp_set_angle(m_config.clamp_open_deg);
    }
}

void ProsMechanism::clampClose() {
    if (m_config.clamp_close) {
        m_config.clamp_close();
    } else if (m_config.clamp_set_angle) {
        m_config.clamp_set_angle(m_config.clamp_close_deg);
    }
}

void ProsMechanism::clampSetAngle(int angle) {
    if (m_config.clamp_set_angle)
        m_config.clamp_set_angle(angle);
}

void ProsMechanism::pushExtend() {
    if (m_config.push_extend)
        m_config.push_extend();
}

void ProsMechanism::pushRetract() {
    if (m_config.push_retract)
        m_config.push_retract();
}

double ProsMechanism::getCurrentMa() const {
    double value = 0.0;
    if (m_config.current_ma) {
        value = m_config.current_ma();
    } else if (m_config.lift_motor) {
        value = static_cast<double>(m_config.lift_motor->get_current_draw());
    }
    return (std::isfinite(value) && value >= 0.0) ? value : 0.0;
}

bool ProsMechanism::isHealthy() const {
    return !m_config.fault || !m_config.fault();
}

} // namespace robot
