#include "robot/pros_mechanism.hpp"

#include <algorithm>

namespace robot {

ProsMechanism::ProsMechanism(const ProsMechanismConfig &config) : m_config(config) {}

void ProsMechanism::setConfig(const ProsMechanismConfig &config) {
    m_config = config;
}

void ProsMechanism::liftTo(double pos, int max_speed) {
    if (!m_config.lift_motor)
        return;
    m_config.lift_motor->move_absolute(pos, std::clamp(max_speed, 0, 127));
}

void ProsMechanism::liftManual(int speed) {
    if (!m_config.lift_motor)
        return;
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
    if (m_config.current_ma)
        return m_config.current_ma();
    if (m_config.lift_motor)
        return static_cast<double>(m_config.lift_motor->get_current_draw());
    return 0.0;
}

} // namespace robot
