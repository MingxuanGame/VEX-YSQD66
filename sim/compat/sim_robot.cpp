#include "sim/compat/sim_robot.hpp"
#include "sim/hardware/sim_battery.hpp"
#include "sim/hardware/sim_imu.hpp"
#include "sim/hardware/sim_motor.hpp"

namespace sim {

SimChassis::SimChassis(Motor *lf, Motor *lb, Motor *rf, Motor *rb)
    : m_lf(lf), m_lb(lb), m_rf(rf), m_rb(rb) {}

void SimChassis::setDrive(int left, int right) {
    if (!m_lf || !m_lb || !m_rf || !m_rb)
        return;
    m_lf->move(left);
    m_lb->move(left);
    m_rf->move(right);
    m_rb->move(right);
}

void SimChassis::stop(bool brake) {
    if (!m_lf || !m_lb || !m_rf || !m_rb)
        return;
    const BrakeMode mode = brake ? BrakeMode::Brake : BrakeMode::Coast;
    m_lf->set_brake_mode(mode);
    m_lb->set_brake_mode(mode);
    m_rf->set_brake_mode(mode);
    m_rb->set_brake_mode(mode);
    m_lf->move(0);
    m_lb->move(0);
    m_rf->move(0);
    m_rb->move(0);
}

double SimChassis::getLeftCurrent() const {
    if (!m_lf || !m_lb)
        return 0.0;
    return (m_lf->get_current_draw() + m_lb->get_current_draw()) / 2.0;
}

double SimChassis::getRightCurrent() const {
    if (!m_rf || !m_rb)
        return 0.0;
    return (m_rf->get_current_draw() + m_rb->get_current_draw()) / 2.0;
}

double SimChassis::getLeftVelocity() const {
    if (!m_lf || !m_lb)
        return 0.0;
    return (m_lf->get_actual_velocity() + m_lb->get_actual_velocity()) / 2.0;
}

double SimChassis::getRightVelocity() const {
    if (!m_rf || !m_rb)
        return 0.0;
    return (m_rf->get_actual_velocity() + m_rb->get_actual_velocity()) / 2.0;
}

double SimChassis::getLeftPosition() const {
    if (!m_lf || !m_lb)
        return 0.0;
    return (m_lf->get_position() + m_lb->get_position()) / 2.0;
}

double SimChassis::getRightPosition() const {
    if (!m_rf || !m_rb)
        return 0.0;
    return (m_rf->get_position() + m_rb->get_position()) / 2.0;
}

void SimChassis::tarePosition() {
    if (!m_lf || !m_lb || !m_rf || !m_rb)
        return;
    m_lf->tare_position();
    m_lb->tare_position();
    m_rf->tare_position();
    m_rb->tare_position();
}

SimImu::SimImu(sim::Imu *imu) : m_imu(imu) {}

double SimImu::getHeading() const {
    return m_imu ? m_imu->get_heading() : 0.0;
}

double SimImu::getRotation() const {
    return m_imu ? m_imu->get_rotation() : 0.0;
}

double SimImu::getGyroRateZ() const {
    return m_imu ? m_imu->get_gyro_rate('z') : 0.0;
}

int SimBattery::getVoltageMv() const {
    return sim::Battery::getVoltage();
}

} // namespace sim
