#include "robot/robot.hpp"

namespace robot {

Robot::Robot(Chassis *chassis, Mechanism *mechanism) : m_chassis(chassis), m_mechanism(mechanism) {}

Chassis *Robot::getChassis() {
    return m_chassis;
}

const Chassis *Robot::getChassis() const {
    return m_chassis;
}

Mechanism *Robot::getMechanism() {
    return m_mechanism;
}

const Mechanism *Robot::getMechanism() const {
    return m_mechanism;
}

bool Robot::isValid() const {
    return m_chassis != nullptr && m_mechanism != nullptr;
}

} // namespace robot
