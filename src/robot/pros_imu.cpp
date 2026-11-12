#include "robot/pros_imu.hpp"

#include <cmath>

namespace robot {

ProsImu::ProsImu(pros::Imu *imu) : m_imu(imu) {}

double ProsImu::getHeading() const {
    if (!m_imu)
        return 0.0;
    const double value = m_imu->get_heading();
    return std::isfinite(value) ? value : 0.0;
}

double ProsImu::getRotation() const {
    if (!m_imu)
        return 0.0;
    const double value = m_imu->get_rotation();
    return std::isfinite(value) ? value : 0.0;
}

double ProsImu::getGyroRateZ() const {
    if (!m_imu)
        return 0.0;
    const double value = m_imu->get_gyro_rate().z;
    return std::isfinite(value) ? value : 0.0;
}

} // namespace robot
