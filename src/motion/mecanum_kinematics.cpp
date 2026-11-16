#include "motion/mecanum_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {
constexpr double kPi = 3.141592653589793;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

bool finiteGeometry(const MecanumGeometry &geometry) {
    return std::isfinite(geometry.wheelbase_in) && geometry.wheelbase_in > 0.0 &&
           std::isfinite(geometry.track_width_in) && geometry.track_width_in > 0.0 &&
           std::isfinite(geometry.wheel_radius_in) && geometry.wheel_radius_in > 0.0 &&
           std::isfinite(geometry.gear_ratio) && geometry.gear_ratio > 0.0;
}
} // namespace

MecanumKinematics::MecanumKinematics(const MecanumGeometry &geometry) : m_geometry(geometry) {}

void MecanumKinematics::setGeometry(const MecanumGeometry &geometry) {
    if (finiteGeometry(geometry))
        m_geometry = geometry;
}

const MecanumGeometry &MecanumKinematics::getGeometry() const {
    return m_geometry;
}

double MecanumKinematics::rotationFactorIn() const {
    return (m_geometry.wheelbase_in + m_geometry.track_width_in) / 2.0;
}

WheelSpeeds MecanumKinematics::inverse(const BodyVelocity &body, double max_wheel_speed_ips) const {
    WheelSpeeds result{};
    if (!std::isfinite(body.vx_ips) || !std::isfinite(body.vy_ips) ||
        !std::isfinite(body.omega_dps)) {
        return result;
    }

    const double rotation_ips = rotationFactorIn() * body.omega_dps * kDegToRad;
    std::array<double, 4> speed{
        body.vx_ips - body.vy_ips - rotation_ips,
        body.vx_ips + body.vy_ips + rotation_ips,
        body.vx_ips + body.vy_ips - rotation_ips,
        body.vx_ips - body.vy_ips + rotation_ips,
    };

    for (int i = 0; i < 4; ++i) {
        speed[i] *= (m_geometry.wheel_direction[i] < 0) ? -1.0 : 1.0;
    }

    double peak = 0.0;
    for (double value : speed)
        peak = std::max(peak, std::fabs(value));

    result.scale = 1.0;
    if (max_wheel_speed_ips > 0.0 && peak > max_wheel_speed_ips) {
        result.scale = max_wheel_speed_ips / peak;
        result.normalized = true;
        for (double &value : speed)
            value *= result.scale;
    }

    result.ips = speed;
    return result;
}

BodyVelocity MecanumKinematics::forward(const WheelSpeeds &wheels) const {
    BodyVelocity result{};
    std::array<double, 4> speed{};
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(wheels.ips[i]))
            return result;
        speed[i] = wheels.ips[i] * ((m_geometry.wheel_direction[i] < 0) ? -1.0 : 1.0);
    }

    result.vx_ips = (speed[0] + speed[1] + speed[2] + speed[3]) / 4.0;
    result.vy_ips = (-speed[0] + speed[1] + speed[2] - speed[3]) / 4.0;
    const double omega_rad_s =
        (-speed[0] + speed[1] - speed[2] + speed[3]) / (4.0 * rotationFactorIn());
    result.omega_dps = omega_rad_s * kRadToDeg;
    return result;
}

std::array<double, 4> MecanumKinematics::toWheelRpm(const WheelSpeeds &wheels) const {
    std::array<double, 4> rpm{};
    const double wheel_circumference_in = 2.0 * kPi * m_geometry.wheel_radius_in;
    if (!(wheel_circumference_in > 0.0))
        return rpm;
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(wheels.ips[i]))
            return {};
        rpm[i] = wheels.ips[i] * 60.0 / wheel_circumference_in * m_geometry.gear_ratio;
    }
    return rpm;
}

WheelSpeeds MecanumKinematics::fromWheelRpm(const std::array<double, 4> &rpm) const {
    WheelSpeeds result{};
    const double wheel_circumference_in = 2.0 * kPi * m_geometry.wheel_radius_in;
    if (!(wheel_circumference_in > 0.0))
        return result;
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(rpm[i]))
            return result;
        result.ips[i] = rpm[i] / m_geometry.gear_ratio * wheel_circumference_in / 60.0;
    }
    return result;
}

} // namespace motion
