#include "motion/mecanum_odometry.hpp"

#include <algorithm>
#include <cmath>

namespace motion {

namespace {
constexpr double kDegToRad = 3.141592653589793 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.141592653589793;
constexpr double kDefaultDtSec = 0.02;
constexpr double kMaxYawRateDps = 2000.0;
} // namespace

MecanumOdometry::MecanumOdometry(const MecanumGeometry &geometry) : m_kinematics(geometry) {}

void MecanumOdometry::setGeometry(const MecanumGeometry &geometry) {
    m_kinematics.setGeometry(geometry);
    reset();
}

void MecanumOdometry::reset() {
    m_pose = MecanumPose{};
    m_last_body = BodyVelocity{};
    m_imu_offset_deg = 0.0;
    m_initialized = false;
    m_imu_was_valid = false;
}

void MecanumOdometry::update(const WheelSpeeds &wheels, double imu_heading_deg, bool imu_valid,
                             double dt_sec) {
    if (!(std::isfinite(dt_sec) && dt_sec > 0.0))
        dt_sec = kDefaultDtSec;

    const BodyVelocity body = m_kinematics.forward(wheels);
    if (!std::isfinite(body.vx_ips) || !std::isfinite(body.vy_ips) ||
        !std::isfinite(body.omega_dps)) {
        return;
    }

    const bool valid_imu =
        imu_valid && std::isfinite(imu_heading_deg) && std::fabs(imu_heading_deg) < 10000.0;
    if (!m_initialized) {
        m_imu_offset_deg = valid_imu ? imu_heading_deg : 0.0;
        m_pose.heading_deg = 0.0;
        m_initialized = true;
        m_imu_was_valid = valid_imu;
        m_last_body = body;
        return;
    }

    // IMU 从无效恢复时重新建立航向基准，避免累积角跳变被拒绝后永远走轮速。
    if (valid_imu && !m_imu_was_valid)
        m_imu_offset_deg = imu_heading_deg - m_pose.heading_deg;
    m_imu_was_valid = valid_imu;

    bool accepted_imu = false;
    const double previous_heading_deg = m_pose.heading_deg;
    if (valid_imu) {
        const double desired = imu_heading_deg - m_imu_offset_deg;
        const double raw_delta = desired - m_pose.heading_deg;
        const double max_delta = kMaxYawRateDps * dt_sec;
        if (std::isfinite(raw_delta) && std::fabs(raw_delta) <= max_delta) {
            m_pose.heading_deg = desired;
            accepted_imu = true;
        }
    }
    if (!accepted_imu)
        m_pose.heading_deg += body.omega_dps * dt_sec;

    const double mid_heading = (previous_heading_deg + m_pose.heading_deg) * 0.5 * kDegToRad;
    const double c = std::cos(mid_heading);
    const double s = std::sin(mid_heading);
    m_pose.x_in += (body.vx_ips * c - body.vy_ips * s) * dt_sec;
    m_pose.y_in += (body.vx_ips * s + body.vy_ips * c) * dt_sec;
    m_last_body = body;
}

const MecanumPose &MecanumOdometry::getPose() const {
    return m_pose;
}

const BodyVelocity &MecanumOdometry::getLastBodyVelocity() const {
    return m_last_body;
}

bool MecanumOdometry::isInitialized() const {
    return m_initialized;
}

} // namespace motion
