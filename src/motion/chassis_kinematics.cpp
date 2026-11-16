#include "motion/chassis_kinematics.hpp"
#include "config/motion_config.hpp"
#include "robot/chassis.hpp"
#include "robot/imu.hpp"
#include "vxmath/math_utils.hpp"
#include "vxmath/units.hpp"
#include <cmath>

namespace motion {

namespace {
constexpr double kDegToRad = 3.141592653589793 / 180.0;
constexpr double kMaxYawRateDegPerSec = 2000.0;
constexpr double kDefaultDtSec = 0.01;
} // namespace

ChassisKinematics::ChassisKinematics()
    : m_pose{0.0, 0.0, 0.0}, m_last_left_position(0.0), m_last_right_position(0.0),
      m_imu_rotation_offset_deg(0.0), m_last_imu_rotation_deg(0.0), m_last_distance(0.0),
      m_last_delta_heading(0.0), m_initialized(false), m_last_imu_valid(false) {}

void ChassisKinematics::reset() {
    m_pose = Pose{0.0, 0.0, 0.0};
    m_last_left_position = 0.0;
    m_last_right_position = 0.0;
    m_imu_rotation_offset_deg = 0.0;
    m_last_imu_rotation_deg = 0.0;
    m_last_distance = 0.0;
    m_last_delta_heading = 0.0;
    m_initialized = false;
    m_last_imu_valid = false;
}

void ChassisKinematics::update(robot::Chassis *chassis, robot::Imu *imu, double dt_sec) {
    if (!chassis || !imu)
        return;

    const double left_ticks = chassis->getLeftPosition();
    const double right_ticks = chassis->getRightPosition();
    if (!vxmath::isValidDouble(left_ticks) || !vxmath::isValidDouble(right_ticks))
        return;

    const double imu_rotation_deg = imu->getRotation();
    const bool imu_valid = imu->isHealthy() && vxmath::isValidAngle(imu_rotation_deg);
    if (!m_initialized) {
        m_last_left_position = left_ticks;
        m_last_right_position = right_ticks;
        m_imu_rotation_offset_deg = imu_valid ? imu_rotation_deg : 0.0;
        m_last_imu_rotation_deg = imu_valid ? imu_rotation_deg : 0.0;
        m_pose.heading = 0.0;
        m_initialized = true;
        m_last_imu_valid = imu_valid;
        return;
    }

    const double delta_left = vxmath::ticksToInches(left_ticks - m_last_left_position);
    const double delta_right = vxmath::ticksToInches(right_ticks - m_last_right_position);
    if (!vxmath::isValidDouble(delta_left) || !vxmath::isValidDouble(delta_right))
        return;
    m_last_left_position = left_ticks;
    m_last_right_position = right_ticks;

    const double delta_distance = (delta_left + delta_right) / 2.0;
    double delta_heading = (delta_right - delta_left) / cfg::kTrackWidthInch;

    const double dt = (vxmath::isValidDouble(dt_sec) && dt_sec > 0.0) ? dt_sec : kDefaultDtSec;
    bool accepted_imu = false;
    if (imu_valid) {
        const double relative_deg = imu_rotation_deg - m_imu_rotation_offset_deg;
        const double new_heading = relative_deg * kDegToRad;
        const double raw_delta = new_heading - m_pose.heading;
        const double max_delta = kMaxYawRateDegPerSec * kDegToRad * dt;
        if (vxmath::isValidDouble(raw_delta) && std::fabs(raw_delta) <= max_delta) {
            delta_heading = raw_delta;
            m_pose.heading = new_heading;
            m_last_imu_rotation_deg = imu_rotation_deg;
            accepted_imu = true;
        }
    }

    if (!accepted_imu) {
        // IMU 掉线/跳变时使用编码器差分，避免把 0° 或尖峰当成真实航向。
        m_pose.heading += delta_heading;
        if (imu_valid)
            m_last_imu_rotation_deg = imu_rotation_deg;
    }
    m_last_imu_valid = accepted_imu;

    const double mid_heading = m_pose.heading - delta_heading * 0.5;
    m_pose.x += delta_distance * std::cos(mid_heading);
    m_pose.y += delta_distance * std::sin(mid_heading);

    m_last_distance = delta_distance;
    m_last_delta_heading = delta_heading;
}

const Pose &ChassisKinematics::getPose() const {
    return m_pose;
}

double ChassisKinematics::getLastDistanceInches() const {
    return m_last_distance;
}

double ChassisKinematics::getLastDeltaHeadingRad() const {
    return m_last_delta_heading;
}

bool ChassisKinematics::isInitialized() const {
    return m_initialized;
}

} // namespace motion
