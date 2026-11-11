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
} // namespace

ChassisKinematics::ChassisKinematics()
    : m_pose{0.0, 0.0, 0.0}, m_last_left_position(0.0), m_last_right_position(0.0),
      m_imu_rotation_offset_deg(0.0), m_last_distance(0.0), m_last_delta_heading(0.0),
      m_initialized(false) {}

void ChassisKinematics::reset() {
    m_pose = Pose{0.0, 0.0, 0.0};
    m_last_left_position = 0.0;
    m_last_right_position = 0.0;
    m_imu_rotation_offset_deg = 0.0;
    m_last_distance = 0.0;
    m_last_delta_heading = 0.0;
    m_initialized = false;
}

void ChassisKinematics::update(robot::Chassis *chassis, robot::Imu *imu, double dt_sec) {
    (void)dt_sec;
    if (!chassis || !imu)
        return;

    const double left_ticks = chassis->getLeftPosition();
    const double right_ticks = chassis->getRightPosition();

    if (!m_initialized) {
        m_last_left_position = left_ticks;
        m_last_right_position = right_ticks;
        // 起点定义为 (0, 0, 0)；记录 IMU 初始读数作为航向零点，
        // 避免第一帧噪声直接变成绝对航向。
        const double imu_rotation_deg = imu->getRotation();
        if (vxmath::isValidAngle(imu_rotation_deg)) {
            m_imu_rotation_offset_deg = imu_rotation_deg;
        }
        m_pose.heading = 0.0;
        m_initialized = true;
        return;
    }

    const double delta_left = vxmath::ticksToInches(left_ticks - m_last_left_position);
    const double delta_right = vxmath::ticksToInches(right_ticks - m_last_right_position);
    m_last_left_position = left_ticks;
    m_last_right_position = right_ticks;

    const double delta_distance = (delta_left + delta_right) / 2.0;
    double delta_heading = (delta_right - delta_left) / cfg::kTrackWidthInch;

    const double imu_rotation_deg = imu->getRotation();
    if (vxmath::isValidAngle(imu_rotation_deg)) {
        const double relative_deg = imu_rotation_deg - m_imu_rotation_offset_deg;
        const double new_heading = relative_deg * kDegToRad;
        delta_heading = new_heading - m_pose.heading;
        m_pose.heading = new_heading;
    } else {
        m_pose.heading += delta_heading;
    }

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
