#include "autonomous/odometry.hpp"

#include "chassis.h"
#include "pros/rtos.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <numeric>

namespace auto_route {

Odometry::Odometry(Pose initial, double travel_in_per_motor_rev)
    : m_pose(initial), m_in_per_motor_deg(travel_in_per_motor_rev / 360.0),
      m_initial_heading_deg(initial.heading_deg) {}

bool Odometry::begin() {
    m_last_ms = pros::millis();
    return read(m_last_drive_in, m_imu_origin_deg);
}

bool Odometry::update() {
    double drive_in;
    double imu_deg;
    if (!read(drive_in, imu_deg)) {
        return false;
    }
    const auto now_ms = pros::millis();
    const auto elapsed_ms = now_ms - m_last_ms;
    if (elapsed_ms == 0) {
        return true;
    }
    const double delta_in = drive_in - m_last_drive_in;
    const double heading_deg = m_initial_heading_deg + imu_deg - m_imu_origin_deg;
    m_dt_s = elapsed_ms / 1000.0;
    const double delta_rad = (heading_deg - m_pose.heading_deg) * std::numbers::pi / 180.0;
    // 传感器断连、编码器重置或异常轮跳不能成为闭环目标。
    if (elapsed_ms > 150 || std::abs(delta_in) > 100.0 * m_dt_s + 0.5 ||
        std::abs(delta_rad) > 24.0 * m_dt_s + 0.25) {
        return false;
    }
    const double arc_factor =
        std::abs(delta_rad) < 1e-5 ? 1.0 : std::sin(delta_rad / 2.0) / (delta_rad / 2.0);
    const double mid_rad = m_pose.heading_deg * std::numbers::pi / 180.0 + delta_rad / 2.0;
    m_pose.position.x += delta_in * arc_factor * std::sin(mid_rad);
    m_pose.position.y += delta_in * arc_factor * std::cos(mid_rad);
    m_pose.heading_deg = heading_deg;
    m_speed_in_s = std::lerp(m_speed_in_s, delta_in / m_dt_s, 0.55);
    m_last_drive_in = drive_in;
    m_last_ms = now_ms;
    return true;
}

Pose Odometry::pose() const {
    return m_pose;
}

double Odometry::speed_in_s() const {
    return m_speed_in_s;
}

double Odometry::dt_s() const {
    return m_dt_s;
}

bool Odometry::read(double &drive_in, double &imu_deg) const {
    std::array<double, 4> motor_deg;
    if (!chassis::read_feedback(motor_deg, imu_deg)) {
        return false;
    }
    drive_in = std::accumulate(motor_deg.begin(), motor_deg.end(), 0.0) * m_in_per_motor_deg / 4.0;
    return true;
}

} // namespace auto_route
