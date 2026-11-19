#pragma once

#include "autonomous/route.hpp"

#include <cstdint>

namespace auto_route {

// 使用四轮编码器的纵向平均和 IMU 航向进行差速里程积分。
class Odometry {
public:
    Odometry(Pose initial, double travel_in_per_motor_rev);
    bool begin();
    bool update();
    Pose pose() const;
    double speed_in_s() const;
    double dt_s() const;

private:
    bool read(double &drive_in, double &imu_deg) const;

    Pose m_pose;
    double m_in_per_motor_deg;
    double m_imu_origin_deg = 0.0;
    double m_initial_heading_deg;
    double m_last_drive_in = 0.0;
    double m_speed_in_s = 0.0;
    double m_dt_s = 0.02;
    std::uint32_t m_last_ms = 0;
};

} // namespace auto_route
