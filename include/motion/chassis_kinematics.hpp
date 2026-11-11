#pragma once

/**
 * @file chassis_kinematics.hpp
 * @brief 底盘里程计：编码器差分 + IMU 航向融合，更新 Pose。
 *        heading 单位为弧度；x/y 单位为英寸。
 */

namespace robot {
class Chassis;
class Imu;
} // namespace robot

namespace motion {

struct Pose {
    double x;
    double y;
    double heading;
};

class ChassisKinematics {
public:
    ChassisKinematics();

    void reset();
    void update(robot::Chassis *chassis, robot::Imu *imu, double dt_sec);

    const Pose &getPose() const;
    double getLastDistanceInches() const;
    double getLastDeltaHeadingRad() const;
    bool isInitialized() const;

private:
    Pose m_pose;
    double m_last_left_position;
    double m_last_right_position;
    double m_imu_rotation_offset_deg;
    double m_last_distance;
    double m_last_delta_heading;
    bool m_initialized;
};

} // namespace motion
