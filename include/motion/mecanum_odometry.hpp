#pragma once

#include "motion/mecanum_kinematics.hpp"

namespace motion {

/**
 * @file mecanum_odometry.hpp
 * @brief 四轮麦轮里程计：四轮正解 + IMU 航向优先，IMU 无效时回退轮速航向。
 *
 * 纯数学模块，不访问硬件。坐标：x 前进、y 左移、heading 逆时针为正（度）。
 */
struct MecanumPose {
    double x_in = 0.0;
    double y_in = 0.0;
    double heading_deg = 0.0;
};

class MecanumOdometry {
public:
    explicit MecanumOdometry(const MecanumGeometry &geometry = {});

    void setGeometry(const MecanumGeometry &geometry);
    void reset();

    /**
     * @param wheels 四轮轮端线速度，单位 inch/s
     * @param imu_heading_deg IMU 累积航向
     * @param imu_valid IMU 是否健康且角度有效
     * @param dt_sec 更新周期
     */
    void update(const WheelSpeeds &wheels, double imu_heading_deg, bool imu_valid, double dt_sec);

    const MecanumPose &getPose() const;
    const BodyVelocity &getLastBodyVelocity() const;
    bool isInitialized() const;

private:
    MecanumKinematics m_kinematics;
    MecanumPose m_pose;
    BodyVelocity m_last_body{};
    double m_imu_offset_deg = 0.0;
    bool m_initialized = false;
    bool m_imu_was_valid = false;
};

} // namespace motion
