#pragma once

#include "motion/mecanum_kinematics.hpp"
#include "motion/wheel_velocity_controller.hpp"

#include <array>

namespace motion {

/**
 * @file mecanum_drive_controller.hpp
 * @brief 麦轮车体速度 -> 四轮目标速度 -> 四轮速度环的纯控制管线。
 *
 * 该类不访问底盘硬件，也不假设端口；后续四轮底盘接口就绪后，
 * 由上层把输出写入四个电机即可。当前主循环未调用它，因此不增加
 * 现有路径的运行时开销。
 */
struct MecanumDriveConfig {
    double max_wheel_speed_ips = 100.0;  // 轮端限速
    double max_accel_ips2 = 200.0;       // 平移加速度限制，<=0 关闭
    double max_alpha_dps2 = 720.0;       // 角加速度限制，<=0 关闭
    int max_power_delta_per_update = 20; // 功率斜率限制，<=0 关闭
    bool enable_slip_detection = true;
    double slip_rpm = 60.0; // 目标/实际轮速差阈值
};

struct MecanumDriveResult {
    BodyVelocity target_body{};
    WheelSpeeds target_wheels{};
    std::array<int, 4> power{};
    bool saturated = false;
    bool slip_detected = false;
    bool valid = false;
};

class MecanumDriveController {
public:
    explicit MecanumDriveController(const MecanumGeometry &geometry = {},
                                    const MecanumDriveConfig &config = {});

    void setGeometry(const MecanumGeometry &geometry);
    void setConfig(const MecanumDriveConfig &config);
    void setWheelGains(const WheelVelocityGains &gains);

    MecanumDriveResult update(const BodyVelocity &target, const WheelSpeeds &measured,
                              double dt_sec);

    const MecanumDriveResult &getResult() const;
    void reset();

private:
    MecanumKinematics m_kinematics;
    WheelVelocityController m_wheels;
    MecanumDriveConfig m_config;
    MecanumDriveResult m_result;
    std::array<int, 4> m_last_power{};
    BodyVelocity m_last_body{};
};

} // namespace motion
