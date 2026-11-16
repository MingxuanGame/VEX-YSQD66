#pragma once

#include <array>

namespace motion {

/**
 * @file mecanum_kinematics.hpp
 * @brief 四轮麦轮逆解/正解，不做硬件访问。
 *
 * 坐标约定：
 *   +vx 前进
 *   +vy 左移
 *   +omega 逆时针（俯视）
 * 轮序固定为 FL / FR / BL / BR。
 *
 * 角度使用 inch、inch/s、deg/s，便于和现有 motion 配置保持一致；
 * 电机 RPM 换算单独提供，避免把减速比和轮径混进运动学。
 */
struct MecanumGeometry {
    double wheelbase_in = 12.0;   // 前后轮中心距
    double track_width_in = 12.0; // 左右轮中心距
    double wheel_radius_in = 1.625;
    double gear_ratio = 1.0; // 电机转数 / 轮端转数
    std::array<int, 4> wheel_direction{1, 1, 1, 1};
};

struct BodyVelocity {
    double vx_ips = 0.0;
    double vy_ips = 0.0;
    double omega_dps = 0.0;
};

struct WheelSpeeds {
    std::array<double, 4> ips{}; // FL, FR, BL, BR
    double scale = 1.0;
    bool normalized = false;
};

class MecanumKinematics {
public:
    explicit MecanumKinematics(const MecanumGeometry &geometry = {});

    void setGeometry(const MecanumGeometry &geometry);
    const MecanumGeometry &getGeometry() const;

    /**
     * @brief 逆解并归一化。
     * @param body 目标车体速度
     * @param max_wheel_speed_ips 轮端最大线速度；<=0 时不归一化
     * @return 四轮目标线速度；scale 保留原始最大值的缩放比例
     */
    WheelSpeeds inverse(const BodyVelocity &body, double max_wheel_speed_ips) const;

    /**
     * @brief 正解：四轮线速度 -> 车体速度。会先应用 wheel_direction。
     */
    BodyVelocity forward(const WheelSpeeds &wheels) const;

    std::array<double, 4> toWheelRpm(const WheelSpeeds &wheels) const;
    WheelSpeeds fromWheelRpm(const std::array<double, 4> &rpm) const;

private:
    double rotationFactorIn() const;

    MecanumGeometry m_geometry;
};

} // namespace motion
