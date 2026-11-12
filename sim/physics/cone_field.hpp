#pragma once
#include <cstdint>

namespace sim_physics {

/**
 * @brief 一组可在场地内被底盘推动的起始锥。
 *
 * 这是仿真的简化模型：每组锥用一个圆形碰撞体表示，推离场地围边后
 * 进入 out_of_bounds 状态，不再阻塞底盘。视觉位移由 UI 读取位置绘制。
 */
struct ConeBody {
    double x_m = 0.0;
    double y_m = 0.0;
    double velocity_x_mps = 0.0;
    double velocity_y_mps = 0.0;
    double radius_m = 0.10;
    bool active = true;
    bool out_of_bounds = false;
};

class ConeField {
public:
    static constexpr int kConeCount = 4;

    explicit ConeField(double field_size_m = 3.6576);

    void reset();

    /**
     * @brief 推进一步锥体运动。
     * @param dt_sec 物理步长，单位秒
     * @param robot_x_m 底盘中心 X
     * @param robot_y_m 底盘中心 Y
     * @param robot_vx_mps 底盘 X 方向速度
     * @param robot_vy_mps 底盘 Y 方向速度
     */
    void update(double dt_sec, double robot_x_m, double robot_y_m, double robot_vx_mps,
                double robot_vy_mps);

    bool hasContact() const;
    const ConeBody &get(int index) const;
    int count() const;

private:
    double m_field_size_m;
    ConeBody m_cones[kConeCount];
    bool m_contact;
};

} // namespace sim_physics
