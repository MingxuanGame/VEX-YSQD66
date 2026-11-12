#pragma once
#include <cstdint>

namespace sim_physics {

/**
 * @brief 可被底盘推动、并能彼此碰撞的锥体。
 *
 * 四锥组合拆成 4 个独立锥体；双黄锥拆成 2 个独立锥体。
 * 这样底盘撞到组合中的单个锥时，只推动该锥，而不是整组刚性平移。
 */
enum class ConeColor { Red, Blue, Yellow, DoubleYellow };

struct ConeBody {
    double x_m = 0.0;
    double y_m = 0.0;
    double velocity_x_mps = 0.0;
    double velocity_y_mps = 0.0;
    double radius_m = 0.030;
    int group = 0;
    ConeColor color = ConeColor::Yellow;
    bool active = true;
    bool out_of_bounds = false;
};

class ConeField {
public:
    static constexpr int kGroupCount = 4;
    static constexpr int kConesPerGroup = 4;
    static constexpr int kDoubleCount = 2;
    static constexpr int kConesPerDouble = 2;
    static constexpr int kConeCount = kGroupCount * kConesPerGroup + kDoubleCount * kConesPerDouble;

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
    void resolveConePair(int first, int second);

    double m_field_size_m;
    ConeBody m_cones[kConeCount];
    bool m_contact;
};

} // namespace sim_physics
