#pragma once
#include <cstdint>

namespace sim_physics {

enum class ToggleColor { Red, Blue, Yellow };

struct ToggleState {
    double x_m = 0.0;
    double y_m = 0.0;
    double angle_deg = 240.0; // 0=red, 120=blue, 240=yellow
    bool seated = true;
    bool touching = false;
    int quadrant = 0;
    ToggleColor face = ToggleColor::Yellow;
};

/**
 * @brief 围边四色 Toggle 的简化旋转模型。
 *
 * 机器人切向推动会改变角度；离开后吸附到最近的三面之一。只有完全吸附
 * 且没有机器人接触时，颜色才算有效，否则按规则视为黄色中立。
 */
class ToggleField {
public:
    static constexpr int kToggleCount = 4;

    explicit ToggleField(double field_size_m = 3.6576);

    void reset();
    void update(double dt_sec, double robot_x_m, double robot_y_m, double robot_vx_mps,
                double robot_vy_mps);

    const ToggleState &get(int index) const;
    int count() const;

private:
    double nearestDetent(double angle_deg) const;
    ToggleColor colorForAngle(double angle_deg) const;

    double m_field_size_m;
    ToggleState m_toggles[kToggleCount];
};

} // namespace sim_physics
