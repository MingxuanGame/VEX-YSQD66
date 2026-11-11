#pragma once
#include <cstdint>

namespace io {
struct InputState;
} // namespace io

namespace robot {
class Mechanism;
} // namespace robot

namespace control {

/**
 * @brief Drive 模式下的机构按键分发。
 *
 * 映射：
 *   R1  -> 夹爪开/合切换
 *   R2  -> 推板伸/缩切换
 *   L1  -> 抬升上行（按住）
 *   L2  -> 抬升下行（按住）
 *   L1+L2 -> 保留给 DriveController 的紧急脱困，机构此处只做 hold。
 *
 * 所有动作都是边沿触发/按住即动，不阻塞主循环。
 */
class MechanismController {
public:
    MechanismController() = default;

    void setMechanism(robot::Mechanism *mechanism);
    void update(const io::InputState &input, uint32_t now_ms);
    void hold();

    double getCurrentMa() const;

private:
    robot::Mechanism *m_mechanism = nullptr;
    bool m_clamp_open = true; // SimMechanism 默认是打开姿态，第一次 R1 应闭合
    bool m_push_extended = false;
};

} // namespace control
