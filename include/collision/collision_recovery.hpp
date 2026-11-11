#pragma once
#include "collision/collision_types.hpp"

/**
 * @file collision_recovery.hpp
 * @brief 碰撞恢复器：包含主动脱离、超时中止、防推挤。
 *        恢复动作受 deadline 约束，防止自动赛在最后几秒卡死在恢复逻辑上。
 */

namespace robot {
class Chassis;
} // namespace robot

namespace collision {

// 用普通函数指针代替 std::function，避免堆分配。
using RetryFunc = bool (*)();
using FallbackFunc = void (*)();

class CollisionRecovery {
public:
    enum class EscapePhase { Idle, Back, Probe, Done };

    explicit CollisionRecovery(const CollisionConfig &config = {});

    /**
     * @brief 注入底盘。恢复动作全部通过 robot::Chassis 执行。
     */
    CollisionRecovery(robot::Chassis &chassis, const CollisionConfig &config = {});

    void setChassis(robot::Chassis *chassis);

    /**
     * @brief 执行碰撞恢复。
     * @param type 碰撞类型。
     * @param retry 重试回调（通常封装当前步骤）。
     * @param fallback 失败后的备用回调。
     * @param deadline_ms 截止时间戳。如果当前时间超过此值，立刻放弃并走 Fallback。
     * @return 是否最终成功（无论是重试成功还是走了 Fallback 保底）。
     * @note 自动赛时间紧迫时，传入系统时间 + 剩余时间，防止无限重试。
     */
    bool recover(CollisionType type, RetryFunc retry, FallbackFunc fallback,
                 uint32_t deadline_ms = 0xFFFFFFFF);

    /**
     * @brief 防推挤监控：检测是否被对方持续顶推，并自动反向卸力。
     * @param now 当前时间（由主循环传入，避免频繁调用 millis()）。
     * @return true 表示正在执行卸力动作。
     * @note 在手动赛的驾驶循环中每帧调用，保护电机不被烧毁。
     */
    bool updateAntiPush(uint32_t now);

    /**
     * @brief 主动多方向脱离（后、左后、右后）。
     * @param deadline_ms 截止时间戳，超时立刻中断脱离。
     * @return true 表示脱困成功，false 表示被卡死。
     */
    bool escapeCollision(uint32_t deadline_ms = 0xFFFFFFFF);

    /**
     * @brief 非阻塞脱困：startEscapeAsync() 后每帧 updateEscapeAsync()。
     *        50Hz 主循环/自动赛应使用这组接口，不要调用带 delay 的 escapeCollision()。
     */
    bool startEscapeAsync(uint32_t now_ms, uint32_t deadline_ms = 0xFFFFFFFF);
    bool updateEscapeAsync(uint32_t now_ms);
    bool isEscaping() const;
    bool escapeSucceeded() const;

    /**
     * @brief 判断路是否通了（短促给动力试探）。
     * @return true 表示对方让开或已经脱困。
     */
    bool isPathClear();

private:
    /**
     * @brief 按时间驱动底盘，并在动作结束后主动刹停。
     * @return false 表示还没开始就超过 deadline。
     */
    bool driveFor(uint32_t duration_ms, int left, int right, uint32_t deadline_ms);

    void sendEscapeLeg(int leg);
    void finishEscape(bool success);

    CollisionConfig m_config;
    robot::Chassis *m_chassis = nullptr;

    uint32_t m_push_start_time = 0;
    bool m_is_being_pushed = false;

    EscapePhase m_escape_phase = EscapePhase::Idle;
    uint32_t m_escape_phase_start_ms = 0;
    uint32_t m_escape_deadline_ms = 0xFFFFFFFF;
    int m_escape_leg = 0;
    bool m_escape_probing = false;
    bool m_escape_succeeded = false;
};

} // namespace collision
