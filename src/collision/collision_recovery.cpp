#include "collision/collision_recovery.hpp"
#include "robot/chassis.hpp"
#include "robot/clock.hpp"
#include <cmath>

/**
 * @file collision_recovery.cpp
 * @brief 碰撞恢复逻辑实现。
 *        核心目标是：在有限时间内，尽最大可能通过多方向试探脱困，超时则立刻降级。
 */

namespace collision {

namespace {

// deadline 约定：0xFFFFFFFF 表示“没有截止时间”。用有符号差值判断，兼容 millis 回绕。
bool expired(uint32_t deadline_ms) {
    if (deadline_ms == 0xFFFFFFFFu)
        return false;
    return static_cast<int32_t>(robot::nowMs() - deadline_ms) >= 0;
}

uint32_t remainingMs(uint32_t deadline_ms) {
    if (deadline_ms == 0xFFFFFFFFu)
        return 0xFFFFFFFFu;
    uint32_t now = robot::nowMs();
    if (static_cast<int32_t>(now - deadline_ms) >= 0)
        return 0;
    return deadline_ms - now;
}

int clampOutput(int value) {
    if (value > 127)
        return 127;
    if (value < -127)
        return -127;
    return value;
}

} // namespace

CollisionRecovery::CollisionRecovery(const CollisionConfig &config) : m_config(config) {}

CollisionRecovery::CollisionRecovery(robot::Chassis &chassis, const CollisionConfig &config)
    : m_config(config), m_chassis(&chassis) {}

void CollisionRecovery::setChassis(robot::Chassis *chassis) {
    m_chassis = chassis;
}

bool CollisionRecovery::recover(CollisionType type, RetryFunc retry, FallbackFunc fallback,
                                uint32_t deadline_ms) {
    if (type == CollisionType::None)
        return true;

    // 回调为空时走 fallback，避免 std::bad_function_call（普通函数指针也不会崩）。
    if (!retry) {
        if (fallback) {
            fallback();
            return true;
        }
        return false;
    }

    if (type == CollisionType::Cone) {
        // 撞锥通常是去抓取的预期事件。说明已经到达目标位，直接重试（执行抓取）。
        return retry();
    }

    if (type == CollisionType::Car) {
        // 撞车：先短暂等待，给对手让开的反应时间，而不是盲目顶着推。
        if (!expired(deadline_ms)) {
            int wait_ms = m_config.car_wait_ms;
            if (wait_ms < 0)
                wait_ms = 0;
            uint32_t wait = static_cast<uint32_t>(wait_ms);
            uint32_t remain = remainingMs(deadline_ms);
            if (wait > remain)
                wait = remain;
            if (wait > 0)
                robot::delayMs(wait);
        }
        if (!expired(deadline_ms) && isPathClear()) {
            if (retry())
                return true; // 对方让开了，立刻重新执行当前步骤
        }
    }

    // 如果对方没让，或者撞的是死区：主动脱离。
    int retries = m_config.max_retries;
    if (retries < 0)
        retries = 0;
    if (retries > 10)
        retries = 10;
    for (int i = 0; i < retries; ++i) {
        if (expired(deadline_ms))
            break; // 时间到，强行退出

        if (escapeCollision(deadline_ms)) {
            if (retry())
                return true; // 脱困成功并完成了当前动作
        } else if (!expired(deadline_ms)) {
            // 脱离动作本身没起作用，稍作停顿后换方向再试。
            robot::delayMs(200);
        }
    }

    // 重试彻底失败，或者时间耗尽，调用备用路线拿保底分。
    // 按接口约定：走 fallback 也算“已处理”，返回 true。
    if (fallback) {
        fallback();
        return true;
    }
    return false;
}

bool CollisionRecovery::driveFor(uint32_t duration_ms, int left, int right, uint32_t deadline_ms) {
    if (!m_chassis)
        return false;
    if (expired(deadline_ms))
        return false;

    uint32_t remain = remainingMs(deadline_ms);
    uint32_t duration = duration_ms < remain ? duration_ms : remain;
    if (duration == 0)
        return false;

    m_chassis->setDrive(left, right);
    robot::delayMs(duration);
    m_chassis->stop(true);
    return !expired(deadline_ms);
}

bool CollisionRecovery::escapeCollision(uint32_t deadline_ms) {
    // 兼容旧接口：内部仍然是阻塞的，主循环请使用 start/updateEscapeAsync。
    if (!startEscapeAsync(robot::nowMs(), deadline_ms))
        return false;
    while (!updateEscapeAsync(robot::nowMs())) {
        robot::delayMs(10);
    }
    return escapeSucceeded();
}

bool CollisionRecovery::startEscapeAsync(uint32_t now_ms, uint32_t deadline_ms) {
    if (!m_chassis)
        return false;

    m_escape_phase = EscapePhase::Back;
    m_escape_phase_start_ms = now_ms;
    m_escape_deadline_ms = deadline_ms;
    m_escape_leg = 0;
    m_escape_probing = false;
    m_escape_succeeded = false;
    sendEscapeLeg(0);
    return true;
}

bool CollisionRecovery::updateEscapeAsync(uint32_t now_ms) {
    if (m_escape_phase == EscapePhase::Idle || m_escape_phase == EscapePhase::Done) {
        return m_escape_phase == EscapePhase::Done;
    }
    if (!m_chassis)
        return false;

    if (expired(m_escape_deadline_ms)) {
        finishEscape(false);
        return true;
    }

    if (m_escape_probing) {
        int probe_ms = m_config.path_probe_ms;
        if (probe_ms < 0)
            probe_ms = 0;
        if (probe_ms > 1000)
            probe_ms = 1000;
        if (static_cast<int32_t>(now_ms - m_escape_phase_start_ms) < probe_ms) {
            return false;
        }

        const double current = (m_chassis->getLeftCurrent() + m_chassis->getRightCurrent()) / 2.0;
        m_chassis->stop(true);
        if (current < m_config.escape_clear_current) {
            finishEscape(true);
            return true;
        }

        ++m_escape_leg;
        if (m_escape_leg > 2 || !m_config.enable_side_escape) {
            finishEscape(false);
            return true;
        }
        m_escape_probing = false;
        m_escape_phase = EscapePhase::Back;
        m_escape_phase_start_ms = now_ms;
        sendEscapeLeg(m_escape_leg);
        return false;
    }

    int back_ms = m_config.escape_back_ms;
    if (back_ms < 0)
        back_ms = 0;
    if (back_ms > 5000)
        back_ms = 5000;
    if (static_cast<int32_t>(now_ms - m_escape_phase_start_ms) < back_ms) {
        return false;
    }

    // 该方向退完后，给一点向前的动力探测路是否通了。
    m_chassis->stop(true);
    m_escape_probing = true;
    m_escape_phase = EscapePhase::Probe;
    m_escape_phase_start_ms = now_ms;
    m_chassis->setDrive(clampOutput(m_config.path_probe_speed),
                        clampOutput(m_config.path_probe_speed));
    return false;
}

bool CollisionRecovery::isEscaping() const {
    return m_escape_phase == EscapePhase::Back || m_escape_phase == EscapePhase::Probe;
}

bool CollisionRecovery::escapeSucceeded() const {
    return m_escape_phase == EscapePhase::Done && m_escape_succeeded;
}

void CollisionRecovery::sendEscapeLeg(int leg) {
    if (!m_chassis)
        return;
    const int speed = clampOutput(m_config.escape_speed);
    switch (leg) {
    case 0:
        m_chassis->setDrive(-speed, -speed);
        break;
    case 1:
        m_chassis->setDrive(-speed / 2, -speed);
        break;
    default:
        m_chassis->setDrive(-speed, -speed / 2);
        break;
    }
}

void CollisionRecovery::finishEscape(bool success) {
    if (m_chassis)
        m_chassis->stop(true);
    m_escape_phase = EscapePhase::Done;
    m_escape_succeeded = success;
    m_escape_probing = false;
}

bool CollisionRecovery::isPathClear() {
    if (!m_chassis)
        return false;

    // 给一点极小的动力向前试探：后退脱困后，要确认原方向是否已经通畅。
    int probe_speed = clampOutput(m_config.path_probe_speed);
    int probe_ms = m_config.path_probe_ms;
    if (probe_ms < 0)
        probe_ms = 0;
    if (probe_ms > 1000)
        probe_ms = 1000;

    m_chassis->setDrive(probe_speed, probe_speed);
    robot::delayMs(static_cast<uint32_t>(probe_ms));
    double current = (m_chassis->getLeftCurrent() + m_chassis->getRightCurrent()) / 2.0;
    m_chassis->stop(true);

    // 如果电流很低，说明没顶着东西，路通了。
    return (current < m_config.escape_clear_current);
}

bool CollisionRecovery::updateAntiPush(uint32_t now) {
    if (!m_chassis) {
        m_push_start_time = 0;
        m_is_being_pushed = false;
        return false;
    }

    double current = (m_chassis->getLeftCurrent() + m_chassis->getRightCurrent()) / 2.0;
    double velocity = (m_chassis->getLeftVelocity() + m_chassis->getRightVelocity()) / 2.0;

    int push_time = m_config.ant_push_time_ms;
    if (push_time < 0)
        push_time = 0;

    // 如果电流很大，且车速几乎为 0，说明我们在用力推，但推不动。
    bool is_pushing = (current > m_config.ant_push_current_threshold && std::abs(velocity) < 10.0);

    if (is_pushing) {
        if (m_push_start_time == 0) {
            // 用 0 当“未开始”哨兵，避免 now==0 时第一帧无法记录。
            m_push_start_time = (now == 0) ? 1 : now;
            m_is_being_pushed = false;
        } else if (static_cast<int32_t>(now - m_push_start_time) > push_time) {
            // 长时间推不动，判定为被对方卡死或顶住。
            m_is_being_pushed = true;
        }
    } else {
        m_push_start_time = 0;
        m_is_being_pushed = false;
    }

    // 如果判定被推挤，主动反向卸力，而不是继续爆电流对抗，保护底盘电机。
    if (m_is_being_pushed) {
        int escape = clampOutput(m_config.ant_push_escape_speed);
        m_chassis->setDrive(escape, escape);
        return true;
    }
    return false;
}

} // namespace collision
