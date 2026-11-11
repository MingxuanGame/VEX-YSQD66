#include "control/mode_manager.hpp"
#include "io/input_state.hpp"

namespace control {

ModeManager::ModeManager() : m_mode(Mode::Drive), m_tune_exit_pending(false) {}

Mode ModeManager::update(const io::InputState &input, uint32_t now_ms) {
    (void)now_ms;

    switch (m_mode) {
    case Mode::Drive:
        // 长按 Y 进入调参，避免比赛中误触。
        if (input.y.longPressed()) {
            m_mode = Mode::Tune;
        }
        break;

    case Mode::Tune:
        // B 退出调参；退出后由 ControlLoop 统一下发参数。
        if (input.b.justPressed()) {
            m_mode = Mode::Drive;
            m_tune_exit_pending = true;
        }
        break;

    case Mode::Auto:
        // 自动赛完全屏蔽手柄状态切换。
        break;

    case Mode::SafeWait:
        // 安全等待：A/B 任意一个确认恢复。
        if (input.a.justPressed() || input.b.justPressed()) {
            m_mode = Mode::Drive;
        }
        break;
    }

    return m_mode;
}

Mode ModeManager::getMode() const {
    return m_mode;
}

bool ModeManager::isTuneExitPending() const {
    return m_tune_exit_pending;
}

bool ModeManager::consumeTuneExit() {
    const bool pending = m_tune_exit_pending;
    m_tune_exit_pending = false;
    return pending;
}

void ModeManager::startAuto() {
    m_mode = Mode::Auto;
}

void ModeManager::endAuto() {
    m_mode = Mode::Drive;
}

void ModeManager::requestSafeWait() {
    m_mode = Mode::SafeWait;
}

void ModeManager::clearSafeWait() {
    if (m_mode == Mode::SafeWait)
        m_mode = Mode::Drive;
}

} // namespace control
