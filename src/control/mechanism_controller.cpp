#include "control/mechanism_controller.hpp"
#include "io/input_state.hpp"
#include "robot/mechanism.hpp"

namespace control {

namespace {
constexpr int kLiftManualSpeed = 80;
} // namespace

void MechanismController::setMechanism(robot::Mechanism *mechanism) {
    m_mechanism = mechanism;
    m_clamp_open = true;
    m_push_extended = false;
}

void MechanismController::update(const io::InputState &input, uint32_t now_ms) {
    (void)now_ms;
    if (!m_mechanism)
        return;

    // L1+L2 是底盘的紧急脱困组合，机构必须先让路。
    if (input.l1.pressed() && input.l2.pressed()) {
        m_mechanism->liftHold();
        return;
    }

    if (input.r1.justPressed()) {
        m_clamp_open = !m_clamp_open;
        if (m_clamp_open) {
            m_mechanism->clampOpen();
        } else {
            m_mechanism->clampClose();
        }
    }

    if (input.r2.justPressed()) {
        m_push_extended = !m_push_extended;
        if (m_push_extended) {
            m_mechanism->pushExtend();
        } else {
            m_mechanism->pushRetract();
        }
    }

    if (input.l1.pressed()) {
        m_mechanism->liftManual(kLiftManualSpeed);
    } else if (input.l2.pressed()) {
        m_mechanism->liftManual(-kLiftManualSpeed);
    } else {
        m_mechanism->liftHold();
    }
}

void MechanismController::hold() {
    if (m_mechanism)
        m_mechanism->liftHold();
}

double MechanismController::getCurrentMa() const {
    return m_mechanism ? m_mechanism->getCurrentMa() : 0.0;
}

bool MechanismController::isHealthy() const {
    return m_mechanism ? m_mechanism->isHealthy() : true;
}

} // namespace control
