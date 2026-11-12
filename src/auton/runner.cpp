#include "auton/kit.hpp"

#include <algorithm>
#include <cmath>

namespace auton {
namespace {
// TODO：转向控制初值（非实测 PID），需按实际底盘标定。
constexpr double kTurnKpMvPerDeg = 100;
constexpr double kTurnToleranceDeg = 1;
constexpr int kMinTurnMv = 1600;
constexpr int kMaxTurnMv = 6000;
constexpr std::uint32_t kTurnSettleMs = 60;
constexpr std::uint32_t kWatchdogMs = 100;
} // namespace

Runner::Runner(Hardware &hardware) : m_hardware(hardware) {}

int Runner::value(Param param) const {
    return m_route.steps[m_step].values[static_cast<int>(param)];
}

void Runner::fail(const char *reason) {
    m_state = State::Fault;
    m_status = reason;
    m_lifting = false;
    m_hardware.stop();
}

bool Runner::start(const Route &route, std::uint32_t now_ms, int first_step, bool single_step) {
    m_hardware.stop();
    m_lifting = false;
    if (first_step < 0 || first_step >= kStepCount) {
        fail("INVALID STEP");
        return false;
    }
    Config candidate{};
    candidate.routes.fill(route);
    if (!valid_config(candidate)) {
        fail("INVALID PARAMETERS");
        return false;
    }
    const int end = single_step ? first_step + 1 : kStepCount;
    for (int step = first_step; step < end; ++step) {
        for (Param required :
             {Param::Forward, Param::Action, Param::ContactTimeout, Param::TurnTimeout}) {
            if (uses_param(step, required) &&
                route.steps[step].values[static_cast<int>(required)] == 0) {
                fail("TODO: SET TIMES FIRST");
                return false;
            }
        }
    }
    if (!m_hardware.ready()) {
        fail("TODO: HARDWARE NOT READY");
        return false;
    }
    m_route = route;
    m_step = first_step;
    m_single = single_step;
    m_start_ms = now_ms;
    m_last_update_ms = now_ms;
    m_state = State::Running;
    m_status = "RUNNING";
    begin_step(now_ms);
    return m_state == State::Running;
}

bool Runner::enter(Phase phase, std::uint32_t now_ms) {
    m_phase = phase;
    m_phase_ms = now_ms;
    m_settling = false;
    bool ok = true;
    switch (phase) {
    case Phase::Turn:
    case Phase::ReturnTurn: {
        double heading_deg = 0;
        if (!m_hardware.heading(heading_deg) || !std::isfinite(heading_deg)) {
            fail("HEADING ERROR");
            return false;
        }
        const int sign = (m_step == 0 || m_step == 1 || m_step == 5) ? -1 : 1;
        m_target_deg = heading_deg + (phase == Phase::ReturnTurn ? 1 : sign) * value(Param::Angle);
        break;
    }
    case Phase::Toggle:
        ok = m_hardware.drive_voltage(0, 0) && m_hardware.action(Action::Toggle);
        break;
    case Phase::Back:
        if (value(Param::Back) == 0) {
            return enter(Phase::Turn, now_ms);
        }
        ok = m_hardware.drive_voltage(-kFullVoltageMv, -kFullVoltageMv);
        break;
    case Phase::Forward:
        ok = m_hardware.drive_voltage(kFullVoltageMv, kFullVoltageMv);
        if (m_step == 5 && value(Param::Lower) > 0) {
            ok = ok && m_hardware.lift_voltage(-kFullVoltageMv);
        }
        break;
    case Phase::Contact: {
        bool pressed = false;
        if (!m_hardware.contact(pressed) || pressed) {
            fail("CONTACT ERROR / ALREADY ON");
            return false;
        }
        m_arrived = false;
        if (m_step == 4 && value(Param::Lift) > 0) {
            m_lift_ms = now_ms;
            m_lifting = true;
            ok = m_hardware.lift_voltage(kFullVoltageMv);
        }
        ok = ok && m_hardware.drive_voltage(kFullVoltageMv, kFullVoltageMv);
        break;
    }
    case Phase::Lower:
        ok = m_hardware.drive_voltage(0, 0) &&
             m_hardware.lift_voltage(value(Param::Lower) > 0 ? -kFullVoltageMv : 0);
        break;
    case Phase::Release:
        ok = m_hardware.lift_voltage(0) && m_hardware.action(Action::Open);
        break;
    case Phase::Grab:
        ok = m_hardware.drive_voltage(0, 0) && m_hardware.action(Action::Close);
        break;
    }
    if (!ok) {
        fail("ACTUATOR / LIMIT ERROR");
    }
    return ok;
}

void Runner::begin_step(std::uint32_t now_ms) {
    m_lifting = false;
    if (m_step == 2 && value(Param::Lift) > 0) {
        m_lift_ms = now_ms;
        m_lifting = true;
        if (!m_hardware.lift_voltage(kFullVoltageMv)) {
            fail("LIFT ERROR");
            return;
        }
    }
    enter(m_step == 1 || m_step == 3 ? Phase::Back : Phase::Turn, now_ms);
}

void Runner::finish_step(std::uint32_t now_ms) {
    m_hardware.stop();
    m_lifting = false;
    if (m_single || m_step == kStepCount - 1) {
        m_state = State::Done;
        m_status = "DONE";
        return;
    }
    ++m_step;
    begin_step(now_ms);
}

void Runner::next_phase(std::uint32_t now_ms) {
    switch (m_phase) {
    case Phase::Back:
        enter(Phase::Turn, now_ms);
        break;
    case Phase::Turn:
        if (m_step == 0) {
            enter(Phase::Toggle, now_ms);
        } else if (m_step == 2 || m_step == 4) {
            enter(Phase::Contact, now_ms);
        } else {
            enter(Phase::Forward, now_ms);
        }
        break;
    case Phase::Toggle:
        enter(Phase::ReturnTurn, now_ms);
        break;
    case Phase::ReturnTurn:
        enter(Phase::Contact, now_ms);
        break;
    case Phase::Contact:
        enter(Phase::Lower, now_ms);
        break;
    case Phase::Lower:
        enter(Phase::Release, now_ms);
        break;
    case Phase::Forward:
        if (m_step == 5) {
            finish_step(now_ms);
        } else {
            enter(Phase::Grab, now_ms);
        }
        break;
    case Phase::Release:
    case Phase::Grab:
        finish_step(now_ms);
        break;
    }
}

void Runner::update(std::uint32_t now_ms, bool allowed) {
    if (m_state != State::Running) {
        return;
    }
    // <R9>，2.1 第 61 页：禁用和模式退出优先于一切目标。
    if (!allowed) {
        cancel();
        return;
    }
    if (now_ms - m_last_update_ms > kWatchdogMs) {
        fail("LOOP WATCHDOG");
        return;
    }
    m_last_update_ms = now_ms;
    if (now_ms - m_start_ms >= kAutoBudgetMs) {
        fail("AUTO TIME LIMIT");
        return;
    }
    if (!m_hardware.ready()) {
        fail("HARDWARE LOST");
        return;
    }
    if (m_lifting) {
        const bool completed = now_ms - m_lift_ms >= static_cast<unsigned>(value(Param::Lift));
        // 每周期重发以便适配器检查实时物理限位，不能只在开始抬升时检查一次。
        if (!m_hardware.lift_voltage(completed ? 0 : kFullVoltageMv)) {
            fail("LIFT / LIMIT ERROR");
            return;
        }
        m_lifting = !completed;
    }
    const auto elapsed = now_ms - m_phase_ms;
    bool done = false;
    switch (m_phase) {
    case Phase::Turn:
    case Phase::ReturnTurn: {
        if (elapsed >= static_cast<unsigned>(value(Param::TurnTimeout))) {
            fail("TURN TIMEOUT");
            return;
        }
        double heading_deg = 0;
        if (!m_hardware.heading(heading_deg) || !std::isfinite(heading_deg)) {
            fail("HEADING ERROR");
            return;
        }
        const double error_deg = m_target_deg - heading_deg;
        int output_mv = 0;
        if (std::abs(error_deg) <= kTurnToleranceDeg) {
            if (!m_settling) {
                m_settling = true;
                m_settle_ms = now_ms;
            }
            done = now_ms - m_settle_ms >= kTurnSettleMs;
        } else {
            m_settling = false;
            const double magnitude = std::clamp(std::abs(error_deg) * kTurnKpMvPerDeg,
                                                double(kMinTurnMv), double(kMaxTurnMv));
            output_mv = static_cast<int>(error_deg < 0 ? -magnitude : magnitude);
        }
        if (!m_hardware.drive_voltage(output_mv, -output_mv)) {
            fail("DRIVE ERROR");
            return;
        }
        break;
    }
    case Phase::Contact: {
        bool pressed = false;
        if (!m_hardware.contact(pressed)) {
            fail("CONTACT SENSOR ERROR");
            return;
        }
        if (pressed && !m_arrived) {
            m_arrived = true;
            if (!m_hardware.drive_voltage(0, 0) || (m_lifting && !m_hardware.lift_voltage(0))) {
                fail("CONTACT STOP ERROR");
                return;
            }
            m_lifting = false;
        }
        const int timeout_ms = m_step == 4
                                   ? std::min(value(Param::ContactTimeout), value(Param::Forward))
                                   : value(Param::ContactTimeout);
        if (!m_arrived && elapsed >= static_cast<unsigned>(timeout_ms)) {
            fail("CONTACT TIMEOUT");
            return;
        }
        // 到桩优先于抬升计时：立即停止底盘/抬升并进入下降，不原地补抬升。
        done = m_arrived;
        break;
    }
    case Phase::Back:
        done = elapsed >= static_cast<unsigned>(value(Param::Back));
        break;
    case Phase::Forward:
        done = elapsed >= static_cast<unsigned>(value(Param::Forward));
        if (m_step == 5) {
            if (!m_hardware.drive_voltage(done ? 0 : kFullVoltageMv, done ? 0 : kFullVoltageMv) ||
                !m_hardware.lift_voltage(
                    elapsed < static_cast<unsigned>(value(Param::Lower)) ? -kFullVoltageMv : 0)) {
                fail("IMPORT ACTUATOR ERROR");
                return;
            }
            done = done && elapsed >= static_cast<unsigned>(value(Param::Lower));
        }
        break;
    case Phase::Lower:
        done = elapsed >= static_cast<unsigned>(value(Param::Lower));
        if (!m_hardware.lift_voltage(done ? 0 : -kFullVoltageMv)) {
            fail("LOWER / LIMIT ERROR");
            return;
        }
        break;
    case Phase::Toggle:
    case Phase::Release:
    case Phase::Grab:
        done = elapsed >= static_cast<unsigned>(value(Param::Action));
        break;
    }
    if (done) {
        next_phase(now_ms);
    }
}

void Runner::cancel() {
    m_hardware.stop();
    m_lifting = false;
    m_state = State::Cancelled;
    m_status = "STOPPED";
}

State Runner::state() const {
    return m_state;
}

Phase Runner::phase() const {
    return m_phase;
}

int Runner::step() const {
    return m_step;
}

const char *Runner::status() const {
    return m_status;
}
} // namespace auton
