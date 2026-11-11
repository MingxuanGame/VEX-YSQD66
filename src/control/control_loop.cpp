#include "control/control_loop.hpp"
#include "io/controller.hpp"
#include "io/input_reader.hpp"
#include "robot/chassis.hpp"
#include "robot/display.hpp"
#include "robot/imu.hpp"
#include "tuning/param_apply.hpp"
#include "tuning/params_manager.hpp"
#include <cstdio>

namespace control {

ControlLoop::ControlLoop(io::Controller &controller, robot::Chassis &chassis, robot::Imu &imu,
                         robot::Battery *battery)
    : m_controller(&controller), m_chassis(&chassis), m_input(), m_mode(), m_drive(chassis),
      m_mechanism(), m_motion(chassis, imu, battery), m_params(nullptr), m_display(nullptr),
      m_last_mode(Mode::Drive), m_last_ui_ms(0), m_last_update_ms(0), m_has_update(false),
      m_auto_escape_active(false) {
    m_drive.setImu(&imu);
    m_drive.setBattery(battery);
}

void ControlLoop::setParamsManager(tuning::ParamsManager *params) {
    m_params = params;
}

void ControlLoop::setMechanism(robot::Mechanism *mechanism) {
    m_mechanism.setMechanism(mechanism);
}

void ControlLoop::setDisplay(robot::Display *display) {
    m_display = display;
    m_last_ui_ms = 0;
}

void ControlLoop::setAssistTargetHeading(double heading_deg) {
    m_drive.setAssistTargetHeading(heading_deg);
}

void ControlLoop::clearAssistTarget() {
    m_drive.clearAssistTarget();
}

void ControlLoop::updateTuneInput() {
    if (!m_params)
        return;

    if (m_input.up.justPressed())
        m_params->selectPrev();
    if (m_input.down.justPressed())
        m_params->selectNext();
    if (m_input.left.justPressed())
        m_params->prevGroup();
    if (m_input.right.justPressed())
        m_params->nextGroup();

    tuning::TunableParam *param = m_params->getSelectedParam();
    if (!param)
        return;

    if (m_input.x.justPressed())
        param->decParam();
    if (m_input.y.justPressed())
        param->incParam();
    if (m_input.a.justPressed())
        param->saveParam();
}

void ControlLoop::update(uint32_t now_ms) {
    if (!m_controller || !m_chassis)
        return;

    // 软件看门狗：如果主循环被 SD 卡/I2C/日志卡住，恢复后的第一帧
    // 先刹车并进入 SafeWait，不能带着旧输出继续跑。
    if (m_has_update && static_cast<int32_t>(now_ms - m_last_update_ms) > kWatchdogTimeoutMs) {
        stopAll();
        m_mode.requestSafeWait();
        m_last_mode = Mode::SafeWait;
        m_last_update_ms = now_ms;
        return;
    }
    m_last_update_ms = now_ms;
    m_has_update = true;

    io::readInput(*m_controller, m_input, now_ms);

    // 断联最高优先级：所有模式都立即刹车并进入安全等待。
    if (!m_input.isConnected) {
        stopAll();
        m_mode.requestSafeWait();
        m_last_mode = Mode::SafeWait;
        return;
    }

    // 自动赛阶段也保留 L1+L2 脱困底牌；先停 MotionEngine，再交给受限脱困。
    // 松手后强制进入 SafeWait，避免自动赛代码在脱困后继续接管底盘。
    if (m_mode.getMode() == Mode::Auto) {
        if (m_input.l1.pressed() && m_input.l2.pressed()) {
            m_motion.stop();
            m_drive.update(m_input, now_ms);
            m_auto_escape_active = true;
            return;
        }
        if (m_auto_escape_active) {
            m_auto_escape_active = false;
            stopAll();
            m_mode.requestSafeWait();
            m_last_mode = Mode::SafeWait;
            return;
        }
    }

    const Mode mode = m_mode.update(m_input, now_ms);

    // 从 TUNE 退出时统一参数下发。
    if (m_mode.consumeTuneExit() && m_params) {
        tuning::ParamApply::applyAllDirty(*m_params);
    }

    enterMode(mode);

    switch (mode) {
    case Mode::Drive:
        // 机构先更新，DriveController 才能拿到本帧的机构电流预算。
        m_mechanism.update(m_input, now_ms);
        m_drive.setAuxCurrentMa(m_mechanism.getCurrentMa());
        m_drive.update(m_input, now_ms);
        break;
    case Mode::Tune:
        // Tune 默认锁死；按住 L1 才允许低速点动，松手立即刹车。
        m_mechanism.hold();
        if (m_input.l1.pressed()) {
            m_drive.setAuxCurrentMa(m_mechanism.getCurrentMa());
            m_drive.update(m_input, now_ms);
        } else {
            m_chassis->stop(true);
            updateTuneInput();
        }
        renderUI(now_ms);
        break;
    case Mode::Auto:
        m_mechanism.hold();
        m_motion.update(now_ms, true);
        // MotionEngine 超时/故障必须收敛到 SafeWait，不能继续留在 Auto。
        if (m_motion.getState() == motion::State::Fault) {
            stopAll();
            m_mode.requestSafeWait();
            m_last_mode = Mode::SafeWait;
        }
        break;
    case Mode::SafeWait:
        m_mechanism.hold();
        m_chassis->stop(true);
        break;
    }
}

void ControlLoop::startAuto() {
    m_drive.stop();
    m_mechanism.hold();
    m_motion.stop();
    m_mode.startAuto();
    m_last_mode = Mode::Auto;
    m_auto_escape_active = false;
    m_has_update = false;
}

void ControlLoop::endAuto() {
    // 自动赛结束可能还在高速冲刺：先硬刹车、清 PID/速度残留，再交还手动。
    m_chassis->stop(true);
    m_motion.stop();
    m_drive.stop();
    m_mechanism.hold();
    m_mode.endAuto();
    m_last_mode = Mode::Drive;
    m_auto_escape_active = false;
    m_has_update = false;
}

void ControlLoop::renderUI(uint32_t now_ms) {
    if (m_mode.getMode() != Mode::Tune || !m_display)
        return;
    if (static_cast<int32_t>(now_ms - m_last_ui_ms) < 100)
        return;
    m_last_ui_ms = now_ms;

    if (!m_params) {
        m_display->clearLine(0);
        m_display->printLine(0, "TUNE (no params)");
        return;
    }

    const tuning::TunableParam *param = m_params->getSelectedParam();
    if (!param) {
        m_display->clearLine(0);
        m_display->printLine(0, "TUNE (empty)");
        return;
    }

    char line[40];
    if (param->getParamType() == tuning::ParamType::Double) {
        std::snprintf(line, sizeof(line), "%-12s %.2f", param->getParamName(),
                      param->getDoubleValue());
    } else {
        std::snprintf(line, sizeof(line), "%-12s %d", param->getParamName(), param->getIntValue());
    }
    m_display->clearLine(0);
    m_display->printLine(0, line);

    std::snprintf(line, sizeof(line), "TUNE  group=%d", static_cast<int>(m_params->getGroup()));
    m_display->clearLine(1);
    m_display->printLine(1, line);
}

Mode ControlLoop::getMode() const {
    return m_mode.getMode();
}

const io::InputState &ControlLoop::getInput() const {
    return m_input;
}

motion::MotionEngine &ControlLoop::getMotionEngine() {
    return m_motion;
}

void ControlLoop::stopAll() {
    m_drive.stop();
    m_mechanism.hold();
    m_motion.stop();
    if (m_chassis)
        m_chassis->stop(true);
}

void ControlLoop::enterMode(Mode mode) {
    if (mode == m_last_mode)
        return;

    // 状态隔离：
    // 1) 离开 AUTO 时必须停 MotionEngine，否则自动赛最后一条 setDrive 会
    //    残留到手动模式，操作手会感觉底盘“自己动了一下”。
    // 2) 进入任何非 DRIVE 模式前停 DriveController，清掉手动残余输出、
    //    脱困相位和防推挤计时，避免 TUNE/SAFE_WAIT 的 stop 被上一帧覆盖。
    if (m_last_mode == Mode::Auto)
        m_motion.stop();
    if (mode != Mode::Drive) {
        m_drive.stop();
        m_mechanism.hold();
    }

    // Tune 点动必须限速；离开 Tune 恢复正常上限。
    if (mode == Mode::Tune) {
        m_drive.setMaxOutput(m_drive.getConfig().tune_max_output);
    } else if (m_last_mode == Mode::Tune) {
        m_drive.setMaxOutput(m_drive.getConfig().max_output);
    }

    m_last_mode = mode;
}

} // namespace control
