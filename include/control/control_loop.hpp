#pragma once
#include "control/drive_controller.hpp"
#include "control/mechanism_controller.hpp"
#include "control/mode_manager.hpp"
#include "io/input_state.hpp"
#include "motion/motion_engine.hpp"

namespace io {
class Controller;
} // namespace io

namespace robot {
class Display;
class Mechanism;
} // namespace robot

namespace tuning {
class ParamsManager;
} // namespace tuning

namespace control {

constexpr int kControlPeriodMs = 20; // 50Hz
constexpr uint32_t kWatchdogTimeoutMs = 100;

/**
 * @brief 主控调度层：读输入 -> 模式机 -> 驾驶/Auto/安全等待。
 *        每 20ms 调一次 update，内部没有任何阻塞等待。
 */
class ControlLoop {
public:
    ControlLoop(io::Controller &controller, robot::Chassis &chassis, robot::Imu &imu,
                robot::Battery *battery = nullptr);

    void setParamsManager(tuning::ParamsManager *params);
    void setMechanism(robot::Mechanism *mechanism);
    void setDisplay(robot::Display *display);

    // 视觉/距离传感器检测到锥体后注入目标航向；X 按住时生效。
    void setAssistTargetHeading(double heading_deg);
    void clearAssistTarget();

    void update(uint32_t now_ms);

    void startAuto();
    void endAuto();
    void reset();

    // 只在 Tune 模式以 100ms 周期刷新；无 Display 时安全 no-op。
    void renderUI(uint32_t now_ms);

    Mode getMode() const;
    const io::InputState &getInput() const;
    motion::MotionEngine &getMotionEngine();

private:
    void stopAll();
    void enterMode(Mode mode);
    void updateTuneInput();

    io::Controller *m_controller;
    robot::Chassis *m_chassis;

    io::InputState m_input;
    ModeManager m_mode;
    DriveController m_drive;
    MechanismController m_mechanism;
    motion::MotionEngine m_motion;

    tuning::ParamsManager *m_params;
    robot::Display *m_display;
    Mode m_last_mode;
    uint32_t m_last_ui_ms;
    uint32_t m_last_update_ms;
    bool m_has_update;
    bool m_auto_escape_active;
};

} // namespace control
