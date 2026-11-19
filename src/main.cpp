#include "main.h"

#include "autons.hpp"
#include "brain_ui.hpp"
#include "chassis.h"
#include "claw.hpp"
#include "io/pros_controller.hpp"
#include "lifting.hpp"

#include <cstdint>

namespace {

constexpr std::uint32_t kDriverPeriodMs = 20;
constexpr int kLiftPower = 80;

void stop_all() {
    chassis::stop();
    lifting::stop();
    claw::stop();
}

} // namespace

void initialize() {
    brain_ui::initialize();
    chassis::initialize();
    lifting::initialize();
    claw::initialize();
}

void disabled() {
    stop_all();
    brain_ui::reset_lift_calibration();
    while (pros::competition::is_disabled()) {
        brain_ui::tick(false);
        pros::delay(kDriverPeriodMs);
    }
}

void competition_initialize() {
    stop_all();
    brain_ui::reset_lift_calibration();
    while (pros::competition::is_disabled()) {
        brain_ui::tick(false);
        pros::delay(kDriverPeriodMs);
    }
}

void autonomous() {
    brain_ui::reset_lift_calibration();
    // 已知问题：load_limits 只有在 lifting 已建立 lower_reference 时才会生成限位。
    // 冷启动直接进自动赛时，autons 可能因 !lifting::limits() 直接返回。
    // TODO: 自动开始前确认机器人处于下限，并调用 use_saved_limits(current)。
    lifting::load_limits();
    autons::run();
    stop_all();
}

void opcontrol() {
    stop_all();
    brain_ui::reset_lift_calibration();
    // 已知行为：进入手动后必须先松开 L1/L2 一次，第一次按键才会驱动升降。
    bool manual_lift_armed = false;

    while (true) {
        auto wake_ms = pros::millis();
        brain_ui::tick(true);

        auto &controller = io::prosController();
        if (controller.isConnected()) {
            const int throttle = controller.analog(io::Analog::LeftY);
            const int turn = controller.analog(io::Analog::RightX);
            chassis::arcade(throttle, turn);

            if (brain_ui::lift_jog_mode()) {
                manual_lift_armed = false;
            } else {
                const bool lift_up = controller.digital(io::Digital::L2);
                const bool lift_down = controller.digital(io::Digital::L1);
                manual_lift_armed |= (!lift_up && !lift_down);
                const int lift_cmd =
                    manual_lift_armed
                        ? kLiftPower * (static_cast<int>(lift_up) - static_cast<int>(lift_down))
                        : 0;
                lifting::set_power(lift_cmd);
            }

            // 已知行为：松开 R1/R2 时保持气阀状态；stop_all 才回到开/放气。
            const bool claw_open = controller.digital(io::Digital::R1);
            const bool claw_close = controller.digital(io::Digital::R2);
            if (claw_open && !claw_close) {
                claw::open();
            } else if (claw_close && !claw_open) {
                claw::close();
            }
        } else {
            stop_all();
            manual_lift_armed = false;
        }

        if (pros::millis() - wake_ms >= kDriverPeriodMs) {
            pros::delay(1);
        } else {
            pros::Task::delay_until(&wake_ms, kDriverPeriodMs);
        }
    }
}
