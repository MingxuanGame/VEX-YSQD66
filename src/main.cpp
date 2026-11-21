#include "main.h"

#include "autons.hpp"
#include "brain_ui.hpp"
#include "chassis.h"
#include "claw.hpp"
#include "io/controller.hpp"
#include "io/pros_controller.hpp"
#include "lifting.hpp"
#include "toggle.hpp"

#include <cstdint>

namespace {

constexpr std::uint32_t kDriverPeriodMs = 20;
constexpr std::uint32_t kClawButtonDelayMs = 50;
constexpr int kLiftPower = 80;

void stop_all() {
    chassis::stop();
    lifting::stop();
    claw::stop();
    toggle::stop();
}

} // namespace

void initialize() {
    claw::initialize();
    brain_ui::initialize();
    chassis::initialize();
    lifting::initialize();
    toggle::initialize();
}

void disabled() {
    stop_all();
    while (pros::competition::is_disabled()) {
        brain_ui::tick();
        pros::delay(kDriverPeriodMs);
    }
}

void competition_initialize() {
    stop_all();
    while (pros::competition::is_disabled()) {
        brain_ui::tick();
        pros::delay(kDriverPeriodMs);
    }
}

void autonomous() {
    autons::run();
    stop_all();
}

void opcontrol() {
    stop_all();
    bool manual_lift_armed = false;
    std::uint32_t claw_button_changed_ms = 0;
    bool claw_button_held = false;
    bool claw_closed = false;
    bool auto_start_held = false;

    while (true) {
        auto wake_ms = pros::millis();
        brain_ui::tick();

        auto &controller = io::prosController();
        if (controller.isConnected()) {
            const int throttle = controller.analog(io::Analog::LeftY);
            const int turn = controller.analog(io::Analog::RightX);
            chassis::arcade(throttle, turn);

            const bool lift_up = controller.digital(io::Digital::L1);
            const bool lift_down = controller.digital(io::Digital::L2);
            manual_lift_armed |= (!lift_up && !lift_down);
            const int lift_cmd =
                manual_lift_armed
                    ? kLiftPower * (static_cast<int>(lift_down) - static_cast<int>(lift_up))
                    : 0;
            lifting::set_power(lift_cmd);

            const bool claw_button_pressed = controller.digital(io::Digital::R1);
            if (claw_button_pressed != claw_button_held) {
                claw_button_held = claw_button_pressed;
                claw_button_changed_ms = wake_ms;
                if (claw_button_pressed && claw::unlock_startup()) {
                    claw_closed = true;
                }
            }
            if (claw_button_held != claw_closed &&
                wake_ms - claw_button_changed_ms >= kClawButtonDelayMs) {
                if (claw_button_held) {
                    claw::close();
                } else {
                    claw::open();
                }
                claw_closed = claw_button_held;
            }

            if (controller.digital(io::Digital::Up)) {
                toggle::up();
            } else if (controller.digital(io::Digital::Down)) {
                toggle::down();
            }

            // 按 A 运行一次所选自动路线；返回后继续手动控制，仅按下沿触发避免重复启动。
            const bool auto_start = controller.digital(io::Digital::A);
            if (auto_start && !auto_start_held) {
                stop_all();
                autons::run_in_driver();
                stop_all();
                claw_button_held = false;
                claw_closed = false;
            }
            auto_start_held = auto_start;
        } else {
            stop_all();
            manual_lift_armed = false;
            claw_button_held = false;
            claw_closed = false;
            auto_start_held = false;
        }

        if (pros::millis() - wake_ms >= kDriverPeriodMs) {
            pros::delay(1);
        } else {
            pros::Task::delay_until(&wake_ms, kDriverPeriodMs);
        }
    }
}
