#include "main.h"

#include "autons.hpp"
#include "brain_ui.hpp"
#include "chassis.h"
#include "claw.hpp"
#include "toggle.hpp"
#include "io/controller.hpp"
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
    int claw_hold_last_ms = -1;
    bool claw_closed = false;

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

            if (controller.digital(io::Digital::R1)) {
                if (claw_closed) {
                    claw_hold_last_ms = pros::millis();
                }
                if (claw_hold_last_ms == -1) {
                    claw_hold_last_ms = pros::millis();
                }

                if (pros::millis() - claw_hold_last_ms >= 50) {
                    claw::close();
                    claw_closed = true;
                }
            }
            if (claw_closed && !controller.digital(io::Digital::R1) &&
                pros::millis() - claw_hold_last_ms >= 50) {
                claw::open();
                claw_hold_last_ms = -1;
                claw_closed = false;
            }

            if (controller.digital(io::Digital::Up)) {
                toggle::up();
            } else if (controller.digital(io::Digital::Down)) {
                toggle::down();
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
