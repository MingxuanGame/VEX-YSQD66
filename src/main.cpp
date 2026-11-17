#include "main.h"
#include "macro/session.hpp"

void initialize() {
    macrorec::initialize();
}

void disabled() {
    macrorec::stop();
    while (pros::competition::is_disabled()) {
        macrorec::idle_tick();
        pros::delay(20);
    }
}

void competition_initialize() {
    macrorec::stop();
    while (pros::competition::is_disabled()) {
        macrorec::idle_tick();
        pros::delay(20);
    }
}

void autonomous() {}

void opcontrol() {
    macrorec::begin_driver();
    while (true) {
        auto wake_ms = pros::millis();
        macrorec::driver_tick();
        if (macrorec::driver_period_ms() == 1 || pros::millis() - wake_ms >= 20) {
            // 宏运行时每 1ms 检查帧边界；若 SD 操作跨周期则不再追赶。
            pros::delay(1);
        } else {
            pros::Task::delay_until(&wake_ms, 20);
        }
    }
}
