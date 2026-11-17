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
        if (macrorec::driver_period_ms() == 1) {
            // 宏每 1ms 检查边界；显式让出 CPU，避免过期的 delay_until 连续追赶。
            pros::delay(1);
        } else if (pros::millis() - wake_ms >= 20) {
            // 停车后的 SD 操作可以超出一周期，但不能导致下一轮忙等追赶。
            pros::delay(1);
        } else {
            pros::Task::delay_until(&wake_ms, 20);
        }
    }
}
