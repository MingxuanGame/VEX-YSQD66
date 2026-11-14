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
    auto wake_ms = pros::millis();
    while (true) {
        macrorec::driver_tick();
        pros::Task::delay_until(&wake_ms, 20);
    }
}
