#include "claw.hpp"

#include "pros/adi.hpp"
#include "pros/misc.hpp"

namespace claw {
namespace {

constexpr char kClawPort = 'E';
constexpr bool kOpenOutput = false;
constexpr bool kClosedOutput = true;

bool _g_startup_locked = true;

const pros::adi::DigitalOut &solenoid() {
    static const pros::adi::DigitalOut output(kClawPort, kClosedOutput);
    return output;
}

void set_output(bool energized) {
    solenoid().set_value(energized);
}

} // namespace

void initialize() {
    solenoid();
}

bool unlock_startup() {
    if (!_g_startup_locked || pros::competition::is_disabled()) {
        return false;
    }
    _g_startup_locked = false;
    return true;
}

void open() {
    if (!_g_startup_locked) {
        set_output(kOpenOutput);
    }
}

void close() {
    set_output(kClosedOutput);
}

void stop() {
    if (_g_startup_locked) {
        close();
    } else {
        solenoid().set_value(kOpenOutput);
    }
}

} // namespace claw
