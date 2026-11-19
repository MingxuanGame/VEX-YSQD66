#include "claw.hpp"

#include "pros/adi.hpp"
#include "pros/misc.hpp"

namespace claw {
namespace {

constexpr char kClawPort = 'E';
constexpr bool kOpenOutput = false;
constexpr bool kClosedOutput = true;

enum class Mode { Driver, Autonomous };

const pros::adi::DigitalOut &solenoid() {
    static const pros::adi::DigitalOut output(kClawPort, kOpenOutput);
    return output;
}

void set_output(bool energized, Mode mode) {
    // Override 2.1 <R9>：只接收当前启用模式的夹爪指令。
    if (!pros::competition::is_disabled() &&
        pros::competition::is_autonomous() == (mode == Mode::Autonomous)) {
        solenoid().set_value(energized);
    }
}

} // namespace

void initialize() {
    solenoid();
}

void open() {
    set_output(kOpenOutput, Mode::Driver);
}

void close() {
    set_output(kClosedOutput, Mode::Driver);
}

void open_autonomous() {
    set_output(kOpenOutput, Mode::Autonomous);
}

void close_autonomous() {
    set_output(kClosedOutput, Mode::Autonomous);
}

void stop() {
    solenoid().set_value(kOpenOutput);
}

} // namespace claw
