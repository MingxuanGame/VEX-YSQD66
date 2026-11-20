#include "claw.hpp"

#include "pros/adi.hpp"
#include "pros/misc.hpp"

namespace claw {
namespace {

constexpr char kClawPort = 'E';
constexpr bool kOpenOutput = false;
constexpr bool kClosedOutput = true;

const pros::adi::DigitalOut &solenoid() {
    static const pros::adi::DigitalOut output(kClawPort, kOpenOutput);
    return output;
}

void set_output(bool energized) {
    // Override 2.1 <R9>：仅在机器人启用时接受夹爪指令。
    if (!pros::competition::is_disabled()) {
        solenoid().set_value(energized);
    }
}

} // namespace

void initialize() {
    solenoid();
}

void open() {
    set_output(kOpenOutput);
}

void close() {
    set_output(kClosedOutput);
}

void stop() {
    solenoid().set_value(kOpenOutput);
}

} // namespace claw
