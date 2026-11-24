#include "pros/adi.hpp"
#include "pros/misc.hpp"

namespace toggle {
namespace {
constexpr char kTogglePort = 'D';
constexpr bool kUpOutput = false;
constexpr bool kDownOutput = true;

const pros::adi::DigitalOut &solenoid() {
    static const pros::adi::DigitalOut output(kTogglePort, kUpOutput);
    return output;
}

void set_output(bool energized) {
    if (!pros::competition::is_disabled()) {
        solenoid().set_value(energized);
    }
}
} // namespace

void initialize() {
    solenoid();
}

void up() {
    set_output(kUpOutput);
}

void down() {
    set_output(kDownOutput);
}

void stop() {
    solenoid().set_value(kUpOutput);
}
} // namespace toggle
