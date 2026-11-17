#include "lift.hpp"

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include <algorithm>

namespace lift {
namespace {

// TODO: 按实际接线修改抬升电机端口和方向。
constexpr int kLiftPort = 5;

pros::Motor _g_lift(kLiftPort, pros::MotorGears::blue, pros::MotorUnits::degrees);

}

void initialize() {
    _g_lift.set_brake_mode(pros::MotorBrake::brake);
}

void set_power(int power) {
    if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
        stop();
        return;
    }
    _g_lift.move(std::clamp(power, -127, 127));
}

void stop() {
    _g_lift.brake();
}

} // namespace lift
