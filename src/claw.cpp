#include "claw.hpp"

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include <algorithm>

namespace claw {
namespace {

// TODO: 占位端口，按实际接线修改。
constexpr int kClawPort = 8;
constexpr int kOpenPower = 80;
constexpr int kClosePower = -80;

// 两个蓝色齿轮啮合，一个电机驱动即可；另一个随动。
pros::Motor _g_claw(kClawPort, pros::MotorGears::blue, pros::MotorUnits::degrees);

} // namespace

void initialize() {
    _g_claw.set_brake_mode(pros::MotorBrake::brake);
}

void open() {
    set_power(kOpenPower);
}

void close() {
    set_power(kClosePower);
}

void set_power(int power) {
    if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
        stop();
        return;
    }
    _g_claw.move(std::clamp(power, -127, 127));
}

void stop() {
    _g_claw.brake();
}

} // namespace claw
