#include "lift.hpp"

#include "pros/misc.hpp"
#include "pros/motors.hpp"

#include <algorithm>

namespace lift {
namespace {

// TODO: 占位端口，按实际接线修改；IMU 固定使用端口 5，不能与 IMU 冲突。
constexpr int kLiftPort = 7;
constexpr int kUpPower = 80;
constexpr int kDownPower = -80;

enum class State { Idle, Up, Down };

pros::Motor _g_lift(kLiftPort, pros::MotorGears::blue, pros::MotorUnits::degrees);
State _g_state = State::Idle;
bool _g_last_pressed = false;

void apply_power(int power) {
    if (pros::competition::is_disabled() || pros::competition::is_autonomous()) {
        _g_lift.brake();
        _g_state = State::Idle;
        return;
    }
    _g_lift.move(std::clamp(power, -127, 127));
}

} // namespace

void initialize() {
    _g_lift.set_brake_mode(pros::MotorBrake::brake);
    _g_state = State::Idle;
    _g_last_pressed = false;
}

void toggle() {
    if (_g_state == State::Idle) {
        _g_state = State::Up;
    } else if (_g_state == State::Up) {
        _g_state = State::Down;
    } else {
        _g_state = State::Up;
    }
    apply_power(_g_state == State::Up ? kUpPower : kDownPower);
}

void update(bool pressed) {
    if (pressed && !_g_last_pressed) {
        toggle();
    }
    _g_last_pressed = pressed;
}

void set_power(int power) {
    _g_state = State::Idle;
    apply_power(power);
}

void stop() {
    _g_lift.brake();
    _g_state = State::Idle;
}

} // namespace lift
