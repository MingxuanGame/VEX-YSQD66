#include "io/pros_controller.hpp"
#include "pros/misc.hpp"

/**
 * @file pros_controller.cpp
 * @brief pros::Controller 到 io::Controller 的适配器。
 *        这是硬件层，只有这个文件可以接触 pros/misc.hpp。
 */

namespace io {

namespace {

class ProsController : public Controller {
public:
    bool isConnected() const override {
        return m_master.is_connected();
    }

    int analog(Analog channel) const override {
        switch (channel) {
        case Analog::LeftY:
            return m_master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        case Analog::RightY:
            return m_master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        case Analog::LeftX:
            return m_master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        case Analog::RightX:
            return m_master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        }
        return 0;
    }

    bool digital(Digital button) const override {
        switch (button) {
        case Digital::Up:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        case Digital::Down:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        case Digital::Left:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
        case Digital::Right:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        case Digital::A:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        case Digital::B:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        case Digital::X:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        case Digital::Y:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        case Digital::L1:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        case Digital::L2:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        case Digital::R1:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        case Digital::R2:
            return m_master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        }
        return false;
    }

private:
    mutable pros::Controller m_master{pros::E_CONTROLLER_MASTER};
};

} // namespace

Controller &prosController() {
    static ProsController controller;
    return controller;
}

} // namespace io
