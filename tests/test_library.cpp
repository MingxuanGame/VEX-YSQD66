#include "test_common.hpp"
#include "collision/collision_tuning.hpp"
#include "io/button.hpp"
#include "io/input_mapping.hpp"
#include "io/input_reader.hpp"
#include "robot/chassis.hpp"
#include "robot/imu.hpp"
#include "robot/mechanism.hpp"
#include "robot/robot.hpp"
#include "tuning/params_manager.hpp"
#include "tuning/param_validate.hpp"
#include "vxmath/filters.hpp"
#include "vxmath/math_utils.hpp"
#include "vxmath/speed_profile.hpp"
#include "vxmath/units.hpp"

#include <cmath>

namespace {

struct LibraryChassis : robot::Chassis {
    void setDrive(int, int) override {}

    void stop(bool) override {}

    double getLeftCurrent() const override {
        return 0.0;
    }

    double getRightCurrent() const override {
        return 0.0;
    }

    double getLeftVelocity() const override {
        return 0.0;
    }

    double getRightVelocity() const override {
        return 0.0;
    }

    double getLeftPosition() const override {
        return 0.0;
    }

    double getRightPosition() const override {
        return 0.0;
    }

    void tarePosition() override {}
};

struct LibraryImu : robot::Imu {
    double getHeading() const override {
        return 0.0;
    }

    double getRotation() const override {
        return 0.0;
    }

    double getGyroRateZ() const override {
        return 0.0;
    }
};

struct LibraryMechanism : robot::Mechanism {
    void liftTo(double, int) override {}

    void liftManual(int) override {}

    void liftHold() override {}

    void clampOpen() override {}

    void clampClose() override {}

    void clampSetAngle(int) override {}

    void pushExtend() override {}

    void pushRetract() override {}
};

struct LibraryController : io::Controller {
    bool connected = true;
    int left_y = 20;
    int right_x = -30;
    bool y = true;
    bool l1 = true;

    bool isConnected() const override {
        return connected;
    }

    int analog(io::Analog channel) const override {
        if (channel == io::Analog::LeftY)
            return left_y;
        if (channel == io::Analog::RightX)
            return right_x;
        return 0;
    }

    bool digital(io::Digital button) const override {
        if (button == io::Digital::Y)
            return y;
        if (button == io::Digital::L1)
            return l1;
        return false;
    }
};

bool buttonY(const io::InputState &in) {
    return in.y.pressed();
}

bool buttonA(const io::InputState &in) {
    return in.a.pressed();
}

} // namespace

void test_library() {
    // ==================== vxmath ====================
    CHECK(vxmath::clampVal(5, 0, 3) == 3);
    CHECK(vxmath::clampVal(-1, 0, 3) == 0);
    CHECK(vxmath::isValidDouble(1.0));
    CHECK(!vxmath::isValidDouble(std::nan("")));
    CHECK(vxmath::isValidAngle(359.0));
    CHECK(!vxmath::isValidAngle(10001.0));
    CHECK(vxmath::isValidGyroRate(2000.0));
    CHECK(!vxmath::isValidGyroRate(3000.0));

    CHECK(std::fabs(vxmath::restrictAngle(370.0) - 10.0) < 1e-9);
    CHECK(std::fabs(vxmath::restrictAngle(-10.0) - 350.0) < 1e-9);
    CHECK(std::fabs(vxmath::getError(350.0, 10.0) - 20.0) < 1e-9);
    CHECK(std::fabs(vxmath::getError(10.0, 350.0) + 20.0) < 1e-9);
    CHECK(std::fabs(vxmath::normalizeAngle180(190.0) + 170.0) < 1e-9);
    CHECK(std::fabs(vxmath::safeGetError(std::nan(""), 0.0, 42.0) - 42.0) < 1e-9);
    CHECK(vxmath::angleInTolerance(0.5, 1.0));
    CHECK(!vxmath::angleInTolerance(1.5, 1.0));
    CHECK(std::fabs(vxmath::lerp(0.0, 10.0, 0.25) - 2.5) < 1e-9);
    CHECK(vxmath::sign(-3) == -1);
    CHECK(vxmath::sign(0) == 0);

    CHECK(std::fabs(vxmath::ticksToInches(vxmath::inchesToTicks(10.0)) - 10.0) < 1e-9);
    CHECK(vxmath::rpmToInchesPerSec(600.0) > 0.0);
    CHECK(vxmath::inchesPerSecToRpm(vxmath::rpmToInchesPerSec(600.0)) - 600.0 < 1e-9);
    CHECK(vxmath::clampMotorOutput(300) == 127);
    CHECK(vxmath::clampMotorOutput(-300) == -127);
    CHECK(vxmath::percentToMotor(100.0) == 127);
    CHECK(std::fabs(vxmath::motorToPercent(127) - 100.0) < 1e-9);

    vxmath::MovingAverage<3> average;
    CHECK(std::fabs(average.update(1.0) - 1.0) < 1e-9);
    CHECK(std::fabs(average.update(2.0) - 1.5) < 1e-9);
    CHECK(std::fabs(average.update(3.0) - 2.0) < 1e-9);
    average.reset();
    CHECK(std::fabs(average.update(9.0) - 9.0) < 1e-9);

    vxmath::LowPass low_pass(0.5);
    CHECK(std::fabs(low_pass.update(10.0) - 10.0) < 1e-9);
    CHECK(std::fabs(low_pass.update(20.0) - 15.0) < 1e-9);
    low_pass.reset();
    CHECK(std::fabs(low_pass.update(4.0) - 4.0) < 1e-9);

    CHECK(vxmath::applyDeadzone(4, 5) == 0);
    CHECK(vxmath::applyDeadzone(5, 5) == 5);
    CHECK(vxmath::expoCurve(127, 1.0) == 127);
    CHECK(vxmath::expoCurve(127, 0.0) == 127);

    vxmath::Hysteresis hysteresis(10.0, 2.0);
    CHECK(!hysteresis.update(11.0));
    CHECK(hysteresis.update(13.0));
    CHECK(hysteresis.update(11.0));
    CHECK(!hysteresis.update(7.0));

    vxmath::TrapezoidProfile profile;
    profile.configure(10.0, 20.0, 40.0);
    CHECK(std::fabs(profile.getVelocity(0.0)) < 1e-9);
    CHECK(profile.getVelocity(5.0) > 0.0);
    CHECK(profile.getVelocity(10.0) < 1e-9);
    CHECK(profile.isDone(10.0));
    CHECK(std::fabs(profile.getVelocityRatio(0.0)) < 1e-9);

    // ==================== io ====================
    io::Button button;
    button.update(true, 100);
    CHECK(button.justPressed());
    CHECK(button.pressed());
    button.update(true, 600);
    CHECK(button.longPressed());
    CHECK(button.pressDuration(600) == 500);
    CHECK(button.repeated(100, 650));
    button.update(false, 700);
    CHECK(button.justReleased());
    CHECK(!button.pressed());

    CHECK(io::processJoystick(0, 5, 1.3) == 0);
    CHECK(io::processJoystick(4, 5, 1.3) == 0);
    CHECK(io::processJoystick(10, 5, 1.0) > 0);
    CHECK(io::processJoystick(-10, 5, 1.0) < 0);
    CHECK(io::processJoystick(127, 0, 2.0) <= 127);
    CHECK(io::processJoystick(-127, 0, 2.0) >= -127);

    LibraryController controller;
    io::InputState input;
    io::readInput(controller, input, 100);
    CHECK(input.isConnected);
    CHECK(input.leftY == 20);
    CHECK(input.rightX == -30);
    CHECK(input.y.pressed());
    CHECK(input.l1.pressed());
    CHECK(io::comboPressed(input, buttonY, buttonA) == false);

    controller.connected = false;
    io::readInput(controller, input, 120);
    CHECK(!input.isConnected);
    CHECK(input.leftY == 0);
    CHECK(input.rightX == 0);

    // ==================== robot ====================
    LibraryChassis chassis;
    LibraryMechanism mechanism;
    robot::Robot robot(&chassis, &mechanism);
    CHECK(robot.isValid());
    CHECK(robot.getChassis() == &chassis);
    CHECK(robot.getMechanism() == &mechanism);
    robot::Robot empty_robot(nullptr, nullptr);
    CHECK(!empty_robot.isValid());
    CHECK(empty_robot.getChassis() == nullptr);
    CHECK(empty_robot.getMechanism() == nullptr);

    // ==================== tuning ====================
    tuning::ParamsManager params;
    CHECK(params.addParam("kp", 1.0, 0.0, 10.0, tuning::ParamType::Double,
                          tuning::ParamGroup::PIDGroup, -2, 2) >= 0);
    CHECK(params.addParam("current", 2500, 0, 5000, tuning::ParamType::Int,
                          tuning::ParamGroup::CollisionGroup, 0, 2) >= 0);
    CHECK(params.getSize() == 2);
    CHECK(params.getParam("kp") != nullptr);
    CHECK(params.getParam("missing") == nullptr);
    CHECK(tuning::ParamValidate::validateAll(params) == 0);

    params.setGroup(tuning::ParamGroup::PIDGroup);
    CHECK(params.getSelectedIndex() == 0);
    params.selectNext();
    CHECK(params.getSelectedIndex() == 0);
    params.setGroup(tuning::ParamGroup::AllGroup);
    params.selectNext();
    CHECK(params.getSelectedIndex() == 1);
    params.selectPrev();
    CHECK(params.getSelectedIndex() == 0);

    tuning::TunableParam *kp = params.getParam("kp");
    if (kp) {
        kp->incParam(); // 1.0 -> 2.0
        CHECK(kp->isDirty());
        kp->saveParam();
        kp->incParam(); // 2.0 -> 3.0
        kp->undoVariance();
        CHECK(kp->getDoubleValue() < 2.5);
        kp->redoVariance();
        CHECK(kp->getDoubleValue() > 2.5);
    }

    tuning::ParamsManager collision_params;
    CHECK(collision::CollisionTuning::registerAll(collision_params) == 6);
    CHECK(collision_params.getParam("coll_current") != nullptr);
    CHECK(collision_params.getParam("esc_dist") != nullptr);
}
