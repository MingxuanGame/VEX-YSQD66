#include "test_common.hpp"
#include "motion/chassis_kinematics.hpp"
#include "motion/motion_engine.hpp"
#include "motion/motion_tuning.hpp"
#include "motion/pid.hpp"
#include "robot/battery.hpp"
#include "robot/chassis.hpp"
#include "robot/imu.hpp"
#include "sim/core/sim_clock.hpp"
#include "tuning/tuning.hpp"
#include "vxmath/units.hpp"

namespace {

struct MotionFakeChassis : robot::Chassis {
    double left_position = 0.0;
    double right_position = 0.0;
    double left_velocity = 0.0;
    double right_velocity = 0.0;
    int drive_left = 0;
    int drive_right = 0;

    void setDrive(int left, int right) override {
        drive_left = left;
        drive_right = right;
    }

    void stop(bool) override {
        drive_left = 0;
        drive_right = 0;
        left_velocity = 0.0;
        right_velocity = 0.0;
    }

    double getLeftCurrent() const override {
        return 0.0;
    }

    double getRightCurrent() const override {
        return 0.0;
    }

    double getLeftVelocity() const override {
        return left_velocity;
    }

    double getRightVelocity() const override {
        return right_velocity;
    }

    double getLeftPosition() const override {
        return left_position;
    }

    double getRightPosition() const override {
        return right_position;
    }

    void tarePosition() override {
        left_position = 0.0;
        right_position = 0.0;
    }

    void advance(double dt_sec) {
        // Simplified test model: wheel speed follows the previous output with lag.
        left_velocity += (drive_left - left_velocity) * 0.3;
        right_velocity += (drive_right - right_velocity) * 0.3;
        const double ticks_left = (left_velocity / 60.0) * 360.0 * dt_sec;
        const double ticks_right = (right_velocity / 60.0) * 360.0 * dt_sec;
        left_position += ticks_left;
        right_position += ticks_right;
    }
};

struct MotionFakeImu : robot::Imu {
    double heading = 0.0;

    double getHeading() const override {
        return heading;
    }

    double getRotation() const override {
        return heading;
    }

    double getGyroRateZ() const override {
        return 0.0;
    }
};

struct MotionFakeBattery : robot::Battery {
    int voltage_mv = 12000;

    int getVoltageMv() const override {
        return voltage_mv;
    }
};

} // namespace

void test_motion() {
    sim::init_clock();

    // Odometry: 360 ticks should equal one wheel circumference.
    MotionFakeChassis chassis;
    MotionFakeImu imu;
    motion::ChassisKinematics kinematics;
    kinematics.update(&chassis, &imu, 0.01);
    chassis.left_position = 360.0;
    chassis.right_position = 360.0;
    kinematics.update(&chassis, &imu, 0.01);
    const double expected = vxmath::ticksToInches(360.0);
    CHECK(kinematics.getPose().x > expected - 0.01);
    CHECK(kinematics.getPose().x < expected + 0.01);
    CHECK(kinematics.getPose().y > -0.01);
    CHECK(kinematics.getPose().y < 0.01);
    kinematics.reset();
    CHECK(kinematics.getPose().x == 0.0);
    CHECK(kinematics.getPose().y == 0.0);
    CHECK(!kinematics.isInitialized());

    // PID output/dt/integral limits.
    motion::Pid pid;
    pid.setGains(10.0, 1.0, 0.0);
    pid.setIntegralLimits(-100.0, 100.0);
    pid.setOutputLimits(-127.0, 127.0);
    CHECK(pid.update(1000.0, 0.0) <= 127.0);
    CHECK(pid.update(-1000.0, 0.0) >= -127.0);
    CHECK(pid.getIntegral() <= 100.0);
    CHECK(pid.getIntegral() >= -100.0);

    // State machine: busy after start; disconnect must stop.
    MotionFakeBattery battery;
    motion::MotionEngine engine(chassis, imu, &battery);

    // PID parameter registration and binding.
    tuning::ParamsManager params;
    motion::MotionTuning motion_tuning(engine);
    CHECK(motion_tuning.registerAll(params) == 3);
    CHECK(motion_tuning.bindAll(params) == 3);

    auto *kp = params.getParam("vel_kp");
    auto *ki = params.getParam("vel_ki");
    CHECK(kp != nullptr);
    CHECK(ki != nullptr);
    if (kp) {
        kp->moveDigitPos(tuning::MoveDirection::Right);
        kp->incParam(); // 2.2 -> 2.3
    }
    if (ki) {
        ki->moveDigitPos(tuning::MoveDirection::Right);
        ki->incParam(); // 0.0 -> 0.1
    }
    tuning::ParamApply::applyAllDirty(params);
    CHECK(kp && kp->getDoubleValue() > 2.29 && kp->getDoubleValue() < 2.31);
    CHECK(ki && ki->getDoubleValue() > 0.09 && ki->getDoubleValue() < 0.11);

    // Drive a short distance: periodic updates should end in Idle.
    engine.startDrive(10.0, 50);
    CHECK(engine.isBusy());
    CHECK(engine.getState() == motion::State::Driving);

    engine.update(sim::millis(), false);
    CHECK(!engine.isBusy());
    CHECK(engine.getState() == motion::State::Idle);
    CHECK(chassis.drive_left == 0);
    CHECK(chassis.drive_right == 0);

    // Turn a small angle, enter Turning; stop() returns to Idle.
    engine.startTurn(45.0, 50);
    CHECK(engine.getState() == motion::State::Turning);
    engine.stop();
    CHECK(engine.getState() == motion::State::Idle);

    // Drive a short distance: periodic updates should end in Idle.
    engine.setVelocityKp(2.2);
    engine.setVelocityKi(0.0);
    engine.setVelocityKd(0.02);
    engine.startDrive(10.0, 50);
    for (int i = 0; i < 1000 && engine.isBusy(); ++i) {
        sim::delay(10);
        chassis.advance(0.01);
        engine.update(sim::millis(), true);
    }
    CHECK(engine.getState() == motion::State::Idle);

    // Clear bindings before local tuning context goes out of scope.
    tuning::ParamApply::clear();
}
