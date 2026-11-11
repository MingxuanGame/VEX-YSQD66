#include "test_common.hpp"
#include "collision/collision_detector.hpp"
#include "control/control_loop.hpp"
#include "io/controller.hpp"
#include "motion/motion_tuning.hpp"
#include "sim/compat/sim_robot.hpp"
#include "sim/core/sim_clock.hpp"
#include "sim/hardware/sim_battery.hpp"
#include "sim/hardware/sim_imu.hpp"
#include "sim/hardware/sim_motor.hpp"
#include "sim/physics/chassis_sim.hpp"
#include "tuning/param_apply.hpp"
#include "tuning/params_manager.hpp"

#include <cmath>

namespace {

sim_physics::ChassisSim *g_physics = nullptr;

void physicsTick(double dt_sec) {
    if (g_physics)
        g_physics->step(dt_sec);
}

struct IntegrationController : io::Controller {
    bool connected = true;
    int left_y = 0;
    bool a = false;
    bool b = false;
    bool y = false;

    bool isConnected() const override {
        return connected;
    }

    int analog(io::Analog channel) const override {
        return (channel == io::Analog::LeftY) ? left_y : 0;
    }

    bool digital(io::Digital button) const override {
        if (button == io::Digital::A)
            return a;
        if (button == io::Digital::B)
            return b;
        if (button == io::Digital::Y)
            return y;
        return false;
    }
};

} // namespace

void test_integration() {
    sim::init_clock();
    sim::Battery::setVoltage(12800);

    sim::Motor left_front(1, sim::Gearset::Green);
    sim::Motor left_back(2, sim::Gearset::Green);
    sim::Motor right_front(3, sim::Gearset::Green, true);
    sim::Motor right_back(4, sim::Gearset::Green, true);
    sim::Imu imu(5);
    imu.setNoiseLevel(0.0);

    sim_physics::ChassisSim physics(&left_front, &left_back, &right_front, &right_back, &imu);
    g_physics = &physics;
    sim::register_physics_tick(physicsTick);

    sim::SimChassis chassis(&left_front, &left_back, &right_front, &right_back);
    sim::SimImu sim_imu(&imu);
    sim::SimBattery battery;
    IntegrationController controller;

    control::ControlLoop loop(controller, chassis, sim_imu, &battery);

    tuning::ParamsManager params;
    motion::MotionTuning motion_tuning(loop.getMotionEngine());
    CHECK(motion_tuning.registerAll(params) == 3);
    CHECK(motion_tuning.bindAll(params) == 3);
    loop.setParamsManager(&params);

    auto step = [&]() {
        sim::delay(control::kControlPeriodMs);
        loop.update(sim::millis());
    };

    // 1. 手动驾驶：摇杆驱动真实仿真底盘。
    controller.left_y = 50;
    for (int i = 0; i < 50; ++i)
        step();
    CHECK(loop.getMode() == control::Mode::Drive);
    const double driven_position = chassis.getLeftPosition();
    CHECK(std::fabs(driven_position) > 1.0);

    // 2. 断联：必须进入 SafeWait，并停住底盘。
    controller.connected = false;
    controller.left_y = 0;
    step();
    CHECK(loop.getMode() == control::Mode::SafeWait);
    const double stopped_position = chassis.getLeftPosition();
    for (int i = 0; i < 50; ++i)
        step();
    // 急停是刹车，不是瞬移归零；1 秒后轮速应被压到接近 0。
    CHECK(std::fabs(chassis.getLeftPosition() - stopped_position) < 100.0);
    CHECK(std::fabs(chassis.getLeftVelocity()) < 2.0);

    // 3. 恢复连接 + A：回到手动驾驶。
    controller.connected = true;
    controller.a = true;
    step();
    controller.a = false;
    CHECK(loop.getMode() == control::Mode::Drive);

    // 4. 长按 Y 进入 TUNE，改参数，再按 B 退出并下发。
    controller.left_y = 0;
    controller.y = true;
    for (int i = 0; i < 30; ++i)
        step();
    CHECK(loop.getMode() == control::Mode::Tune);

    auto *kp = params.getParam("vel_kp");
    CHECK(kp != nullptr);
    if (kp)
        kp->incParam();
    controller.y = false;
    controller.b = true;
    step();
    controller.b = false;
    CHECK(loop.getMode() == control::Mode::Drive);
    if (kp) {
        kp->decParam();
        tuning::ParamApply::applyAllDirty(params);
    }

    // 5. 自动赛：MotionEngine 直行 + 转向，全部走同一个 ControlLoop。
    loop.startAuto();
    motion::MotionEngine &motion = loop.getMotionEngine();
    motion.resetPose();
    motion.startDrive(12.0, 60);

    for (int i = 0; i < 3000 && motion.isBusy(); ++i)
        step();
    CHECK(motion.getState() == motion::State::Idle);
    CHECK(std::fabs(motion.getPose().x - 12.0) < 1.0);

    motion.resetPose();
    motion.startTurn(90.0, 60);
    for (int i = 0; i < 3000 && motion.isBusy(); ++i)
        step();
    CHECK(motion.getState() == motion::State::Idle);
    const double heading_deg = motion.getPose().heading * 180.0 / 3.141592653589793;
    CHECK(heading_deg > 89.8 && heading_deg < 90.2);

    loop.endAuto();
    CHECK(loop.getMode() == control::Mode::Drive);

    // 6. 自动赛结束后手动驾驶仍然可用。
    controller.left_y = 30;
    step();
    CHECK(loop.getMode() == control::Mode::Drive);

    // 7. 物理墙 + 碰撞检测：堵转电流高、速度为零，应判为 Deadzone。
    physics.setCollisionObject(sim_physics::CollisionObject::Wall);
    controller.left_y = 100;
    for (int i = 0; i < 30; ++i)
        step();
    collision::CollisionDetector detector(chassis, sim_imu);
    CHECK(detector.isTriggered());
    CHECK(detector.detect() == collision::CollisionType::Deadzone);
    physics.setCollisionObject(sim_physics::CollisionObject::None);

    controller.left_y = 0;
    for (int i = 0; i < 10; ++i)
        step();

    tuning::ParamApply::clear();
    g_physics = nullptr;
}
