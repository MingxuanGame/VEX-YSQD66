#include "test_common.hpp"
#include "sim/core/sim_clock.hpp"
#include "sim/config/sim_config.hpp"
#include "sim/hardware/sim_battery.hpp"
#include "sim/hardware/sim_imu.hpp"
#include "sim/hardware/sim_mechanism.hpp"
#include "sim/hardware/sim_motor.hpp"
#include "sim/physics/chassis_sim.hpp"

#include <cmath>

void test_sim() {
    // ==================== 电池内阻压降 ====================
    sim::Battery::reset();
    CHECK(sim::Battery::getVoltage() == 12800);
    sim::Battery::setVoltage(12000);
    sim::Battery::updateLoad(20000.0); // 20A
    const int loaded = sim::Battery::getVoltage();
    CHECK(loaded < 12000);
    CHECK(loaded > 0);

    sim::Battery::setInternalResistanceMilliohm(0.0);
    CHECK(sim::Battery::getVoltage() == 12000);
    sim::Battery::reset();

    // ==================== 确定性噪声 ====================
    sim::Imu a(1);
    sim::Imu b(1);
    a.setNoiseLevel(0.5);
    b.setNoiseLevel(0.5);
    a.setRandomSeed(12345u);
    b.setRandomSeed(12345u);
    a.reset();
    b.reset();
    CHECK(std::fabs(a.get_heading() - b.get_heading()) < 1e-9);
    CHECK(std::fabs(a.get_rotation() - b.get_rotation()) < 1e-9);
    CHECK(std::fabs(a.get_gyro_rate('z') - b.get_gyro_rate('z')) < 1e-9);

    sim::Motor m1(1, sim::Gearset::Green);
    sim::Motor m2(1, sim::Gearset::Green);
    m1.setRandomSeed(99u);
    m2.setRandomSeed(99u);
    m1.move(100);
    m2.move(100);
    m1.physics_update(0.01, 0.0);
    m2.physics_update(0.01, 0.0);
    CHECK(std::fabs(m1.get_actual_velocity() - m2.get_actual_velocity()) < 1e-9);
    CHECK(std::fabs(m1.get_position() - m2.get_position()) < 1e-9);
    CHECK(std::fabs(m1.get_current_draw() - m2.get_current_draw()) < 1e-9);

    // ==================== 底盘位姿积分 ====================
    sim::Battery::reset();
    sim::Motor lf(1, sim::Gearset::Green);
    sim::Motor lb(2, sim::Gearset::Green);
    sim::Motor rf(3, sim::Gearset::Green, true);
    sim::Motor rb(4, sim::Gearset::Green, true);
    sim::Imu imu(5);
    imu.setNoiseLevel(0.0);
    sim_physics::ChassisSim chassis(&lf, &lb, &rf, &rb, &imu);
    chassis.reset();

    lf.move(80);
    lb.move(80);
    rf.move(80);
    rb.move(80);
    for (int i = 0; i < 40; ++i)
        chassis.step(0.01);
    CHECK(chassis.getPoseXMeters() > 0.0);
    CHECK(std::fabs(chassis.getPoseYMeters()) < 0.05);
    CHECK(chassis.getTotalCurrentMa() > 0.0);
    CHECK(sim::Battery::getVoltage() <= sim::Battery::getOpenCircuitVoltage());

    // ==================== 原地转向位姿 ====================
    chassis.reset();
    lf.move(80);
    lb.move(80);
    rf.move(-80);
    rb.move(-80);
    for (int i = 0; i < 40; ++i)
        chassis.step(0.01);
    CHECK(std::fabs(chassis.getPoseHeadingDeg()) > 5.0);

    // ==================== 级联抬升 / 夹爪 / 推板 ====================
    sim::SimMechanism mechanism;
    mechanism.liftTo(0.60, 127);
    for (int i = 0; i < 5000; ++i)
        mechanism.update(0.001);
    CHECK(std::fabs(mechanism.getLiftHeightM() - 0.60) < 0.02);
    CHECK(mechanism.getCascadeStage() > 1.5);

    mechanism.clampClose();
    for (int i = 0; i < 1000; ++i)
        mechanism.update(0.001);
    CHECK(mechanism.getClampAngleDeg() > 140.0);

    mechanism.clampOpen();
    for (int i = 0; i < 1000; ++i)
        mechanism.update(0.001);
    CHECK(mechanism.getClampAngleDeg() < 40.0);

    mechanism.pushExtend();
    for (int i = 0; i < 500; ++i)
        mechanism.update(0.001);
    CHECK(mechanism.getPushPosition() > 0.9);
    mechanism.pushRetract();
    for (int i = 0; i < 500; ++i)
        mechanism.update(0.001);
    CHECK(mechanism.getPushPosition() < 0.1);

    mechanism.liftManual(80);
    for (int i = 0; i < 1000; ++i)
        mechanism.update(0.001);
    CHECK(mechanism.getLiftHeightM() < 0.90);
    CHECK(mechanism.getCurrentMa() > 0.0);

    sim::Battery::reset();
}
