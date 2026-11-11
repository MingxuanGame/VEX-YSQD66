#ifndef ENABLE_SIMULATOR
#define ENABLE_SIMULATOR
#endif
#include "compat/pros_compat.hpp"
#include "compat/sim_robot.hpp"
#include "motion/motion_engine.hpp"
#include "motion/motion_tuning.hpp"
#include "sim/physics/chassis_sim.hpp"
#include "sim/core/sim_clock.hpp"
#include "tuning/tuning.hpp"
#include <cstdio>

pros::Motor left_front(1, pros::E_MOTOR_GEARSET_18);
pros::Motor left_back(2, pros::E_MOTOR_GEARSET_18);
pros::Motor right_front(3, pros::E_MOTOR_GEARSET_18, true);
pros::Motor right_back(4, pros::E_MOTOR_GEARSET_18, true);
pros::Imu imu(5);

sim_physics::ChassisSim chassis(&left_front, &left_back, &right_front, &right_back, &imu);

void physics_tick(double dt_sec) {
    chassis.step(dt_sec);
}

int main() {
    sim::init_clock();
    sim::register_physics_tick(physics_tick);
    imu.setNoiseLevel(0.0);

    sim::SimChassis sim_chassis(&left_front, &left_back, &right_front, &right_back);
    sim::SimImu sim_imu(&imu);
    sim::SimBattery sim_battery;
    motion::MotionEngine engine(sim_chassis, sim_imu, &sim_battery);

    // 把速度环 PID 注册到调参系统，再绑定到当前 MotionEngine。
    tuning::ParamsManager params;
    motion::MotionTuning motion_tuning(engine);
    const int registered = motion_tuning.registerAll(params);
    const int bound = motion_tuning.bindAll(params);

    auto *kp = params.getParam("vel_kp");
    auto *ki = params.getParam("vel_ki");
    auto *kd = params.getParam("vel_kd");

    auto set_param = [](tuning::TunableParam *p, double value) {
        if (!p)
            return;
        char line[128];
        std::snprintf(line, sizeof(line), "%s|1|%.6f|0|2|1|0.000000|10.000000|0", p->getParamName(),
                      value);
        p->deserialize(line);
    };

    // 仿真扫出来的 PID 参数：kp=2.2, ki=0, kd=0.02。
    set_param(kp, 2.2);
    set_param(ki, 0.0);
    set_param(kd, 0.02);
    tuning::ParamApply::applyAllDirty(params);
    engine.setAngleTolerance(0.2);

    std::printf("tuning: registered=%d bound=%d kp=%.2f ki=%.2f kd=%.2f\n", registered, bound,
                kp ? kp->getDoubleValue() : -1.0, ki ? ki->getDoubleValue() : -1.0,
                kd ? kd->getDoubleValue() : -1.0);

    std::printf("=== 运动控制仿真 ===\n");
    engine.startDrive(24.0, 80);
    int guard = 0;
    while (engine.isBusy() && guard++ < 3000) {
        sim::delay(10);
        engine.update(sim::millis(), true);
    }

    const motion::Pose pose_after_drive = engine.getPose();
    std::printf("state=%d pos=(%.2f, %.2f) heading=%.2f deg\n", static_cast<int>(engine.getState()),
                pose_after_drive.x, pose_after_drive.y,
                pose_after_drive.heading * 180.0 / 3.141592653589793);

    engine.startTurn(90.0, 60);
    guard = 0;
    while (engine.isBusy() && guard++ < 3000) {
        sim::delay(10);
        engine.update(sim::millis(), true);
    }

    const motion::Pose pose_after_turn = engine.getPose();
    std::printf("state=%d pos=(%.2f, %.2f) heading=%.2f deg\n", static_cast<int>(engine.getState()),
                pose_after_turn.x, pose_after_turn.y,
                pose_after_turn.heading * 180.0 / 3.141592653589793);

    std::printf("=== 运动控制仿真结束 ===\n");
    return 0;
}
