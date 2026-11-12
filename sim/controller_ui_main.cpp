#include "control/control_loop.hpp"
#include "io/input_state.hpp"
#include "motion/motion_tuning.hpp"
#include "sim/compat/sim_robot.hpp"
#include "sim/config/sim_config.hpp"
#include "sim/core/sim_clock.hpp"
#include "sim/hardware/sim_battery.hpp"
#include "sim/hardware/sim_imu.hpp"
#include "sim/hardware/sim_mechanism.hpp"
#include "sim/hardware/sim_motor.hpp"
#include "sim/physics/chassis_sim.hpp"
#include "sim/ui/controller_ui.hpp"
#include "tuning/param_apply.hpp"
#include "tuning/params_manager.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace {

sim_physics::ChassisSim *g_physics = nullptr;

void physicsTick(double dt_sec) {
    if (g_physics)
        g_physics->step(dt_sec);
}

constexpr double kFieldSizeM = 3.6576;
constexpr double kGoalPositions[9][2] = {{0.655, 0.345}, {0.510, 0.495}, {0.340, 0.655},
                                         {0.190, 0.820}, {0.350, 0.345}, {0.500, 0.350},
                                         {0.660, 0.495}, {0.345, 0.495}, {0.500, 0.655}};

void updateFieldCollision(sim_physics::ChassisSim &physics, const sim::ControllerUi &ui) {
    const double x = physics.getPoseXMeters();
    const double y = physics.getPoseYMeters();
    const bool outside = (x < 0.25 || x > kFieldSizeM - 0.25 || y < 0.25 || y > kFieldSizeM - 0.25);

    // 顶到场地边界时形成“墙”；操作手反向时解除，允许倒车回场。
    if (outside && ui.analog(io::Analog::LeftY) >= 0) {
        physics.setCollisionObject(sim_physics::CollisionObject::Wall);
        return;
    }

    // 得分座是实体，仿真里给它们近似 0.20m 的碰撞半径。
    for (const auto &goal : kGoalPositions) {
        const double gx = goal[0] * kFieldSizeM;
        const double gy = (1.0 - goal[1]) * kFieldSizeM;
        const double dx = x - gx;
        const double dy = y - gy;
        if (dx * dx + dy * dy < 0.04) {
            physics.setCollisionObject(sim_physics::CollisionObject::Cone);
            return;
        }
    }
    physics.setCollisionObject(sim_physics::CollisionObject::None);
}

const char *modeName(control::Mode mode) {
    switch (mode) {
    case control::Mode::Drive:
        return "DRIVE";
    case control::Mode::Tune:
        return "TUNE";
    case control::Mode::Auto:
        return "AUTO";
    case control::Mode::SafeWait:
        return "SAFE_WAIT";
    default:
        return "UNKNOWN";
    }
}

} // namespace

int main() {
    sim::ControllerUi ui;
    if (!ui.open()) {
        std::printf("Controller UI is only available on Windows.\n");
        return 1;
    }

    sim::Battery::reset();
    sim::init_clock();

    sim::Motor lf(1, sim::Gearset::Green);
    sim::Motor lb(2, sim::Gearset::Green);
    sim::Motor rf(3, sim::Gearset::Green, true);
    sim::Motor rb(4, sim::Gearset::Green, true);
    sim::Imu imu(5);
    imu.setNoiseLevel(0.05);
    imu.setRandomSeed(12345u);

    sim_physics::ChassisSim physics(&lf, &lb, &rf, &rb, &imu);
    physics.setPose(0.50, 0.12, 90.0); // 底侧起始位，面朝场地中心
    g_physics = &physics;
    sim::register_physics_tick(physicsTick);

    sim::SimChassis chassis(&lf, &lb, &rf, &rb);
    sim::SimImu sim_imu(&imu);
    sim::SimBattery battery;
    const sim::SimConfig sim_config = sim::defaultSimConfig();
    sim::SimMechanism mechanism(sim_config);

    control::ControlLoop loop(ui, chassis, sim_imu, &battery);
    loop.setMechanism(&mechanism);

    tuning::ParamsManager params;
    motion::MotionTuning motion_tuning(loop.getMotionEngine());
    motion_tuning.registerAll(params);
    motion_tuning.bindAll(params);
    loop.setParamsManager(&params);

    std::printf("VEX V5 simulator opened. PLACEMENT: drag robot, Enter/START; R restart; "
                "Arrows = 8-way joystick; ESC quit.\n");

    auto resetSimulation = [&]() {
        physics.reset();
        mechanism.reset();
        sim::Battery::reset();
        sim::init_clock();
        sim::register_physics_tick(physicsTick);
        loop.reset();
        ui.resetField();
        ui.beginPlacement();
        ui.setTuneMode(false);
    };

    auto startSimulation = [&](const sim::PlacementPose &pose) {
        physics.reset();
        physics.setPose(pose.x_m, pose.y_m, pose.heading_deg);
        mechanism.reset();
        sim::Battery::reset();
        sim::init_clock();
        sim::register_physics_tick(physicsTick);
        loop.reset();
        ui.resetField();
        ui.setTuneMode(false);
    };

    ui.beginPlacement();

    while (ui.isOpen()) {
        ui.pump();
        if (!ui.isOpen())
            break;

        if (ui.consumeRestartRequest()) {
            resetSimulation();
            continue;
        }

        if (ui.consumeStartRequest()) {
            startSimulation(ui.getPlacementPose());
            continue;
        }

        if (ui.isPlacementActive()) {
            const sim::PlacementPose pose = ui.getPlacementPose();
            sim::FieldState field;
            field.valid = true;
            field.x_m = pose.x_m;
            field.y_m = pose.y_m;
            field.heading_deg = pose.heading_deg;
            field.mode = static_cast<int>(control::Mode::Drive);
            ui.setFieldState(field);
            ui.setStatusText("PLACEMENT", "Drag robot, then ENTER or click START");
            ui.render();
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            continue;
        }

        // 固定 1ms 物理步进，渲染约 60Hz。
        updateFieldCollision(physics, ui);
        for (int i = 0; i < 16; ++i) {
            sim::advance_time(0.001);
            mechanism.update(0.001);
        }
        loop.update(sim::millis());
        ui.setTuneMode(loop.getMode() == control::Mode::Tune);

        sim::FieldState field;
        field.valid = true;
        field.x_m = physics.getPoseXMeters();
        field.y_m = physics.getPoseYMeters();
        field.heading_deg = std::fmod(physics.getPoseHeadingDeg(), 360.0);
        if (field.heading_deg > 180.0)
            field.heading_deg -= 360.0;
        if (field.heading_deg < -180.0)
            field.heading_deg += 360.0;
        field.velocity_mps = physics.getChassisVelocityMps();
        field.current_ma = physics.getTotalCurrentMa();
        field.lift_height_m = mechanism.getLiftHeightM();
        field.clamp_angle_deg = mechanism.getClampAngleDeg();
        field.push_position = mechanism.getPushPosition();
        field.mode = static_cast<int>(loop.getMode());
        ui.setFieldState(field);

        char line0[80];
        char line1[80];
        const tuning::TunableParam *param = params.getSelectedParam();
        if (loop.getMode() == control::Mode::Tune && param) {
            if (param->getParamType() == tuning::ParamType::Double) {
                std::snprintf(line0, sizeof(line0), "TUNE %s = %.2f", param->getParamName(),
                              param->getDoubleValue());
            } else {
                std::snprintf(line0, sizeof(line0), "TUNE %s = %d", param->getParamName(),
                              param->getIntValue());
            }
            std::snprintf(line1, sizeof(line1),
                          "Up/Down select  X/Y adjust  A save  B exit  |  L1+W/S/Q/E jog");
        } else {
            std::snprintf(line0, sizeof(line0), "%s  Arrows = 8-way joystick",
                          modeName(loop.getMode()));
            std::snprintf(line1, sizeof(line1), "R = restart   ESC = quit");
        }
        ui.setStatusText(line0, line1);
        ui.render();

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    tuning::ParamApply::clear();
    g_physics = nullptr;
    return 0;
}
