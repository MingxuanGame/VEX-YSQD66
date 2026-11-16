#include "test_common.hpp"
#include "motion/mecanum_drive_controller.hpp"
#include "motion/mecanum_kinematics.hpp"
#include "motion/mecanum_odometry.hpp"
#include "motion/wheel_velocity_controller.hpp"

#include <array>
#include <cmath>
#include <limits>

namespace {
constexpr double kEps = 1e-6;
}

void test_mecanum() {
    motion::MecanumGeometry geometry;
    geometry.wheelbase_in = 12.0;
    geometry.track_width_in = 12.0;
    geometry.wheel_radius_in = 1.625;
    geometry.gear_ratio = 1.0;
    motion::MecanumKinematics kinematics(geometry);

    // 前进：四轮同向。
    auto forward = kinematics.inverse({10.0, 0.0, 0.0}, 0.0);
    CHECK(std::fabs(forward.ips[0] - 10.0) < kEps);
    CHECK(std::fabs(forward.ips[1] - 10.0) < kEps);
    CHECK(std::fabs(forward.ips[2] - 10.0) < kEps);
    CHECK(std::fabs(forward.ips[3] - 10.0) < kEps);
    auto body = kinematics.forward(forward);
    CHECK(std::fabs(body.vx_ips - 10.0) < kEps);
    CHECK(std::fabs(body.vy_ips) < kEps);
    CHECK(std::fabs(body.omega_dps) < kEps);

    // 左移：左侧反转、右侧正转。
    auto strafe = kinematics.inverse({0.0, 10.0, 0.0}, 0.0);
    CHECK(std::fabs(strafe.ips[0] + 10.0) < kEps);
    CHECK(std::fabs(strafe.ips[1] - 10.0) < kEps);
    CHECK(std::fabs(strafe.ips[2] - 10.0) < kEps);
    CHECK(std::fabs(strafe.ips[3] + 10.0) < kEps);
    body = kinematics.forward(strafe);
    CHECK(std::fabs(body.vx_ips) < kEps);
    CHECK(std::fabs(body.vy_ips - 10.0) < kEps);
    CHECK(std::fabs(body.omega_dps) < kEps);

    // 原地逆时针：左侧后退、右侧前进。
    auto turn = kinematics.inverse({0.0, 0.0, 90.0}, 0.0);
    CHECK(turn.ips[0] < 0.0);
    CHECK(turn.ips[1] > 0.0);
    CHECK(turn.ips[2] < 0.0);
    CHECK(turn.ips[3] > 0.0);
    body = kinematics.forward(turn);
    CHECK(std::fabs(body.vx_ips) < kEps);
    CHECK(std::fabs(body.vy_ips) < kEps);
    CHECK(std::fabs(body.omega_dps - 90.0) < kEps);

    // 归一化保持比例。
    auto limited = kinematics.inverse({100.0, 0.0, 0.0}, 10.0);
    CHECK(limited.normalized);
    CHECK(std::fabs(limited.scale - 0.1) < kEps);
    for (double value : limited.ips)
        CHECK(std::fabs(value - 10.0) < kEps);

    // 轮向取反后正解仍回到原车体速度。
    geometry.wheel_direction[0] = -1;
    kinematics.setGeometry(geometry);
    auto reversed = kinematics.inverse({10.0, 0.0, 0.0}, 0.0);
    CHECK(reversed.ips[0] < 0.0);
    body = kinematics.forward(reversed);
    CHECK(std::fabs(body.vx_ips - 10.0) < kEps);

    // RPM 双向换算。
    auto rpm = kinematics.toWheelRpm(forward);
    auto back = kinematics.fromWheelRpm(rpm);
    for (int i = 0; i < 4; ++i)
        CHECK(std::fabs(back.ips[i] - forward.ips[i]) < kEps);

    // 四轮速度环：输出限幅、非有限目标保护。
    motion::WheelVelocityController wheels;
    std::array<double, 4> target{100.0, 100.0, 100.0, 100.0};
    std::array<double, 4> feedback{0.0, 0.0, 0.0, 0.0};
    auto outputs = wheels.update(target, feedback, 0.01);
    for (int value : outputs) {
        CHECK(value > 0);
        CHECK(value <= 127);
    }

    motion::WheelVelocityGains limited_gains;
    limited_gains.output_limit = 10.0;
    wheels.setGains(limited_gains);
    outputs = wheels.update(target, feedback, 0.01);
    for (int value : outputs)
        CHECK(value <= 10);

    target[0] = std::numeric_limits<double>::quiet_NaN();
    outputs = wheels.update(target, feedback, 0.01);
    for (int value : outputs)
        CHECK(value == 0);

    // 车体速度 -> 四轮速度环：加速度限制、归一化和非法反馈。
    motion::MecanumGeometry drive_geometry = geometry;
    drive_geometry.wheel_direction = {1, 1, 1, 1};
    motion::MecanumDriveConfig drive_config;
    drive_config.max_accel_ips2 = 10.0;
    drive_config.max_alpha_dps2 = 90.0;
    drive_config.max_wheel_speed_ips = 0.5;
    drive_config.max_power_delta_per_update = 127;
    motion::MecanumDriveController drive(drive_geometry, drive_config);
    motion::WheelSpeeds still{};
    auto result = drive.update({100.0, 0.0, 0.0}, still, 0.1);
    CHECK(result.valid);
    CHECK(result.saturated);
    CHECK(std::fabs(result.target_body.vx_ips - 1.0) < kEps);
    for (double value : result.target_wheels.ips)
        CHECK(std::fabs(value) <= 0.5 + kEps);

    motion::WheelSpeeds bad_measured{};
    bad_measured.ips[0] = std::numeric_limits<double>::quiet_NaN();
    result = drive.update({10.0, 0.0, 0.0}, bad_measured, 0.1);
    CHECK(!result.valid);
    for (int value : result.power)
        CHECK(value == 0);

    // 四轮里程：前进、横移、旋转和 IMU 无效回退。
    motion::MecanumKinematics odom_kinematics(drive_geometry);
    motion::MecanumOdometry odometry(drive_geometry);
    motion::WheelSpeeds straight{};
    straight.ips = {10.0, 10.0, 10.0, 10.0};
    odometry.update(straight, 0.0, false, 0.1);
    odometry.update(straight, 0.0, false, 0.1);
    CHECK(odometry.getPose().x_in > kEps);
    CHECK(std::fabs(odometry.getPose().y_in) < kEps);
    motion::WheelSpeeds lateral = odom_kinematics.inverse({0.0, 10.0, 0.0}, 0.0);
    odometry.update(lateral, 0.0, false, 0.1);
    CHECK(odometry.getPose().y_in > kEps);

    motion::MecanumOdometry turn_odometry(drive_geometry);
    auto turn_wheels = odom_kinematics.inverse({0.0, 0.0, 90.0}, 0.0);
    turn_odometry.update(turn_wheels, 0.0, false, 0.1);
    turn_odometry.update(turn_wheels, 0.0, false, 0.1);
    CHECK(turn_odometry.getPose().heading_deg > 0.0);
}
