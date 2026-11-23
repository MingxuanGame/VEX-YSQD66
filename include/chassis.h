#pragma once

#include <optional>

#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/motors.h"

namespace chassis {

// 初始化四电机差速底盘（端口 1-4）、端口 5 IMU 与 LemLib 里程计，设置刹车模式并执行 IMU 校准，
// 同时将里程计复位至 (0, 0, 0)；阻塞至校准完成，不驱动车辆。
void initialize();

// 取消所有 LemLib 运动并立即刹车停止四个驱动电机；非阻塞。
void stop();

// 设置左右底盘电机组的刹车模式；非阻塞。
void set_brake_mode(pros::motor_brake_mode_e mode);

// 手动 Arcade 混控控制：throttle（前后油门）与 turn（转向）范围均为 [-127, 127]，
// 经 LemLib 指数驾驶曲线映射；禁用或自动阶段下自动刹车停止，非阻塞。
void arcade(int throttle, int turn);

// 获取底盘 LemLib 控制器引用（供自动赛路线调用 moveToPoint / moveToPose / turnToHeading 等）。
lemlib::Chassis &controller();

// 获取当前里程计位姿（x/y 单位：英寸，theta 单位：度）。
lemlib::Pose pose();

// 读取 IMU 累计转角（单位：度，顺时针为正）；校准中或读取失败时返回 std::nullopt。
std::optional<double> imu_rotation_deg();

} // namespace chassis
