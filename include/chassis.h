#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/motors.h"

namespace chassis {

// 初始化四电机差速底盘（双侧直轮）、端口 5 的 IMU 与 LemLib 里程计；停车校准，不驱动车辆。
void initialize();

// 取消所有 LemLib 运动并立即刹车停止四个驱动电机。
void stop();

// 设置底盘电机刹车模式。
void set_brake_mode(pros::motor_brake_mode_e mode);

// 手动 Arcade 混控控制 [-127, 127]，使用 LemLib 指数驾驶曲线；禁用或自动阶段下保持停止。
void arcade(int throttle, int turn);

// 获取底盘 LemLib 控制器引用（供自动赛路线调用 moveToPoint / moveToPose / turnToHeading 等）。
lemlib::Chassis &controller();

// 获取当前里程计位姿（x/y 单位英寸，theta 单位度）。
lemlib::Pose pose();

} // namespace chassis
