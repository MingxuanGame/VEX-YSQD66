#pragma once

#include "auton/kit.hpp"

namespace chassis {
// 初始化唯一的四电机底盘，设置蓝齿轮/度编码器/刹车；非阻塞，不驱动车辆。
void initialize();
// 读取唯一手柄，进行限幅 Arcade 混控；非阻塞，断联/禁用时停止。
void move();
// 左右侧电压，单位 mV，限幅 ±12000；非阻塞，禁用或 SDK 失败返回 false 并停止。
bool set_voltage(int left_mv, int right_mv);
// 非阻塞停止四个驱动电机，不执行机械归零。
void stop();
// 非阻塞读取端口 3、4、-1、-2 的实际反馈；缺失/错误标记 valid=false。
std::array<auton::MotorReading, 4> readings();
} // namespace chassis
