#pragma once

namespace chassis {
// 初始化唯一的四电机底盘，设置蓝齿轮/度编码器/刹车；非阻塞，不驱动车辆。
void initialize();
// 应用左右组件功率 [-127,127]，非阻塞；禁用/自动模式/SDK 失败返回 false 并停止。
// 手动及宏共用同一入口，调用方负责控制权与场控连接检查。
bool set_power(int left_power, int right_power);
// 非阻塞停止四个驱动电机，不执行机械归零。
void stop();
} // namespace chassis
