#pragma once

namespace chassis {

// 初始化四电机底盘（蓝齿轮、度编码器、刹车模式），非阻塞，不驱动车辆。
void initialize();

// 停止输出、保持刹车模式并清零编码器参考（不执行机械归零）。
void reset_state();

// 应用左右侧开环功率 [-127, 127]，按补偿偏移限幅；禁用或自动阶段下保持停止。
void set_power(int left_power, int right_power);

// 立即刹车停止四个驱动电机。
void stop();

} // namespace chassis
