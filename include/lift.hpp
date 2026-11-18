#pragma once

namespace lift {

// 初始化抬升电机；只设置刹车，不驱动。
void initialize();

// 手动功率，范围 -127..127；禁用/自动阶段自动停止。
void set_power(int power);

// 立即刹车停止。
void stop();

} // namespace lift
