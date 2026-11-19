#pragma once

namespace lift {

// 初始化抬升电机；只设置刹车，不动作。
void initialize();

// 手动调用一次：空闲 -> 上升，上升 -> 下降，下降 -> 上升。
void toggle();

// 适合主循环：按下边沿触发 toggle，按住不会反复翻转。
void update(bool pressed);

// 手动功率，范围 -127..127；会退出 toggle 状态。
void set_power(int power);

// 刹车停止，回到空闲。
void stop();

} // namespace lift
