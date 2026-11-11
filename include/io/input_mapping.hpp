#pragma once
#include "io/input_state.hpp"

/**
 * @file input_mapping.hpp
 * @brief 摇杆信号处理与组合键。
 */

namespace io {

// 组合键判断（传入两个按键的取值函数）
bool comboPressed(const InputState &in, bool (*btn1)(const InputState &),
                  bool (*btn2)(const InputState &));

/**
 * @brief 摇杆死区与指数曲线处理
 * @param raw 原生摇杆值 (-127 ~ 127)
 * @param deadzone 死区大小
 * @param expo 指数曲线系数 (1.0 为线性)
 * @return 处理后的摇杆值
 * @note 为什么需要？防止摇杆中位漂移导致底盘自己乱跑，同时让微调更精准。
 */
int processJoystick(int raw, int deadzone, double expo);

} // namespace io
