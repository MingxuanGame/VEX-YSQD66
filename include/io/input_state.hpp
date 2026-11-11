#pragma once
#include "io/button.hpp"

/**
 * @file input_state.hpp
 * @brief 输入状态快照。
 *        将一帧的所有数据打包传递，避免各模块重复读取硬件。
 */

namespace io {

struct InputState {
    // ==================== 安全检测 ====================
    // 为什么需要这个？如果在比赛中遥控器没电或被干扰断联，
    // 原生 API 会返回 0。如果没有此标志位，底盘会以为摇杆回中了，
    // 导致机器人瞬间失控急刹，甚至因为惯性翻车。
    bool isConnected = true;

    // ==================== 摇杆数据 ====================
    int leftY = 0;
    int rightY = 0;
    int leftX = 0;
    int rightX = 0;

    // ==================== 按键状态 ====================
    Button up, down, left, right;
    Button a, b, x, y;
    Button l1, l2, r1, r2;
};

} // namespace io
