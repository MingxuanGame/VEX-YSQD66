#pragma once

#include <cstdint>
#include <optional>

namespace lifting {

struct MotorAngles {
    std::optional<double> port_9_deg;
    std::optional<double> port_10_deg;
};

struct TargetAngles {
    double port_9_deg;
    double port_10_deg;
};

// 设置双电机保持制动模式，并以启动位置为编码器零点。
void initialize();

// 读取两台电机的累计轴角度，正方向与正功率一致；读取失败时对应值为空。
MotorAngles motor_angles();

// 启用手动时下发功率 [-127, 127]；零功率刹车。
int set_power(int power);

// 启用自动时按功率运行指定毫秒数，结束或退出模式时刹车。
bool move_for(int power, std::uint32_t duration_ms);

// 自动模式下按屏幕的 P9、P10 顺序接收绝对角度（度），以正转速上限（rpm）运行。
// 阻塞等待双电机到位，超时（ms）、读数异常或退出自动模式时刹车并返回 false。
bool move_to(TargetAngles target_deg, std::int32_t max_velocity_rpm, std::uint32_t timeout_ms);

// 刹车并保持当前位置。
void stop();

} // namespace lifting
