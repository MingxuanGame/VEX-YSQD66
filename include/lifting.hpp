#pragma once

#include <cstdint>
#include <optional>

namespace lifting {

// 以启动位置为零，角度减小为上升。
inline constexpr double kUpperLimitDeg = -1530.0;
inline constexpr double kLowerLimitDeg = +1530.0;

struct MotorAngles {
    std::optional<double> port_9_deg;
    std::optional<double> port_10_deg;
};

// 设置双电机保持制动模式，并以当前位置为编码器零点；非阻塞。
void initialize();

// 读取两台电机的累计轴角度（单位：度，正方向与正功率一致）；读取失败时对应值为 std::nullopt。
MotorAngles motor_angles();

// 应用开环功率 [-127, 127]（负值上升）并自动限位减速；非阻塞，未初始化、零功率、触及限位、读数异常或禁用时刹车，返回实际输出功率。
int set_power(int power);

// 按开环功率 [-127, 127] 持续运行指定毫秒数（ms）；阻塞等待完成，触及限位、读数异常或禁用时提前刹车并返回 false。
bool move_for(int power, std::uint32_t duration_ms);

// 按绝对角度（度）同时驱动双电机，以正转速上限（rpm）运行。
// 拒绝未初始化或越界目标；阻塞等待双电机到位，超时（ms）、读数异常、越界或禁用时刹车并返回 false。
bool move_to(double target_deg, std::int32_t max_velocity_rpm, std::uint32_t timeout_ms);

// 立即刹车并保持双电机当前位置；非阻塞。
void stop();

} // namespace lifting
