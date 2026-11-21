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

struct TargetAngles {
    double port_9_deg;
    double port_10_deg;
};

// 设置双电机保持制动模式，并以启动位置为编码器零点。
void initialize();

// 读取两台电机的累计轴角度，正方向与正功率一致；读取失败时对应值为空。
MotorAngles motor_angles();

// 取消回零后应用手动功率 [-127, 127]，负值上升；零功率、限位或读数失效时刹车。
int set_power(int power);

// 在手动循环中调用。start 为按下沿，目标为两台电机的启动零位。
// 非阻塞，处理回零时返回 true，空闲返回 false。
bool update_return_to_zero(bool start);

// 自动模式下按功率运行指定毫秒数，触及限位或中断时刹车并返回 false。
bool move_for(int power, std::uint32_t duration_ms);

// 按屏幕的 P9、P10 顺序接收绝对角度（度），以正转速上限（rpm）运行。
// allow_driver 为 true 时允许在手动阶段执行，供 opcontrol 里的按 A 自动路线入口调用。
// 拒绝越界目标；阻塞等待双电机到位，超时（ms）、读数异常、越界或退出时刹车。
bool move_to(TargetAngles target_deg, std::int32_t max_velocity_rpm, std::uint32_t timeout_ms,
             bool allow_driver);

// 取消回零，刹车并保持当前位置。
void stop();

} // namespace lifting
