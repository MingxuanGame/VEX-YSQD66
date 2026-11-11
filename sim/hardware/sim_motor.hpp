#pragma once
#include <cstdint>
#include "sim/core/sim_rng.hpp"

/**
 * @file sim_motor.hpp
 * @brief 虚拟 V5 智能电机。
 *        基于直流电机物理模型（含惯性、反电动势、扭矩负载）实现。
 *        接口与 pros::Motor 保持高度一致，支持无缝替换。
 */
namespace sim {

// 齿轮箱配置：对应真实的减速比
enum class Gearset {
    Green = 18, // 18:1，最高转速 200 RPM，扭矩 2.1 Nm
    Blue = 6,   // 6:1，最高转速 600 RPM，扭矩 0.7 Nm
    Red = 36    // 36:1，最高转速 100 RPM，扭矩 4.2 Nm
};

enum class BrakeMode { Coast, Brake, Hold };

class Motor {
public:
    /**
     * @brief 构造虚拟电机
     * @param port 端口号（虚拟的，用于日志打印）
     * @param gear 齿轮箱类型（决定最大转速和扭矩）
     * @param reversed 是否反向。仅用于保持和真机接口一致；仿真按“指令方向”
     *        积分速度/位置，这样正指令的 get_actual_velocity() 始终为正。
     */
    Motor(int port, Gearset gear = Gearset::Green, bool reversed = false);

    // ==================== 控制接口 ====================
    void move(int voltage);            // 功率控制 -127 ~ 127
    void move_voltage(int voltage_mv); // 电压控制
    void brake();                      // 刹车（速度快速归零）
    void set_brake_mode(BrakeMode mode);
    void tare_position();              // 清零编码器
    void reset();                      // 复位速度/位置/电流/温度
    void setRandomSeed(uint32_t seed); // 固定噪声序列，便于复现实验

    // ==================== 传感器读取接口 ====================
    double get_position() const;        // 编码器位置（tick）
    double get_actual_velocity() const; // 实际转速（RPM）
    double get_current_draw() const;    // 实际电流（mA）
    double get_temperature() const;     // 模拟温度

    // ==================== 物理引擎调用接口 ====================
    /**
     * @brief 物理步进更新（由物理引擎每 dt 秒调用）
     * @param dt_sec 步进时间（秒）
     * @param external_load_nm 来自底盘/环境的阻力扭矩（牛米）
     * @note 外部阻力如果大于电机当前能提供的最大扭矩，电机将堵转。
     */
    void physics_update(double dt_sec, double external_load_nm);

private:
    // 电机固有属性
    int m_port;
    Gearset m_gear;
    bool m_reversed;

    // 齿轮箱物理常数（真实 V5 数据近似值）
    double m_max_rpm;           // 空载最高转速
    double m_stall_torque_nm;   // 堵转扭矩
    double m_nominal_voltage_v; // 额定电压（通常是 12V）

    // 电机运行状态
    int m_cmd_voltage;        // 当前指令电压 -127 ~ 127
    double m_velocity_rpm;    // 当前实际转速
    double m_position_ticks;  // 累计编码器刻度
    double m_current_draw_ma; // 当前电流
    double m_temp_c;          // 模拟温度
    BrakeMode m_brake_mode;
    XorShift32 m_rng;

    // 电机机械常数（用于惯性积分）
    const double kMechanicalTimeConst = 0.1; // 机械时间常数（秒），决定加速快慢
};
} // namespace sim
