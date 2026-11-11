#include "sim/hardware/sim_motor.hpp"
#include "sim/hardware/sim_battery.hpp"
#include <cmath>
#include <algorithm>

namespace sim {

Motor::Motor(int port, Gearset gear, bool reversed)
    : m_port(port), m_gear(gear), m_reversed(reversed), m_cmd_voltage(0), m_velocity_rpm(0),
      m_position_ticks(0), m_current_draw_ma(0), m_temp_c(25.0), m_brake_mode(BrakeMode::Coast),
      m_rng(static_cast<uint32_t>(port) * 2654435761u + 1u) {

    // 根据齿轮箱设定物理参数
    switch (gear) {
    case Gearset::Green:
        m_max_rpm = 200.0;
        m_stall_torque_nm = 2.1;
        break;
    case Gearset::Blue:
        m_max_rpm = 600.0;
        m_stall_torque_nm = 0.7;
        break;
    case Gearset::Red:
        m_max_rpm = 100.0;
        m_stall_torque_nm = 4.2;
        break;
    }
    m_nominal_voltage_v = 12.0;
}

void Motor::move(int voltage) {
    m_cmd_voltage = std::clamp(voltage, -127, 127);
}

void Motor::move_voltage(int voltage_mv) {
    // 把毫伏映射到 -127 ~ 127 的指令范围
    int val = static_cast<int>((voltage_mv / 12000.0) * 127.0);
    m_cmd_voltage = std::clamp(val, -127, 127);
}

void Motor::brake() {
    m_cmd_voltage = 0;
}

void Motor::set_brake_mode(BrakeMode mode) {
    m_brake_mode = mode;
}

void Motor::tare_position() {
    m_position_ticks = 0;
}

void Motor::reset() {
    m_cmd_voltage = 0;
    m_velocity_rpm = 0.0;
    m_position_ticks = 0.0;
    m_current_draw_ma = 0.0;
    m_temp_c = 25.0;
    m_brake_mode = BrakeMode::Coast;
}

void Motor::setRandomSeed(uint32_t seed) {
    m_rng.seed(seed);
}

double Motor::get_position() const {
    return m_position_ticks;
}

double Motor::get_actual_velocity() const {
    return m_velocity_rpm;
}

double Motor::get_current_draw() const {
    return m_current_draw_ma;
}

double Motor::get_temperature() const {
    return m_temp_c;
}

void Motor::physics_update(double dt_sec, double external_load_nm) {
    // ==================== 电压补偿与损耗模拟 ====================
    // 1. 获取当前真实的虚拟电池电压（V）
    double battery_voltage_v = sim::Battery::getVoltage() / 1000.0;

    // 2. 计算施加在电机上的实际物理电压
    // 这里完美模拟了：当电池电量低时，即使指令给 127，施加的物理电压也会变低，导致扭矩下降。
    // 注意：仿真按“指令方向”积分速度和位置。reversed 只影响真实接线，
    // 不应让 get_actual_velocity() 在正指令时返回负值，否则底盘左右速度会互相抵消。
    double applied_voltage_v = (m_cmd_voltage / 127.0) * battery_voltage_v;

    // 3. 计算反电动势（BEMF）
    // 反电动势由电机自身转速决定，满速时接近额定电压，而不是随电池电压变化。
    double bemf_v = (m_velocity_rpm / m_max_rpm) * m_nominal_voltage_v;
    double effective_voltage_v = applied_voltage_v - bemf_v;

    // 4. 计算电机当前能输出的扭矩
    // 扭矩与有效电压成正比。如果电池没电，同样的指令输出的扭矩会大打折扣。
    double motor_torque_nm = (effective_voltage_v / m_nominal_voltage_v) * m_stall_torque_nm;

    // 刹车模式：如果给 0 电压且开启了刹车，扭矩强行抵抗运动
    if (m_cmd_voltage == 0 &&
        (m_brake_mode == BrakeMode::Brake || m_brake_mode == BrakeMode::Hold)) {
        motor_torque_nm = (m_velocity_rpm > 0) ? -2.0 : (m_velocity_rpm < 0 ? 2.0 : 0);
    }

    // 5. 简化物理模型：用机械时间常数加速逼近目标速度
    double target_rpm = (m_cmd_voltage / 127.0) * m_max_rpm;

    // 如果外部阻力（比如撞墙）大于电机最大扭矩，速度直接归零（堵转）
    if (std::abs(external_load_nm) > m_stall_torque_nm && std::abs(m_cmd_voltage) > 0) {
        m_velocity_rpm = 0;
        // 堵转电流：拉满并加一点确定性白噪声
        m_current_draw_ma = 3500.0 + static_cast<int>(m_rng.next() % 200);
    } else {
        // 惯性逼近（机械时间常数，模拟电机和车的重量导致的加速延迟）
        double alpha = std::min(1.0, dt_sec / kMechanicalTimeConst);
        m_velocity_rpm += (target_rpm - m_velocity_rpm) * alpha;

        // 模拟正常电流：与电机当前输出的扭矩（以及电压）成正比
        // V5 电机空载约 200mA，满载约 2500mA
        double current_factor = std::abs(motor_torque_nm) / m_stall_torque_nm;
        m_current_draw_ma = 200.0 + current_factor * 2300.0;
        // 加一点白噪声，让仿真更真实
        m_current_draw_ma += static_cast<int>(m_rng.next() % 50) - 25;
    }

    // 6. 积分编码器位置
    // V5 智能电机转一圈是 360 度（tick），按当前转速累加
    m_position_ticks += (m_velocity_rpm / 60.0) * 360.0 * dt_sec;

    // 7. 温度模拟（长时间满功率或堵转会升温）
    double heat = (std::abs(m_current_draw_ma) / 3000.0) * 0.5 * dt_sec;
    double cooling = (m_temp_c - 25.0) * 0.01 * dt_sec;
    m_temp_c += heat - cooling;
}

} // namespace sim
