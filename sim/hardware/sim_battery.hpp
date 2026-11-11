#pragma once

/**
 * @file sim_battery.hpp
 * @brief 虚拟电池（含内阻和负载压降）。
 *
 * 只设置开路电压不够：真实 V5 电池在大电流下会明显掉压。这里用
 * V_terminal = V_oc - I * R_internal 近似，驱动电机和电压补偿代码都能
 * 感受到“低电量 + 大电流”的工况。
 */
namespace sim {

class Battery {
public:
    // 当前端电压（mV），已经包含负载压降。
    static int getVoltage();

    // 开路电压（mV）。
    static int getOpenCircuitVoltage();

    // 设置开路电压（mV），默认满电 12.8V。
    static void setVoltage(int mv);

    // 设置内阻（mΩ），默认 20mΩ。
    static void setInternalResistanceMilliohm(double mohm);

    // 由底盘物理层每步更新总负载电流（mA）。
    static void updateLoad(double current_ma);

    // 恢复默认电池状态。
    static void reset();

private:
    static int m_open_circuit_mv;
    static double m_load_current_ma;
    static double m_internal_resistance_mohm;
};

} // namespace sim
