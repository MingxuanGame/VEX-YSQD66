#pragma once

#include "motion/pid.hpp"

#include <array>

namespace motion {

/**
 * @file wheel_velocity_controller.hpp
 * @brief 四轮独立速度环，为麦轮底盘准备；不直接访问硬件。
 *
 * 每个轮子独立 PID，顺序 FL / FR / BL / BR。四个环共用默认增益，
 * 后续可按轮子单独覆盖。目标/反馈非有限值时整组复位并输出 0。
 */
struct WheelVelocityGains {
    double kp = 2.2;
    double ki = 0.0;
    double kd = 0.02;
    double derivative_filter_tau = 0.02;
    double integral_limit = 100.0;
    double output_limit = 127.0;
};

class WheelVelocityController {
public:
    WheelVelocityController();

    void setGains(const WheelVelocityGains &gains);
    void setWheelGains(int wheel, const WheelVelocityGains &gains);
    const WheelVelocityGains &getGains() const;

    /**
     * @brief 更新四轮速度环。
     * @param target_rpm 目标轮端 RPM，顺序 FL/FR/BL/BR
     * @param current_rpm 实测轮端 RPM
     * @param dt_sec 控制周期
     * @return 四轮功率输出，-127..127
     */
    std::array<int, 4> update(const std::array<double, 4> &target_rpm,
                              const std::array<double, 4> &current_rpm, double dt_sec);

    const std::array<int, 4> &getOutputs() const;
    void reset();

private:
    void applyGains(int wheel);

    std::array<Pid, 4> m_pids;
    std::array<WheelVelocityGains, 4> m_gains;
    std::array<int, 4> m_outputs{};
};

} // namespace motion
