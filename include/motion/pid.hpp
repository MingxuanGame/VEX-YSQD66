#pragma once

/**
 * @file pid.hpp
 * @brief 通用 PID，带 dt 防护、积分限幅和输出限幅。
 */

namespace motion {

class Pid {
public:
    Pid();

    void setGains(double kp, double ki, double kd);
    void setIntegralLimits(double min, double max);
    void setOutputLimits(double min, double max);

    // dt_sec <= 0 时使用最小 dt，避免除零/积分爆炸。
    double update(double error, double dt_sec);
    void reset();

    double getOutput() const;
    double getIntegral() const;

private:
    double m_kp;
    double m_ki;
    double m_kd;
    double m_integral;
    double m_prev_error;
    double m_output;
    double m_output_min;
    double m_output_max;
    double m_integral_min;
    double m_integral_max;
    bool m_has_prev_error;
};

} // namespace motion
