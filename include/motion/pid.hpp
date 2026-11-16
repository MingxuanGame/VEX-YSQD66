#pragma once

/**
 * @file pid.hpp
 * @brief 通用 PID：带 dt 防护、积分限幅、条件积分、微分滤波和输出限幅。
 */

namespace motion {

class Pid {
public:
    Pid();

    void setGains(double kp, double ki, double kd);
    void setIntegralLimits(double min, double max);
    void setOutputLimits(double min, double max);

    /**
     * @brief 设置微分低通时间常数，单位秒；<=0 时不滤波。
     *
     * 速度目标突变时直接对误差微分会产生 derivative kick，低通后更适合
     * 有齿轮背隙和结构振动的底盘。
     */
    void setDerivativeFilterTau(double tau_sec);

    /**
     * @brief 更新 PID。
     * @param error 目标值 - 当前值
     * @param dt_sec 控制周期，<=0 时使用最小周期
     * @param feedforward 可选前馈，直接叠加到输出
     * @return 限幅后的输出
     */
    double update(double error, double dt_sec, double feedforward = 0.0);

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
    double m_derivative_filter_tau;
    double m_filtered_derivative;
    bool m_has_prev_error;
    bool m_has_filtered_derivative;
};

} // namespace motion
