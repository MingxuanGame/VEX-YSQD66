#pragma once

namespace math {

// 增益非负；integral_limit 单位为误差单位·秒，output_limit 为输出绝对值上限。
// 调用方用 constexpr 定义参数；integral_limit 为零时禁用积分累积。
struct PidConfig {
    double kp;
    double ki;
    double kd;
    double integral_limit;
    double output_limit;
};

// 离散 PID：微分按实际时间间隔计算，积分限幅并在输出饱和时阻止同向累积。
class Pid {
public:
    explicit Pid(const PidConfig &config);

    // 清除积分和微分历史；重置后的首次采样不产生微分冲击。
    void reset();
    // 非阻塞；error 为目标减反馈，dt_s 必须是有限正秒数，返回限幅后的输出。
    double update(double error, double dt_s);

private:
    const PidConfig m_config;
    double m_integral = 0.0;
    double m_previous_error = 0.0;
    bool m_has_previous = false;
};

} // namespace math
