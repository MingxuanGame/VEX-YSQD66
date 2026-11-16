#pragma once
#include "motion/pid.hpp"

/**
 * @file velocity_controller.hpp
 * @brief 左右轮速度闭环。输入目标/当前 RPM，输出 -127~127 电压。
 */

namespace motion {

class VelocityController {
public:
    VelocityController();

    void update(double target_rpm_left, double target_rpm_right, double current_rpm_left,
                double current_rpm_right, double dt_sec);

    int getLeftOutput() const;
    int getRightOutput() const;
    void reset();

    void setGains(double kp, double ki, double kd);
    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);
    void setDerivativeFilterTau(double tau_sec);

private:
    void applyGains();

    Pid m_left_pid;
    Pid m_right_pid;
    int m_left_output;
    int m_right_output;
    double m_kp;
    double m_ki;
    double m_kd;
};

} // namespace motion
