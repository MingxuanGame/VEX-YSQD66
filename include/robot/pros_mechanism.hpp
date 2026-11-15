#pragma once

#include "robot/mechanism.hpp"
#include "pros/motors.hpp"

namespace robot {

/**
 * @file pros_mechanism.hpp
 * @brief 机构适配器：升降使用一个智能电机，夹爪/推板通过注入回调接入。
 *
 * 当前硬件方案未定，夹爪可能是电机、舵机或气缸，推板也可能只是无动力件。
 * 因此不在适配器里猜测端口和传动，而是把非升降动作做成静态回调，由 main
 * 在硬件确定后提供。所有回调都不得阻塞；到位等待由上层超时逻辑负责。
 */
struct ProsMechanismConfig {
    pros::Motor *lift_motor = nullptr;

    // 夹爪：优先使用 clamp_open/clamp_close；没有时退回 clamp_set_angle。
    void (*clamp_open)() = nullptr;
    void (*clamp_close)() = nullptr;
    void (*clamp_set_angle)(int angle_deg) = nullptr;
    // 仅用于 clamp_set_angle 回退路径；角度必须按实际机构重新标定。
    int clamp_open_deg = 0;
    int clamp_close_deg = 100;

    // 推板：无动力件可以为空。
    void (*push_extend)() = nullptr;
    void (*push_retract)() = nullptr;

    // 可选：机构总电流反馈，单位 mA；为空时只读升降电机电流。
    double (*current_ma)() = nullptr;

    // 可选：升降限位和总故障回调；返回 true 表示已触发/存在故障。
    bool (*lift_upper_limit)() = nullptr;
    bool (*lift_lower_limit)() = nullptr;
    bool (*fault)() = nullptr;
};

class ProsMechanism final : public Mechanism {
public:
    ProsMechanism() = default;
    explicit ProsMechanism(const ProsMechanismConfig &config);

    void setConfig(const ProsMechanismConfig &config);

    void liftTo(double pos, int max_speed = 100) override;
    void liftManual(int speed) override;
    void liftHold() override;

    void clampOpen() override;
    void clampClose() override;
    void clampSetAngle(int angle) override;

    void pushExtend() override;
    void pushRetract() override;

    double getCurrentMa() const override;
    bool isHealthy() const override;

private:
    ProsMechanismConfig m_config;
};

} // namespace robot
