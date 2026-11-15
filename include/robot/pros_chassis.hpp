#pragma once

#include "robot/chassis.hpp"

namespace robot {

/**
 * @file pros_chassis.hpp
 * @brief 把现有 chassis::set_power 适配为 robot::Chassis。
 *
 * 该适配器不拥有电机，也不重复构造设备；底盘设备仍由 chassis 模块唯一持有。
 * setDrive 接收 -127..127，直接交给 chassis::set_power。
 *
 * 当前 chassis 接口没有暴露电流、速度或编码器反馈，因此反馈 getter 返回 0；
 * ControlLoop 的防推挤和靠编码器的运动闭环暂不可用，不能把这些 0 当成实测值。
 */
class ProsChassis final : public Chassis {
public:
    ProsChassis() = default;

    void setDrive(int left, int right) override;
    void stop(bool brake = true) override;

    double getLeftCurrent() const override;
    double getRightCurrent() const override;
    double getLeftVelocity() const override;
    double getRightVelocity() const override;
    double getLeftPosition() const override;
    double getRightPosition() const override;

    /**
     * @brief 当前无编码器反馈，tare 为空操作；等待底盘模块补回反馈后实现。
     */
    void tarePosition() override;

    bool isHealthy() const override;

private:
    bool m_healthy = true;
};

} // namespace robot
