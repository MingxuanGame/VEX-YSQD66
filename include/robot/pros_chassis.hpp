#pragma once

#include "robot/chassis.hpp"

namespace robot {

/**
 * @file pros_chassis.hpp
 * @brief 把现有 chassis::set_voltage/readings 适配为 robot::Chassis。
 *
 * 该适配器不拥有电机，也不重复构造设备；底盘设备仍由 chassis 模块唯一持有。
 * setDrive 接收 -127..127，内部换算成 ±12000mV 后交给 chassis::set_voltage。
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
     * @brief 软件零点，不直接清零电机编码器。
     *
     * chassis 模块目前没有暴露物理 tare 接口；这里记录当前读数作为零参考，
     * 让 MotionEngine 可以使用相对位移。需要物理清零时由底盘模块补充接口。
     */
    void tarePosition() override;

private:
    double m_left_zero_deg = 0.0;
    double m_right_zero_deg = 0.0;
};

} // namespace robot
