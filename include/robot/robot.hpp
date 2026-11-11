#pragma once
#include "robot/chassis.hpp"
#include "robot/mechanism.hpp"

/**
 * @file robot.hpp
 * @brief 机器人硬件抽象门面。
 *        持有底盘、机构、IMU 的抽象指针。主控只需与 Robot 交互，不关心底层具体实现。
 */

namespace robot {

class Robot {
public:
    /**
     * @brief 注入具体的硬件实现
     * @param chassis 底盘实例
     * @param mechanism 机构实例
     * @note 构造函数接收指针，意味着在 main.cpp 里你可以随时替换不同的机械实现，
     *       或者在没有实车的情况下注入一个 Mock 对象用于测试。
     */
    Robot(Chassis *chassis, Mechanism *mechanism);
    ~Robot() = default;

    Chassis *getChassis();
    const Chassis *getChassis() const;
    Mechanism *getMechanism();
    const Mechanism *getMechanism() const;
    bool isValid() const;

private:
    Chassis *m_chassis;
    Mechanism *m_mechanism;
};

} // namespace robot
