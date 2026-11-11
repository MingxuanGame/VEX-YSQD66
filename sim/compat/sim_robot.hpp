#pragma once
#include "robot/battery.hpp"
#include "robot/chassis.hpp"
#include "robot/imu.hpp"

/**
 * @file sim_robot.hpp
 * @brief 把仿真的电机/IMU 适配成 robot::Chassis / robot::Imu。
 *        实现放在 sim/compat/sim_robot.cpp，头文件只做声明。
 */

namespace sim {

class Motor;
class Imu;

class SimChassis : public robot::Chassis {
public:
    SimChassis(Motor *lf, Motor *lb, Motor *rf, Motor *rb);

    void setDrive(int left, int right) override;
    void stop(bool brake = true) override;

    double getLeftCurrent() const override;
    double getRightCurrent() const override;
    double getLeftVelocity() const override;
    double getRightVelocity() const override;
    double getLeftPosition() const override;
    double getRightPosition() const override;
    void tarePosition() override;

private:
    Motor *m_lf;
    Motor *m_lb;
    Motor *m_rf;
    Motor *m_rb;
};

class SimImu : public robot::Imu {
public:
    explicit SimImu(sim::Imu *imu);

    double getHeading() const override;
    double getRotation() const override;
    double getGyroRateZ() const override;

private:
    sim::Imu *m_imu;
};

class SimBattery : public robot::Battery {
public:
    int getVoltageMv() const override;
};

} // namespace sim
