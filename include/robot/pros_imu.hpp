#pragma once

#include "robot/imu.hpp"
#include "pros/imu.hpp"

namespace robot {

/**
 * @file pros_imu.hpp
 * @brief 将 pros::Imu 适配为 robot::Imu。
 *
 * IMU 的构造、校准和端口检查由 main/初始化流程负责；本类只做数值转换，
 * 不重复创建设备。传入空指针时返回 0，调用方应把“未配置”视为无效传感器。
 */
class ProsImu final : public Imu {
public:
    explicit ProsImu(pros::Imu *imu = nullptr);

    double getHeading() const override;
    double getRotation() const override;
    double getGyroRateZ() const override;

private:
    pros::Imu *m_imu;
};

} // namespace robot
