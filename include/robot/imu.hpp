#pragma once

/**
 * @file imu.hpp
 * @brief IMU 抽象接口。
 *        碰撞检测只关心“航向/角速度”，不关心底层是 PROS IMU 还是仿真 IMU。
 *        这样真车与仿真可以共用同一份检测逻辑。
 */

namespace robot {

class Imu {
public:
    virtual ~Imu() = default;

    virtual double getHeading() const = 0;   // 0 ~ 360 度
    virtual double getRotation() const = 0;  // 累积角度
    virtual double getGyroRateZ() const = 0; // Z 轴角速度（度/秒）
};

} // namespace robot
