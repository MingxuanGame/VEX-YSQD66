#pragma once

/**
 * @file battery.hpp
 * @brief 电池电压抽象接口。
 *        运动层用它做电压补偿，不直接依赖 PROS。
 */

namespace robot {

class Battery {
public:
    virtual ~Battery() = default;

    virtual int getVoltageMv() const = 0;
};

} // namespace robot
