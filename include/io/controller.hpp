#pragma once

/**
 * @file controller.hpp
 * @brief 手柄抽象接口。业务层通过该接口读取输入，与 PROS 具体硬件解耦。
 */

namespace io {

enum class Analog { LeftY, RightY, LeftX, RightX };

enum class Digital { Up, Down, Left, Right, A, B, X, Y, L1, L2, R1, R2 };

class Controller {
public:
    virtual ~Controller() = default;

    virtual bool isConnected() const = 0;
    virtual int analog(Analog channel) const = 0;
    virtual bool digital(Digital button) const = 0;
};

} // namespace io
