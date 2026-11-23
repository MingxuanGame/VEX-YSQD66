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

    // 返回手柄当前是否已连接；非阻塞。
    virtual bool isConnected() const = 0;

    // 读取指定摇杆轴的瞬时值，范围 [-127, 127]；非阻塞。
    virtual int analog(Analog channel) const = 0;

    // 读取指定按键当前是否处于按下状态；非阻塞。
    virtual bool digital(Digital button) const = 0;
};

} // namespace io
