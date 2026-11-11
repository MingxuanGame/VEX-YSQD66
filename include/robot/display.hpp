#pragma once

/**
 * @file display.hpp
 * @brief 屏幕抽象接口。
 *        ControlLoop 只依赖这个接口，真机用 pros::lcd，仿真/测试用 FakeDisplay。
 */

namespace robot {

class Display {
public:
    virtual ~Display() = default;

    virtual void clearLine(int line) = 0;
    virtual void printLine(int line, const char *text) = 0;
};

} // namespace robot
