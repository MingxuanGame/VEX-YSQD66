#pragma once
#include "io/controller.hpp"

// 真车手柄单例工厂。头文件不直接引入 pros 头文件，实现位于 src/io/pros_controller.cpp。
namespace io {

// 返回绑定 PROS 主手柄的 Controller 单例引用；首次调用时构造，非阻塞。
Controller &prosController();

} // namespace io
