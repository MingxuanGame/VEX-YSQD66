#pragma once
#include "io/controller.hpp"

// 真车手柄工厂声明。头文件不直接引入 pros 头文件，实现放在 src/io/pros_controller.cpp。
namespace io {

Controller &prosController();

} // namespace io
