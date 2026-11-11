#pragma once
#include "io/controller.hpp"

/**
 * @file pros_controller.hpp
 * @brief 真车手柄工厂声明。
 *        头文件不包含 pros/*，具体实现放在 src/io/pros_controller.cpp。
 */

namespace io {

Controller &prosController();

} // namespace io
