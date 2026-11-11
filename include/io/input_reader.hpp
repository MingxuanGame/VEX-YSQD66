#pragma once
#include <cstdint>
#include "io/controller.hpp"
#include "io/input_state.hpp"

/**
 * @file input_reader.hpp
 * @brief 读取手柄输入。只依赖 io::Controller，不包含任何 pros/ 头文件。
 */

namespace io {

/**
 * @brief 读取手柄数据并填充 InputState
 * @param master 手柄接口
 * @param in 输入状态结构体
 * @param now 当前时间戳（统一从主循环传入，保证所有按键时间基准一致）
 */
void readInput(Controller &master, InputState &in, uint32_t now);

} // namespace io
