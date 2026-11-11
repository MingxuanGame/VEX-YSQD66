// sim/ui/terminal_input.hpp
#pragma once
#include "io/input_state.hpp"
#include <cstdint>

/**
 * @file terminal_input.hpp
 * @brief 终端虚拟遥控器。
 *        通过监听 PC 键盘，模拟出 InputState 给上层逻辑调用。
 */
namespace sim {

class TerminalInput {
public:
    void update(io::InputState &in, uint32_t now);

    // 模拟连接状态
    bool isConnected = true;
};

} // namespace sim
