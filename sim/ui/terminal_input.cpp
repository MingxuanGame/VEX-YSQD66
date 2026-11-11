// sim/ui/terminal_input.cpp
#include "sim/ui/terminal_input.hpp"

#ifdef _WIN32
#include <conio.h> // Windows 平台键盘监听
#endif

namespace sim {

#ifdef _WIN32

void TerminalInput::update(io::InputState &in, uint32_t now) {
    // 检查是否有按键按下
    if (_kbhit()) {
        int ch = _getch();

        // --- 模拟摇杆前后左右 (W/S/A/D) ---
        switch (ch) {
        case 'w':
            in.leftY = 127;
            break;
        case 's':
            in.leftY = -127;
            break;
        case 'd':
            in.rightX = 127;
            break;
        case 'a':
            in.rightX = -127;
            break;

        // --- 模拟调参按键 ---
        case 'u':
            in.up.update(true, now);
            break;
        case 'j':
            in.down.update(true, now);
            break;
        case 'h':
            in.left.update(true, now);
            break;
        case 'k':
            in.right.update(true, now);
            break;
        case 'y':
            in.a.update(true, now);
            break;
        case 'n':
            in.b.update(true, now);
            break;

        // 松开逻辑（下次循环如果没有按键，强制清零）
        case 27:
            isConnected = !isConnected;
            break; // ESC 键模拟拔掉遥控器
        }
    } else {
        // 没有按键时，摇杆回中，所有按键强制松开
        in.leftY = 0;
        in.rightX = 0;
        in.up.update(false, now);
        in.down.update(false, now);
        in.left.update(false, now);
        in.right.update(false, now);
        in.a.update(false, now);
        in.b.update(false, now);
    }
}

#else

void TerminalInput::update(io::InputState &in, uint32_t now) {
    // 非 Windows 平台没有 conio；先保持输入为中位，避免链接失败。
    (void)now;
    in.leftY = 0;
    in.rightX = 0;
}

#endif

} // namespace sim
