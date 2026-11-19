#pragma once

namespace brain_ui {

enum class AutonPreset { RedLeft, RedDown, BlueRight, BlueUp, Count };

// 初始化 Brain 触屏界面（自动赛预设选择）；非阻塞，不驱动任何机构。
void initialize();

// 返回当前在屏幕上选中的自动赛预设。
AutonPreset selected_auton();

// 返回预设显示名称。
const char *auton_name(AutonPreset preset);

// 处理自动赛预设按钮事件。
void tick();

} // namespace brain_ui
