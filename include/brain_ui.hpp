#pragma once

namespace brain_ui {

enum class AutonPreset { RedLeft, RedDown, BlueRight, BlueUp, Count };

// 初始化 Brain 触屏界面（自动赛预设选择 + 升降限位测量）；非阻塞，不驱动任何机构。
void initialize();

// 返回当前在屏幕上选中的自动赛预设。
AutonPreset selected_auton();

// 返回预设显示名称。
const char *auton_name(AutonPreset preset);

// 重置升降限位标定与点动状态（模式切换或禁用时调用）。
void reset_lift_calibration();

// 当前是否处于限位点动标定状态（未激活限位或已采集下限待采上限）。
bool lift_jog_mode();

// 周期刷新 Brain 屏幕、处理按钮事件并在点动模式下响应 X/B 键（建议 20ms 调用一次）。
// driver_enabled 为 true 表示处于启用的手动控制阶段。
void tick(bool driver_enabled);

} // namespace brain_ui
