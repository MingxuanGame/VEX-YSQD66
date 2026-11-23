#pragma once

namespace brain_ui {

enum class AutonPreset { RedLeft, RedDown, BlueRight, BlueUp, Count };
enum class AutonPlan { Plan1, Plan2, Plan3, Count };

// 初始化 Brain 触屏界面（自动赛预设/方案选择与状态读数）；非阻塞，不驱动任何机构。
void initialize();

// 返回当前在屏幕上选中的自动赛预设，开机默认 RedLeft。
AutonPreset selected_auton();

// 返回当前在屏幕上选中的自动赛方案，开机默认 Plan1。
AutonPlan selected_plan();

// 返回指定预设的显示名称字符串（无效枚举时返回空串）。
const char *auton_name(AutonPreset preset);

// 轮询并应用屏幕按钮选择事件，并按周期刷新传感器读数；非阻塞，应在主循环中周期调用。
void tick();

} // namespace brain_ui
