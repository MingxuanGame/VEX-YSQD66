#pragma once

namespace brain_ui {

// 初始化 Brain LLEMU 升降限位界面与三按钮回调；非阻塞，不驱动任何机构。
void initialize();

// 重置升降限位标定与点动状态（模式切换或禁用时调用）。
void reset_lift_calibration();

// 当前是否处于限位点动标定状态（未激活限位或已采集下限待采上限）。
bool lift_jog_mode();

// 周期刷新 Brain 限位界面、处理按钮动作并在点动模式下响应 X/B 键（建议 20ms 调用一次）。
// driver_enabled 为 true 表示处于启用的手动控制阶段。
void tick(bool driver_enabled);

} // namespace brain_ui
