#pragma once

#include "lifting.hpp"
#include "macro/engine.hpp"

#include <vector>

namespace macrorec {

enum class View { Home, List, Checkpoints, Running, Result, Calibration, LiftLimits };

enum class TouchAction {
    None,
    NewRecording,
    OpenList,
    OpenCalibration,
    OpenLiftLimits,
    MarkLiftLower,
    MarkLiftUpper,
    UseSavedLiftLimits,
    Select,
    Resume,
    Previous,
    Next,
    Home,
    Stop,
    Retry
};

struct Touch {
    TouchAction action = TouchAction::None;
    int id = 0;
};

// 初始化 Brain 屏幕配色与触摸基准；使用 pros::screen 独占绘图（不与 LVGL 共用）。
void initialize_screen();

// 重置触摸按压计数，避免模式切换后旧触摸事件误触发。
void reset_touch();

// 轮询触摸按下沿并返回对应菜单动作；ids 在 Checkpoints 页为检查点帧数，其他页为宏编号。
Touch poll_touch(View view, const std::vector<int> &ids, std::size_t page, bool retry_save);

// 按需刷新当前视图；Running 页每 100ms 刷新计时与进度。
void draw_screen(View view, const std::vector<int> &ids, std::size_t page, const Engine &engine,
                 std::uint32_t now_ms, int selected_id, const char *message,
                 const DebugInfo &debug_info, bool retry_save, bool force = false);

// 显示实时电机轴角度及采集的端点，常态刷新周期 100ms。
void draw_lift_limits(std::uint32_t now_ms, const std::optional<lifting::Position> &position,
                      const std::optional<lifting::Position> &lower, const char *status,
                      bool force);

} // namespace macrorec
