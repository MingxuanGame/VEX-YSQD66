#pragma once

#include "macro/engine.hpp"

#include <vector>

namespace macrorec {

enum class View { Home, List, Checkpoints, Running, Result };
enum class TouchAction {
    None,
    NewRecording,
    OpenList,
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

// 无后台任务；在当前竞赛回调中使用 PROS screen 独占绘图，不与 LLEMU/LVGL 共用。
// 非阻塞（仅 SDK 屏幕调用），不得驱动执行器或访问 SD；列表每页 4 项。
// Checkpoints 页的 ids 是检查点结束帧数，包含 725（结尾），其他页为宏编号。
void initialize_screen();
// 重置按压计数，防止模式切换后旧触摸触发动作；非阻塞。
void reset_touch();
// 20ms 轮询新按下沿，返回屏幕请求；忙碌页仅允许停止，Result 页可重试失败保存。
Touch poll_touch(View view, const std::vector<int> &ids, std::size_t page, bool retry_save);
// 静态页面按需刷新；动态时间每 100ms 刷新（不影响 20ms 组件更新）。
void draw_screen(View view, const std::vector<int> &ids, std::size_t page, const Engine &engine,
                 std::uint32_t now_ms, int selected_id, const char *message, bool retry_save,
                 bool force = false);

} // namespace macrorec
