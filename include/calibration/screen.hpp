#pragma once

#include <array>
#include <cstddef>

namespace calibration {

enum class Page { Menu, Guide, Entry, Armed, Running, Result };
enum class Action {
    None,
    Exit,
    Menu,
    Previous,
    Next,
    Select,
    Proceed,
    Cancel,
    Digit,
    Dot,
    Backspace,
    Accept
};

struct Touch {
    Action action = Action::None;
    int value = 0;
};

// 文本仅在一次调用内读取。Entry 的首行跨屏显示，其余行在键盘左侧自动换行。
struct Screen {
    Page page = Page::Menu;
    const char *title = nullptr;
    std::array<const char *, 6> lines{};
    const char *value = nullptr;
    bool can_proceed = true;
    std::size_t menu_page = 0;
};

// 进入或恢复测量菜单时读取当前按压计数，避免点击从上一个界面穿透。
void reset_screen_touch();

// 按当前页返回一个新按压动作。Select 为 Test 下标，Digit 为 0–9。
Touch poll_screen_touch(const Screen &screen);

// 立即绘制当前页。刷新频率由拥有 Brain 屏幕的会话调度。
void draw_screen(const Screen &screen);

} // namespace calibration
