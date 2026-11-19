#include "calibration/screen.hpp"
#include "calibration/measurements.hpp"
#include "pros/screen.hpp"

#include <array>
#include <cstdint>
#include <cstdio>
#include <string_view>

namespace calibration {
namespace {

constexpr std::uint32_t kBackground = 0x101820;
constexpr std::uint32_t kText = 0xFFFFFF;
constexpr std::uint32_t kPanel = 0x23313D;
constexpr std::uint32_t kButton = 0x236A93;
constexpr std::uint32_t kAccent = 0x127766;
constexpr std::uint32_t kMuted = 0x7F99A9;
constexpr std::uint32_t kStop = 0xA53030;
constexpr std::size_t kItemsPerPage = 5;
constexpr auto kTestCount = static_cast<std::size_t>(Test::Count);
constexpr auto kPageCount = (kTestCount + kItemsPerPage - 1) / kItemsPerPage;

struct Rect {
    int left;
    int top;
    int right;
    int bottom;
};

struct Button {
    Rect rect;
    const char *label;
    Touch touch;
    std::uint32_t color = kButton;
};

constexpr Rect kFooterLeft{12, 202, 151, 236};
constexpr Rect kFooterMiddle{161, 202, 318, 236};
constexpr Rect kFooterRight{328, 202, 467, 236};
constexpr Rect kPromptBack{12, 198, 102, 236};
constexpr Rect kPromptConfirm{111, 198, 235, 236};
constexpr Rect kCancelBar{12, 197, 467, 236};

std::int32_t _g_press_count = 0;

void fill(const Rect &rect, std::uint32_t color) {
    pros::screen::set_pen(color);
    pros::screen::fill_rect(rect.left, rect.top, rect.right, rect.bottom);
}

void text(int x, int y, std::string_view label, std::uint32_t color = kText) {
    pros::screen::set_pen(color);
    pros::screen::print(pros::E_TEXT_SMALL, x, y, "%.*s", static_cast<int>(label.size()),
                        label.data());
}

void draw_button(const Button &button) {
    const auto &rect = button.rect;
    const bool enabled = button.touch.action != Action::None;
    fill(rect, enabled ? button.color : kPanel);
    text(rect.left + 12, rect.top + (rect.bottom - rect.top - 14) / 2, button.label,
         enabled ? kText : kMuted);
}

constexpr std::array<Button, 12> kKeypad{{
    {{}, "1", {Action::Digit, 1}},
    {{}, "2", {Action::Digit, 2}},
    {{}, "3", {Action::Digit, 3}},
    {{}, "4", {Action::Digit, 4}},
    {{}, "5", {Action::Digit, 5}},
    {{}, "6", {Action::Digit, 6}},
    {{}, "7", {Action::Digit, 7}},
    {{}, "8", {Action::Digit, 8}},
    {{}, "9", {Action::Digit, 9}},
    {{}, ".", {Action::Dot}},
    {{}, "0", {Action::Digit, 0}},
    {{}, "DEL", {Action::Backspace}},
}};

// 绘制和触摸复用同一组矩形，避免按钮与热区随页码或状态变化后错位。
void for_buttons(const Screen &screen, const auto &emit) {
    switch (screen.page) {
    case Page::Menu: {
        const auto first = screen.menu_page * kItemsPerPage;
        for (std::size_t row = 0; row < kItemsPerPage && first + row < kTestCount; ++row) {
            const auto index = first + row;
            const int y = 51 + static_cast<int>(row) * 29;
            emit(Button{{12, y, 467, y + 25},
                        recipe(static_cast<Test>(index)).title,
                        {Action::Select, static_cast<int>(index)}});
        }
        emit(Button{
            kFooterLeft, "< PREV", {screen.menu_page > 0 ? Action::Previous : Action::None}});
        emit(Button{kFooterMiddle, "HOME", {Action::Exit}});
        emit(Button{kFooterRight,
                    "NEXT >",
                    {screen.menu_page + 1 < kPageCount ? Action::Next : Action::None}});
        break;
    }
    case Page::Guide:
        emit(Button{kFooterLeft, "BACK", {Action::Menu}});
        emit(Button{kFooterRight,
                    "CONTINUE",
                    {screen.can_proceed ? Action::Proceed : Action::None},
                    kAccent});
        break;
    case Page::Entry:
        for (std::size_t index = 0; index < kKeypad.size(); ++index) {
            auto key = kKeypad[index];
            const int x = 251 + static_cast<int>(index % 3) * 74;
            const int y = 70 + static_cast<int>(index / 3) * 42;
            key.rect = {x, y, x + 67, y + 38};
            emit(key);
        }
        emit(Button{kPromptBack, "BACK", {Action::Menu}});
        emit(Button{kPromptConfirm,
                    "USE VALUE",
                    {screen.can_proceed ? Action::Accept : Action::None},
                    kAccent});
        break;
    case Page::Armed:
    case Page::Running:
        emit(Button{kCancelBar, "STOP / CANCEL", {Action::Cancel}, kStop});
        break;
    case Page::Result:
        emit(Button{kFooterMiddle, "BACK TO TESTS", {Action::Menu}});
        break;
    }
}

void draw_entry(const Screen &screen) {
    if (screen.lines.front()) {
        text(12, 44, std::string_view(screen.lines.front()).substr(0, 56));
    }
    int y = 71;
    for (std::size_t line = 1; line < screen.lines.size() && y < 143; ++line) {
        if (!screen.lines[line]) {
            continue;
        }
        std::string_view words = screen.lines[line];
        while (!words.empty() && y < 143) {
            constexpr std::size_t kColumns = 27;
            const auto split = words.size() > kColumns ? words.rfind(' ', kColumns) : words.size();
            const auto count = split == std::string_view::npos || split == 0 ? kColumns : split;
            text(12, y, words.substr(0, count), kMuted);
            words.remove_prefix(count);
            if (!words.empty() && words.front() == ' ') {
                words.remove_prefix(1);
            }
            y += 15;
        }
    }
    fill({12, 150, 235, 188}, kPanel);
    pros::screen::set_pen(kText);
    pros::screen::print(pros::E_TEXT_MEDIUM, 22, 159, "%.20s", screen.value ? screen.value : "");
}

} // namespace

void reset_screen_touch() {
    const auto touch = pros::screen::touch_status();
    if (touch.touch_status != pros::E_TOUCH_ERROR) {
        _g_press_count = touch.press_count;
    }
}

Touch poll_screen_touch(const Screen &screen) {
    const auto touch = pros::screen::touch_status();
    if (touch.touch_status == pros::E_TOUCH_ERROR || touch.press_count == _g_press_count) {
        return {};
    }
    _g_press_count = touch.press_count;
    Touch action{};
    for_buttons(screen, [&](const Button &button) {
        const auto &rect = button.rect;
        if (touch.x >= rect.left && touch.x <= rect.right && touch.y >= rect.top &&
            touch.y <= rect.bottom) {
            action = button.touch;
        }
    });
    return action;
}

void draw_screen(const Screen &screen) {
    pros::screen::set_eraser(kBackground);
    pros::screen::erase();
    pros::screen::set_pen(kText);
    pros::screen::print(pros::E_TEXT_MEDIUM, 12, 11, "%.38s",
                        screen.title ? screen.title : "CALIBRATION");
    fill({12, 35, 467, 36}, kAccent);
    if (screen.page == Page::Menu) {
        std::array<char, 16> progress{};
        std::snprintf(progress.data(), progress.size(), "%u / %u",
                      static_cast<unsigned>(screen.menu_page + 1),
                      static_cast<unsigned>(kPageCount));
        text(410, 16, progress.data(), kMuted);
    } else if (screen.page == Page::Entry) {
        draw_entry(screen);
    } else {
        int y = 48;
        for (const auto *line : screen.lines) {
            if (line) {
                text(12, y, std::string_view(line).substr(0, 56));
            }
            y += 24;
        }
    }
    for_buttons(screen, draw_button);
    pros::screen::set_pen(kText);
}

} // namespace calibration
