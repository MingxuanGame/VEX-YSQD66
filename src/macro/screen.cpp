#include "macro/screen.hpp"
#include "pros/screen.hpp"

#include <algorithm>
#include <array>
#include <cstdio>

namespace macrorec {
namespace {
constexpr std::uint32_t kBackground = 0x101820;
constexpr std::uint32_t kText = 0xFFFFFF;
constexpr std::uint32_t kButton = 0x236A93;
constexpr std::uint32_t kStop = 0xA53030;
int _g_press_count = 0;
bool _g_first_draw = true;
View _g_previous_view = View::Home;
std::uint32_t _g_draw_ms = 0;
std::array<std::uint32_t, 24 * 24> _g_glyph_pixels{};

// 中文按钮字形使用 Noto Sans CJK SC 24px 的静态位图，无需 SD 字库。
// 字形数据与许可见 src/macro/labels.inc、docs/licenses/NotoSansCJK-OFL.txt。
#include "labels.inc"

void rectangle(int x0, int y0, int x1, int y1, std::uint32_t color) {
    pros::screen::set_pen(color);
    pros::screen::fill_rect(x0, y0, x1, y1);
    pros::screen::set_pen(kText);
}

void label(int x, int y, const char *text) {
    pros::screen::print(pros::E_TEXT_SMALL, x, y, "%s", text);
}

void button(int x0, int y0, int x1, int y1, const char *text, bool stop = false) {
    rectangle(x0, y0, x1, y1, stop ? kStop : kButton);
    label(x0 + 12, y0 + (y1 - y0 - 14) / 2, text);
}

void chinese_label(int x, int y, const std::uint32_t *rows, std::size_t count) {
    for (std::size_t glyph = 0; glyph < count; ++glyph) {
        for (int row = 0; row < 24; ++row) {
            for (int col = 0; col < 24; ++col) {
                _g_glyph_pixels[row * 24 + col] =
                    (rows[glyph * 24 + row] & (1U << (23 - col))) ? kText : kButton;
            }
        }
        const auto left = static_cast<std::int16_t>(x + glyph * 24);
        pros::screen::copy_area(left, y, left + 23, y + 23, _g_glyph_pixels.data(), 24);
    }
}
} // namespace

void initialize_screen() {
    pros::screen::set_eraser(kBackground);
    pros::screen::set_pen(kText);
    reset_touch();
}

void reset_touch() {
    const auto touch = pros::screen::touch_status();
    if (touch.touch_status != pros::E_TOUCH_ERROR) {
        _g_press_count = touch.press_count;
    }
}

Touch poll_touch(View view, const std::vector<int> &ids, std::size_t page, bool retry_save) {
    const auto touch = pros::screen::touch_status();
    if (touch.touch_status == pros::E_TOUCH_ERROR || touch.press_count == _g_press_count) {
        return {};
    }
    _g_press_count = touch.press_count;
    const int x = touch.x;
    const int y = touch.y;
    if (x < 12 || x > 467) {
        return {};
    }
    if (view == View::Home) {
        if (y >= 90 && y <= 145) {
            return {TouchAction::NewRecording};
        }
        if (y >= 157 && y <= 212) {
            return {TouchAction::OpenList};
        }
    } else if (view == View::List) {
        if (y >= 69 && y < 193) {
            const auto index = page * 4 + static_cast<std::size_t>((y - 69) / 31);
            if (index < ids.size()) {
                return {TouchAction::Select, ids[index]};
            }
        }
        if (y >= 207 && y <= 238) {
            return {x < 160 ? TouchAction::Previous
                            : (x < 320 ? TouchAction::Home : TouchAction::Next)};
        }
    } else if (view == View::Running && y >= 185 && y <= 235) {
        return {TouchAction::Stop};
    } else if (view == View::Result) {
        if (retry_save && y >= 115 && y <= 160) {
            return {TouchAction::Retry};
        }
        if (y >= 185 && y <= 235) {
            return {TouchAction::Home};
        }
    }
    return {};
}

void draw_screen(View view, const std::vector<int> &ids, std::size_t page, const Engine &engine,
                 std::uint32_t now_ms, int selected_id, const char *message, bool retry_save,
                 bool force) {
    const bool full = force || _g_first_draw || view != _g_previous_view;
    if (!full && (view != View::Running || now_ms - _g_draw_ms < 100)) {
        return;
    }
    _g_draw_ms = now_ms;
    _g_previous_view = view;
    _g_first_draw = false;
    if (full) {
        pros::screen::erase();
        pros::screen::set_pen(kText);
        pros::screen::print(pros::E_TEXT_MEDIUM, 12, 12, "MACRO  #%06d", selected_id);
        pros::screen::print(pros::E_TEXT_SMALL, 12, 43, "%.57s", message);
    }
    switch (view) {
    case View::Home:
        rectangle(12, 90, 467, 145, kButton);
        chinese_label(40, 106, kNewLabel.data(), 5);
        label(210, 112, "NEW RECORDING");
        rectangle(12, 157, 467, 212, kButton);
        chinese_label(40, 173, kUseLabel.data(), 3);
        label(210, 179, "USE MACRO");
        break;
    case View::List:
        for (std::size_t row = 0; row < 4 && page * 4 + row < ids.size(); ++row) {
            std::array<char, 64> text{};
            std::snprintf(text.data(), text.size(), "MACRO #%06d    14.5s / 725 frames",
                          ids[page * 4 + row]);
            const int y = 69 + static_cast<int>(row) * 31;
            button(12, y, 467, y + 27, text.data());
        }
        button(12, 207, 155, 238, "< PREVIOUS");
        button(164, 207, 311, 238, "HOME");
        button(320, 207, 467, 238, "NEXT >");
        break;
    case View::Running: {
        rectangle(12, 68, 467, 169, kBackground);
        pros::screen::print(pros::E_TEXT_MEDIUM, 12, 72, "%s", engine.status());
        const auto elapsed = engine.elapsed_ms(now_ms);
        const auto total = engine.state() == State::Countdown   ? kCountdownMs
                           : engine.state() == State::Resetting ? kResetMs
                                                                : kRecordMs;
        pros::screen::print(pros::E_TEXT_LARGE, 12, 101, "%5.2fs / %5.2fs", elapsed / 1000.0,
                            total / 1000.0);
        pros::screen::print(pros::E_TEXT_SMALL, 12, 151, "%s | FRAMES %u / 725 | 20ms",
                            engine.recording() ? "RECORD" : "REPLAY",
                            static_cast<unsigned>(engine.frame_count()));
        if (full) {
            button(12, 185, 467, 235, "STOP / CANCEL", true);
        }
        break;
    }
    case View::Result:
        label(12, 78, "OUTPUT STOPPED");
        if (retry_save) {
            button(12, 115, 467, 160, "RETRY SD SAVE");
        }
        button(12, 185, 467, 235, retry_save ? "DISCARD / HOME" : "HOME");
        break;
    }
}

} // namespace macrorec
