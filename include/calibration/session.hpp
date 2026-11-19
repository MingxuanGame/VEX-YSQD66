#pragma once

#include "calibration/measurements.hpp"
#include "calibration/screen.hpp"
#include "calibration/trial.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace calibration {

// Brain 标定菜单和试跑共用手动主循环。进入界面后由此会话独占底盘输出。
class Session {
public:
    explicit Session(macrorec::Hardware &hardware);
    void open();
    void interrupt();
    bool active() const;

    // 每 20ms 调用。allowed 表示脱离场控且启用手动，试跑须持续按住 A。
    void tick(std::uint32_t now_ms, bool allowed, bool controller_connected, bool a_held);

private:
    void change_page(Page page);
    void handle(Touch touch);
    void begin_fields(bool after);
    void load_field();
    void accept_field();
    void prepare_trial();
    void update_trial(std::uint32_t now_ms, bool allowed, bool connected, bool a_held);
    void finish();
    void abort(const char *reason);
    Screen screen_layout() const;
    void render(std::uint32_t now_ms, bool allowed, bool connected);

    Trial m_trial;
    Page m_page = Page::Menu;
    Test m_test = Test::Travel;
    bool m_active = false;
    bool m_redraw = false;
    bool m_entering_after = false;
    bool m_a_released = false;
    bool m_entry_prefilled = false;
    std::optional<std::uint32_t> m_hold_since_ms;
    std::uint32_t m_rendered_ms = 0;
    std::size_t m_menu_page = 0;
    std::size_t m_field = 0;
    std::array<double, 8> m_before{};
    std::array<double, 8> m_after{};
    std::array<std::optional<double>, static_cast<std::size_t>(Remember::Count)> m_remembered{};
    std::array<std::array<char, 56>, 5> m_lines{};
    std::string m_entry;
    const char *m_notice = nullptr;
    Result m_result{};
};

} // namespace calibration
